/**
 * Boids Workshop
 * -------------
 * This code was written for educational purposes.
 * Original repository: https://github.com/ulfben/boids_workshop/
 * 
 * License:
 * This code is released under a permissive, attribution-friendly license.
 * You are free to use, modify, and distribute it for any purpose - personal, 
 * educational, or commercial.
 * 
 * While not required, attribution with a link back to the original repository 
 * is appreciated if you find this code useful.
 * 
 * Copyright (c) 2025, Ulf Benjaminsson
 */
#pragma once
#include "raylib.h"
#include <cassert>
#include <memory>
#include <vector>
#include <span>

template<class T>
class QuadTree{
   static constexpr size_t MAX_DEPTH = 8;
   Rectangle boundary = {};
   size_t capacity = 0;
   size_t depth = 0;
   std::vector<const T*> objects{};
   bool subdivided = false;
   std::unique_ptr<QuadTree> north_west{nullptr};
   std::unique_ptr<QuadTree> north_east{nullptr};
   std::unique_ptr<QuadTree> south_west{nullptr};
   std::unique_ptr<QuadTree> south_east{nullptr};

   void subdivide(){
      float x = boundary.x;
      float y = boundary.y;
      float w = boundary.width / 2.0f;
      float h = boundary.height / 2.0f;

      Rectangle nw_rect = {x, y, w, h};
      north_west = std::make_unique<QuadTree>(nw_rect, capacity, depth + 1);

      Rectangle ne_rect = {x + w, y, w, h};
      north_east = std::make_unique<QuadTree>(ne_rect, capacity, depth + 1);

      Rectangle sw_rect = {x, y + h, w, h};
      south_west = std::make_unique<QuadTree>(sw_rect, capacity, depth + 1);

      Rectangle se_rect = {x + w, y + h, w, h};
      south_east = std::make_unique<QuadTree>(se_rect, capacity, depth + 1);

      subdivided = true;
   }

public:
   QuadTree(const Rectangle& boundary, size_t capacity, size_t depth = 0)
      : boundary(boundary), capacity(capacity), depth(depth){
      assert(depth <= MAX_DEPTH);
      assert(capacity > 0);
   }

   bool insert(std::span<const T> values){
      bool all_inserted = true;      
      for(auto& obj : values){          
         all_inserted &= insert(&obj);            
      }
      return all_inserted;
   }

   bool insert(const T* obj){
      if(!CheckCollisionPointRec(obj->position, boundary)){
         return false;
      }

      if(objects.size() < capacity || depth >= MAX_DEPTH){
         objects.push_back(obj);
         return true;
      }

      if(!subdivided){
         subdivide();
      }
      if(north_west->insert(obj)) return true;
      if(north_east->insert(obj)) return true;
      if(south_west->insert(obj)) return true;
      if(south_east->insert(obj)) return true;

      assert(false && "should not happen if obj is within the boundary");
      return false;
   }

   void query_range(const Rectangle& range, std::vector<const T*>& found) const{
      if(!CheckCollisionRecs(boundary, range)){
         return;
      }
      for(auto obj : objects){
         if(CheckCollisionPointRec(obj->position, range)){
            found.push_back(obj);
         }
      }
      if(subdivided){
         north_west->query_range(range, found);
         north_east->query_range(range, found);
         south_west->query_range(range, found);
         south_east->query_range(range, found);
      }
   }

   void clear() noexcept{
      objects.clear();
      if(subdivided){
         north_west->clear();
         north_east->clear();
         south_west->clear();
         south_east->clear();
         north_west.reset(nullptr);
         north_east.reset(nullptr);
         south_west.reset(nullptr);
         south_east.reset(nullptr);
         subdivided = false;
      }
   }
   void render() const noexcept{
      DrawRectangleLinesEx(boundary, 1, GREEN);
      if(subdivided){
         north_west->render();
         north_east->render();
         south_west->render();
         south_east->render();
      }
   }
};