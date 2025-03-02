#pragma once
#include "raylib.h"
#include <cassert>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

// LinearQuadTree stores nodes in a contiguous vector and organizes leaf data contiguously.
// based on Lisyarus' excellent article: https://lisyarus.github.io/blog/posts/building-a-quadtree.html

template<class T>
class LinearQuadTree{
   using node_idx = uint32_t; // index to a node in the 'nodes' vector
   using index_t = uint32_t; // index to an object in the original collection or data vector
   using count_t = uint32_t; // number of objects in a node or distance between two indexes.
   static constexpr node_idx NO_CHILD = static_cast<node_idx>(-1);
   static constexpr node_idx ROOT_ID = 0;

   enum class Quadrant : uint8_t{
      TopLeft = 0,
      TopRight = 1,
      BottomLeft = 2,
      BottomRight = 3
   };

   struct Node final{
      Rectangle boundary{0, 0, 0, 0};
      index_t data_begin = 0;  // starting index into the 'data' vector.
      count_t data_count = 0;  // number of objects stored in this node.      
      node_idx quads[4] = {NO_CHILD, NO_CHILD, NO_CHILD, NO_CHILD};

      constexpr bool is_leaf() const noexcept{
         return (quads[0] == NO_CHILD && quads[1] == NO_CHILD && quads[2] == NO_CHILD && quads[3] == NO_CHILD);
      }

      constexpr node_idx& operator[](Quadrant quadrant) noexcept{
         return quads[std::to_underlying(quadrant)];
      }

      constexpr const node_idx& operator[](Quadrant quadrant) const noexcept{
         return quads[std::to_underlying(quadrant)];
      }
   };

   std::vector<Node> nodes;      // linear storage for all nodes.
   std::vector<const T*> data;   // all object pointers stored contiguously by leaves.
   Rectangle boundary = {0, 0, 0, 0}; // boundary of the root node   
   count_t capacity = 8;   // objects per quad before subdivision
   count_t max_depth = 5;  // maximum depth allowed

   //Reorders elements in-place such that elements satisfying the predicate come before those that do not. 
   //The returned index is the boundary between these two groups.
   constexpr index_t partition_data(index_t start, index_t end, auto predicate) noexcept{
      auto iter = std::partition(data.begin() + start, data.begin() + end, predicate);
      return static_cast<index_t>(iter - data.begin());
   }

   // Recursively builds the tree by partitioning the 'data'-vector in-place.
   // 'data' holds all objects, [start, end) is the subrange to work on.
   // 'bound' is the current node's boundary and 'depth' the current recursion depth.
   // Returns the index of the new node in the 'nodes' vector.
   node_idx build_tree(index_t start, index_t end, const Rectangle& bound, count_t depth){
      assert(start < end);
      const auto nodeIndex = static_cast<node_idx>(nodes.size());
      nodes.emplace_back(bound, start);
      Node& node = nodes.back();

      if(count_t count = end - start;
         count <= capacity || depth >= max_depth){
         node.data_count = count;
         return nodeIndex;
      }
      // This node will not store any objects; its children will. 
      // So we subdivide into four quads, and arrange the objects in 'data' based on screen position.      
      
      // 1. Split vertically: objects in top half of 'bounds' will be in the first half of 'data', objects in bottom half go in second half   
      const Vector2 center = {bound.x + bound.width * 0.5f, bound.y + bound.height * 0.5f};
      const index_t idx_split_y = partition_data(start, end, [center](const T* p){
         return p->position.y < center.y;
         });
      
      // 2. Split top half horizontally: objects in top-left quadrant of 'bounds' go in first quarter of 'data', followed by objects in the top-right quadrant
      const index_t idx_split_x_left = partition_data(start, idx_split_y, [center](const T* p){
         return p->position.x < center.x;
         });
      
      // 3. Split bottom half horizontally: objects in bottom-left quadrant go in third quarter of 'data', followed by objects in the bottom-right quadrant
      const index_t idx_split_x_right = partition_data(idx_split_y, end, [center](const T* p){
         return p->position.x < center.x;
         });

      const float x = bound.x;
      const float y = bound.y;
      const float w = bound.width * 0.5f;
      const float h = bound.height * 0.5f;

      if(idx_split_x_left > start){
         Rectangle top_left = {x, y, w, h};
         node[Quadrant::TopLeft] = build_tree(start, idx_split_x_left, top_left, depth + 1); //first quarter
      }
      if(idx_split_y > idx_split_x_left){
         Rectangle top_right = {x + w, y, w, h};
         node[Quadrant::TopRight] = build_tree(idx_split_x_left, idx_split_y, top_right, depth + 1); //second quarter
      }
      if(idx_split_x_right > idx_split_y){
         Rectangle bottom_left = {x, y + h, w, h};
         node[Quadrant::BottomLeft] = build_tree(idx_split_y, idx_split_x_right, bottom_left, depth + 1); //third quarter
      }
      if(end > idx_split_x_right){
         Rectangle bottom_right = {x + w, y + h, w, h};
         node[Quadrant::BottomRight] = build_tree(idx_split_x_right, end, bottom_right, depth + 1); //fourth quarter
      }
      return nodeIndex;
   }

   void query_range_recursive(node_idx nodeIndex, const Rectangle& range, std::vector<const T*>& found) const{
      if(nodeIndex == NO_CHILD || nodeIndex >= nodes.size()){
         return;
      }
      const Node& node = nodes[nodeIndex];
      if(!CheckCollisionRecs(node.boundary, range)){
         return;
      }
      if(node.is_leaf()){
         for(count_t i = 0; i < node.data_count; i++){
            const T* obj = data[node.data_begin + i];
            if(CheckCollisionPointRec(obj->position, range)){
               found.push_back(obj);
            }
         }
         return;
      }
      query_range_recursive(node[Quadrant::TopLeft], range, found);
      query_range_recursive(node[Quadrant::TopRight], range, found);
      query_range_recursive(node[Quadrant::BottomLeft], range, found);
      query_range_recursive(node[Quadrant::BottomRight], range, found);
   }

   static Rectangle compute_bounds_of(std::span<const T> objects) noexcept{
      if(objects.empty()){ return {0, 0, 0, 0}; }
      auto [min_x, min_y] = objects[0].position;
      auto [max_x, max_y] = objects[0].position;
      for(size_t i = 1; i < objects.size(); ++i){
         const auto& pos = objects[i].position;
         if(pos.x < min_x) min_x = pos.x;
         if(pos.x > max_x) max_x = pos.x;
         if(pos.y < min_y) min_y = pos.y;
         if(pos.y > max_y) max_y = pos.y;
      }
      const float padding = 1.0f; // Add padding to avoid objects exactly at the boundary
      return {
          min_x - padding,
          min_y - padding,
          (max_x - min_x) + 2 * padding,
          (max_y - min_y) + 2 * padding
      };
   }

public:
   LinearQuadTree() = default;
   LinearQuadTree(const Rectangle& boundary_, std::span<const T> objects, count_t capacity_, count_t max_depth_ = 5)
      : boundary(boundary_), capacity(capacity_), max_depth(max_depth_){
      assert(capacity > 0);
      assert(max_depth > 0);
      rebuild(objects);
   }

   LinearQuadTree(std::span<const T> objects, count_t capacity_, count_t max_depth_ = 5)
      : LinearQuadTree(compute_bounds_of(objects), objects, capacity_, max_depth_){}

   void rebuild(std::span<const T> objects){
      nodes.clear();
      data.clear();
      if(objects.empty()){ return; }

      data.reserve(objects.size());
      for(auto& obj : objects){ //NOTE: if objects are guarantueed to be within the bounds, you can skip this filtering!
         if(CheckCollisionPointRec(obj.position, boundary)){
            data.push_back(std::addressof(obj));
         }
      }
      if(data.empty()){ return; }
      build_tree(0, static_cast<index_t>(data.size()), boundary, 0);
   }

   void rebuild_and_fit_to(std::span<const T> objects){
      boundary = compute_bounds_of(objects);
      rebuild(objects);
   }

   void render() const{
      for(const auto& node : nodes){
         DrawRectangleLinesEx(node.boundary, 1, GREEN);
      }
   }

   void query_range(const Rectangle& range, std::vector<const T*>& found) const{
      query_range_recursive(ROOT_ID, range, found);
   }
};