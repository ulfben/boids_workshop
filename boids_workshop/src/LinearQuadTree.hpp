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
   using node_id = uint32_t; // index to a node in the 'nodes' vector
   using index_t = uint32_t; // index to an object in the original collection or data vector
   using count_t = uint32_t; // number of objects in a node or distance between two indexes.
   static constexpr node_id NoChild = static_cast<node_id>(-1);
   static constexpr node_id ROOT_ID = 0;

   static constexpr count_t distance(auto iter1, auto iter2){
      return static_cast<count_t>(std::distance(iter1, iter2));
   }

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
      node_id quads[4] = {NoChild, NoChild, NoChild, NoChild};
      
      constexpr bool is_leaf() const noexcept{
         return (quads[0] == NoChild && quads[1] == NoChild && quads[2] == NoChild && quads[3] == NoChild);
      }

      constexpr node_id& operator[](Quadrant quadrant) noexcept{
         return quads[std::to_underlying(quadrant)];
      }

      constexpr const node_id& operator[](Quadrant quadrant) const noexcept{
         return quads[std::to_underlying(quadrant)];
      }
   };

   std::vector<Node> nodes;      // linear storage for all nodes.
   std::vector<const T*> data;   // all object pointers stored contiguously by leaves.
   Rectangle boundary = {0, 0, 0, 0}; // boundary of the root node   
   count_t capacity = 8;   // objects per quad before subdivision
   count_t max_depth = 5;  // maximum depth allowed

   // Recursively builds the tree by partitioning the vector of pointers in-place.
   // 'data' vector holds the pointers; [start, end) is the subrange to work on.
   // 'bound' is the current node's boundary and 'depth' the current recursion depth.
   // Returns the index of the new node in the 'nodes' vector.
   node_id build_tree(index_t start, index_t end, const Rectangle& bound, count_t depth){
      assert(start < end);
      const auto nodeIndex = static_cast<node_id>(nodes.size());
      nodes.emplace_back(bound, start);
      Node& node = nodes.back();
            
      if(count_t count = end - start; 
         count <= capacity || depth >= max_depth){
         node.data_count = count;
         return nodeIndex;
      }
      // This node will not store any data; its children will. We split in four quadrants and partition the objects accordingly.      
      const Vector2 center = {bound.x + bound.width * 0.5f, bound.y + bound.height * 0.5f};
      const auto first = data.begin() + start;
      const auto last = data.begin() + end;
      
      // Partitioning by y-coordinate. All objects in the top half of the rect will be in the first half of 'data'
      const auto split_y = std::partition(first, last, [center](const T* p){
         return p->position.y < center.y;
      });
      const index_t idx_split_y = distance(data.begin(), split_y);

      // Partition the top half by x-coordinate. All object in the top-left quadrant will be in the first 25% of the container, followed by all objects in the top-right quadrant
      const auto split_x_left = std::partition(first, split_y, [center](const T* p){
         return p->position.x < center.x;
      });
      const index_t idx_split_x_left = distance(data.begin(), split_x_left);

      // Partition the bottom half by x-coordinate. All objects in the bottom-left quadrant will be in the 50%-75% range of the container, followed by all objects in the bottom-right quadrant
      const auto split_x_right = std::partition(split_y, last, [center](const T* p){
         return p->position.x < center.x;
      });
      const index_t idx_split_x_right = distance(data.begin(), split_x_right);

      const float x = bound.x;
      const float y = bound.y;
      const float w = bound.width * 0.5f;
      const float h = bound.height * 0.5f;

      if(idx_split_x_left > start){
         Rectangle top_left = {x, y, w, h};
         node[Quadrant::TopLeft] = build_tree(start, idx_split_x_left, top_left, depth + 1);
      }
      if(idx_split_y > idx_split_x_left){
         Rectangle top_right = {x + w, y, w, h};
         node[Quadrant::TopRight] = build_tree(idx_split_x_left, idx_split_y, top_right, depth + 1);
      }
      if(idx_split_x_right > idx_split_y){
         Rectangle bottom_left = {x, y + h, w, h};
         node[Quadrant::BottomLeft] = build_tree(idx_split_y, idx_split_x_right, bottom_left, depth + 1);
      }
      if(end > idx_split_x_right){
         Rectangle bottom_right = {x + w, y + h, w, h};
         node[Quadrant::BottomRight] = build_tree(idx_split_x_right, end, bottom_right, depth + 1);
      }
      return nodeIndex;
   }

   void query_range_recursive(node_id nodeIndex, const Rectangle& range, std::vector<const T*>& found) const{      
      if(nodeIndex == NoChild || nodeIndex >= nodes.size()){
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