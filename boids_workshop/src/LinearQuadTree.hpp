#pragma once
#include "raylib.h"
#include <vector>
#include <span>
#include <algorithm>
#include <cassert>
#include <array>
#include <cstdint>
#include <iterator>

// LinearQuadTree stores nodes in a contiguous vector and organizes leaf data contiguously.
// based on Lisyarus' article: https://lisyarus.github.io/blog/posts/building-a-quadtree.html

template<class T>
class LinearQuadTree{
   using node_id = uint32_t;
   static constexpr node_id NoChild = static_cast<node_id>(-1);

   enum class Quadrant : uint8_t{
      TopLeft = 0,
      TopRight = 1,
      BottomLeft = 2,
      BottomRight = 3
   };

   struct Node final{
      Rectangle boundary{0, 0, 0, 0};
      std::array<node_id, 4> child = {{ NoChild, NoChild, NoChild, NoChild }};
      size_t data_begin = 0;  // starting index into the 'data' vector.
      size_t data_count = 0;  // number of objects stored in this node.

      constexpr bool is_leaf() const noexcept{
         return (child[0] == NoChild && child[1] == NoChild &&
            child[2] == NoChild && child[3] == NoChild);
      }

      constexpr node_id& operator[](Quadrant quadrant) noexcept{
         return child[std::to_underlying(quadrant)];
      }

      constexpr const node_id& operator[](Quadrant quadrant) const noexcept{
         return child[std::to_underlying(quadrant)];
      }
   };

   std::vector<Node> nodes;      // linear storage for all nodes.
   std::vector<const T*> data;   // all object pointers stored contiguously by leaves.
   Rectangle boundary = {0, 0, 0, 0};
   node_id root_index = NoChild;
   size_t capacity = 8;   // objects per quad before subdivision
   size_t max_depth = 5;  // maximum depth allowed

   // Recursively builds the tree by partitioning the vector of pointers in-place.
   // 'objects' holds the pointers; [start, end) is the subrange to work on.
   // 'bound' is the current node's boundary and 'depth' the current recursion depth.
   node_id build_tree(std::vector<const T*>& objects, size_t start, size_t end,
      const Rectangle& bound, int depth){
      assert(start < end);
      const auto nodeIndex = static_cast<node_id>(nodes.size());
      nodes.push_back({bound});
      Node& node = nodes.back();

      const size_t count = end - start;
      const auto first = objects.begin() + start;
      const auto last = objects.begin() + end;

      if(count <= capacity || depth >= static_cast<int>(max_depth)){
         node.data_begin = data.size();
         node.data_count = count;
         data.insert(data.end(), first, last);
         return nodeIndex;
      }

      // This node will not store any data; its children will.
      node.data_begin = data.size();
      node.data_count = 0;

      // Partitioning by y-coordinate
      const Vector2 center = {bound.x + bound.width * 0.5f, bound.y + bound.height * 0.5f};
      const auto split_y = std::partition(first, last, [center](const T* p){
         return p->position.y < center.y;
         });
      const size_t idx_split_y = std::distance(objects.begin(), split_y);

      // Partition the top half by x-coordinate:
      const auto split_x_left = std::partition(first, split_y, [center](const T* p){
         return p->position.x < center.x;
         });
      const size_t idx_split_x_left = std::distance(objects.begin(), split_x_left);

      // Partition the bottom half by x-coordinate:
      const auto split_x_right = std::partition(split_y, last, [center](const T* p){
         return p->position.x < center.x;
         });
      const size_t idx_split_x_right = std::distance(objects.begin(), split_x_right);

      const float x = bound.x;
      const float y = bound.y;
      const float w = bound.width * 0.5f;
      const float h = bound.height * 0.5f;

      if(idx_split_x_left > start){
         Rectangle top_left = {x, y, w, h};
         node[Quadrant::TopLeft] = build_tree(objects, start, idx_split_x_left, top_left, depth + 1);
      }
      if(idx_split_y > idx_split_x_left){
         Rectangle top_right = {x + w, y, w, h};
         node[Quadrant::TopRight] = build_tree(objects, idx_split_x_left, idx_split_y, top_right, depth + 1);
      }
      if(idx_split_x_right > idx_split_y){
         Rectangle bottom_left = {x, y + h, w, h};
         node[Quadrant::BottomLeft] = build_tree(objects, idx_split_y, idx_split_x_right, bottom_left, depth + 1);
      }
      if(end > idx_split_x_right){
         Rectangle bottom_right = {x + w, y + h, w, h};
         node[Quadrant::BottomRight] = build_tree(objects, idx_split_x_right, end, bottom_right, depth + 1);
      }
      return nodeIndex;
   }

   void query_range_recursive(node_id nodeIndex, const Rectangle& range,
      std::vector<const T*>& found) const{
      if(nodeIndex == NoChild || nodeIndex >= nodes.size()){
         return;
      }
      const Node& node = nodes[nodeIndex];
      if(!CheckCollisionRecs(node.boundary, range)){
         return;
      }

      if(node.is_leaf()){
         for(size_t i = 0; i < node.data_count; i++){
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
      float min_x = objects[0].position.x;
      float max_x = objects[0].position.x;
      float min_y = objects[0].position.y;
      float max_y = objects[0].position.y;
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

   LinearQuadTree(const Rectangle& boundary_, std::span<const T> objects, size_t capacity_, size_t max_depth_ = 5)
      : boundary(boundary_), capacity(capacity_), max_depth(max_depth_){
      assert(capacity > 0);
      assert(max_depth > 0);
      rebuild(objects);
   }

   LinearQuadTree(std::span<const T> objects, size_t capacity_, size_t max_depth_ = 5)
      : LinearQuadTree(compute_bounds_of(objects), objects, capacity_, max_depth_){}

   bool rebuild(std::span<const T> objects){
      if(objects.empty()) return false;
      nodes.clear();
      data.clear();

      std::vector<const T*> objectPtrs; //TODO: I don't love this temporary allocation.
      objectPtrs.reserve(objects.size());
      for(auto& obj : objects){
         if(CheckCollisionPointRec(obj.position, boundary)){
            objectPtrs.push_back(std::addressof(obj));
         }         
      }
      root_index = build_tree(objectPtrs, 0, objectPtrs.size(), boundary, 0);
      return true;
   }

   bool rebuild_and_fit_to(std::span<const T> objects){
      assert(!objects.empty() && "Cannot resize boundary with an empty collection");
      boundary = compute_bounds_of(objects);
      return rebuild(objects);
   }

   void render() const{
      for(const auto& node : nodes){
         DrawRectangleLinesEx(node.boundary, 1, GREEN);
      }
   }

   void query_range(const Rectangle& range, std::vector<const T*>& found) const{
      query_range_recursive(root_index, range, found);
   }
};