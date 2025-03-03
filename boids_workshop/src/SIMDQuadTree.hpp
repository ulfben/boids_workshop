#pragma once
#include "raylib.h"
#include <cassert>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>
#include <immintrin.h> // For SSE/AVX intrinsics

// Starting from the LinearQuadTree, this is an experiment in using SIMD instructions to speed up the quadtree.
template<class T>
class SIMDQuadTree{
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
   struct alignas(16) SIMDRect{
      static_assert(sizeof(SIMDRect) == 16, "SIMDRect should be 16 bytes for good SIMD performance");
      union{
#pragma warning(disable : 4201) //nonstandard extension used: nameless struct/union. TODO: fix the problem instead of suppressing the warning...
         struct{
            float minX, minY; //more suitable representation of a rect
            float maxX, maxY;
         };
#pragma warning(default : 4201)
         __m128 vec; // SSE vector for SIMD operations
      };

      constexpr SIMDRect() noexcept : minX(0), minY(0), maxX(0), maxY(0){}
      explicit constexpr SIMDRect(const Rectangle& r) noexcept :
         minX(r.x), minY(r.y),
         maxX(r.x + r.width), maxY(r.y + r.height){}

      constexpr Rectangle to_rect() const noexcept{
         return {minX, minY, maxX - minX, maxY - minY};
      }
   };

   struct Node final{
      SIMDRect boundary;
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

      constexpr Rectangle to_rect() const noexcept{
         return boundary.to_rect();
      }
   };

   std::vector<Node> nodes;      // linear storage for all nodes.
   std::vector<const T*> data;   // all object pointers stored contiguously by leaves.
   SIMDRect boundary;      // boundary of the root node   
   count_t capacity = 8;   // objects per quad before subdivision
   count_t max_depth = 5;  // maximum depth allowed

   static bool CheckCollisionRectsSIMD(const SIMDRect& r1, const SIMDRect& r2) noexcept{
#if defined(__AVX__) || defined(__AVX2__) //TODO: something more cross plattform would be nice. '__SSE4_1__' seems to be standard elsewhere.
      // Load rectangle bounds into SSE registers
      __m128 r1v = r1.vec;
      __m128 r2v = r2.vec;
      __m128 r2v_shuf = _mm_shuffle_ps(r2v, r2v, _MM_SHUFFLE(1, 0, 3, 2)); // Shuffle r2 to get [maxX, maxY, minX, minY]

      // Compare r1.min <= r2.max && r1.max >= r2.min
      // Now we have [r1.minX <= r2.maxX, r1.minY <= r2.maxY, r1.maxX >= r2.minX, r1.maxY >= r2.minY]
      __m128 cmp1 = _mm_cmple_ps(r1v, _mm_shuffle_ps(r2v_shuf, r2v_shuf, _MM_SHUFFLE(1, 0, 1, 0)));
      __m128 cmp2 = _mm_cmpge_ps(_mm_shuffle_ps(r1v, r1v, _MM_SHUFFLE(3, 2, 3, 2)), r2v_shuf);

      // Combine comparisons with bitwise AND
      __m128 result = _mm_and_ps(cmp1, cmp2);

      // Extract the first and second elements, then the third and fourth elements
      int res1 = _mm_extract_ps(result, 0) & _mm_extract_ps(result, 1);
      int res2 = _mm_extract_ps(result, 2) & _mm_extract_ps(result, 3);

      // Final result:
      int mask = _mm_movemask_ps(result);
      return (mask == 0b1111); // All four conditions must be true.
#else 
      // non-simd fallback      
      return (r1.minX <= r2.maxX && r1.maxX >= r2.minX && r1.minY <= r2.maxY && r1.maxY >= r2.minY);
#endif
   }

   // Helper to check if a point is inside a rectangle
   static bool CheckCollisionPointRectSIMD(const Vector2& point, const SIMDRect& rect) noexcept{
      return (point.x >= rect.minX && point.x <= rect.maxX &&
         point.y >= rect.minY && point.y <= rect.maxY);
   }

   //helper function to run std::partition and convert the boundary iterator to an index.
   //partition reorders elements in-place such that elements satisfying the predicate come before those that do not. 
   //the returned index is the boundary between these two groups.
   constexpr index_t partition_data(index_t start, index_t end, auto predicate) noexcept{
      auto iter = std::partition(data.begin() + start, data.begin() + end, predicate);
      return static_cast<index_t>(iter - data.begin());
   }

   // Helper function to build a child quad. Extracted to reduce the length of the build_tree function.
   // Returns the node index of the created child, or NO_CHILD if no child was created
   constexpr node_idx build_child_quad(index_t start, index_t end, const SIMDRect& bound, count_t depth){
      if(start < end){
         return build_tree(start, end, bound, depth + 1);
      }
      return NO_CHILD;
   }

   // Recursively builds the tree by partitioning the 'data'-vector in-place.
   // 'data' holds all objects, [start, end) is the subrange to work on.
   // 'bound' is the current node's boundary and 'depth' the current recursion depth.
   // Returns the index of the new node in the 'nodes' vector.
   node_idx build_tree(index_t start, index_t end, const SIMDRect& bound, count_t depth){
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
      const float center_x = (bound.minX + bound.maxX) * 0.5f;
      const float center_y = (bound.minY + bound.maxY) * 0.5f;
      const Vector2 center = {center_x, center_y};
      const auto to_the_left = [center](const T* p) noexcept{ return p->position.x < center.x; };
      const auto to_the_top = [center](const T* p) noexcept{ return p->position.y < center.y; };
      // time to partition the [start, end) subrange inside the 'data' vector;
      // 1. Split vertically: objects in top half of 'bounds' will be in the first half of the range, objects in bottom half go in second half   
      const index_t idx_split_y = partition_data(start, end, to_the_top);
      // 2. Split top half horizontally: objects in top-left quadrant of 'bounds' go in first quarter of the range, followed by objects in the top-right quadrant
      const index_t idx_split_x_left = partition_data(start, idx_split_y, to_the_left);
      // 3. Split bottom half horizontally: objects in bottom-left quadrant go in third quarter of the range, followed by objects in the bottom-right quadrant
      const index_t idx_split_x_right = partition_data(idx_split_y, end, to_the_left);

        // Create child boundaries
      SIMDRect top_left = Rectangle{bound.minX, bound.minY, center_x - bound.minX, center_y - bound.minY};
      SIMDRect top_right = Rectangle{center_x, bound.minY, bound.maxX - center_x, center_y - bound.minY};
      SIMDRect bottom_left = Rectangle{bound.minX, center_y, center_x - bound.minX, bound.maxY - center_y};
      SIMDRect bottom_right = Rectangle{center_x, center_y, bound.maxX - center_x, bound.maxY - center_y};

      node[Quadrant::TopLeft] = build_child_quad(start, idx_split_x_left, top_left, depth);
      node[Quadrant::TopRight] = build_child_quad(idx_split_x_left, idx_split_y, top_right, depth);
      node[Quadrant::BottomLeft] = build_child_quad(idx_split_y, idx_split_x_right, bottom_left, depth);
      node[Quadrant::BottomRight] = build_child_quad(idx_split_x_right, end, bottom_right, depth);
      return nodeIndex;
   }

   void query_range_recursive(node_idx nodeIndex, const SIMDRect& range, std::vector<const T*>& found) const{
      if(nodeIndex == NO_CHILD || nodeIndex >= nodes.size()){
         return;
      }
      const Node& node = nodes[nodeIndex];
      if(!CheckCollisionRectsSIMD(node.boundary, range)){
         return;
      }
      if(node.is_leaf()){
         for(count_t i = 0; i < node.data_count; i++){
            const T* obj = data[node.data_begin + i];
            if(CheckCollisionPointRectSIMD(obj->position, range)){
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

   static SIMDRect compute_bounds_of(std::span<const T> objects) noexcept{
      if(objects.empty()){ return {}; }
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
      return {Rectangle{
          min_x - padding,
          min_y - padding,
          (max_x - min_x) + 2 * padding,
          (max_y - min_y) + 2 * padding
      }};
   }

public:
   SIMDQuadTree() = default;
   SIMDQuadTree(const Rectangle& boundary_, std::span<const T> objects, count_t capacity_, count_t max_depth_ = 5)
      : boundary(boundary_), capacity(capacity_), max_depth(max_depth_){
      assert(capacity > 0);
      assert(max_depth > 0);
      rebuild(objects);
   }

   SIMDQuadTree(std::span<const T> objects, count_t capacity_, count_t max_depth_ = 5)
      : SIMDQuadTree(compute_bounds_of(objects), objects, capacity_, max_depth_){}

   void rebuild(std::span<const T> objects){
      nodes.clear();
      data.clear();
      if(objects.empty()){ return; }
      data.reserve(objects.size());
      for(auto& obj : objects){ //NOTE: if objects are guarantueed to be within the bounds, you can skip this filtering!
         if(CheckCollisionPointRectSIMD(obj.position, boundary)){
            data.push_back(std::addressof(obj));
         }
      }
      if(data.empty()){ return; }
      nodes.reserve(data.size() / (capacity / 2)); // just a rough estimate, but might save a few re-allocations.
      build_tree(0, static_cast<index_t>(data.size()), boundary, 0);
   }

   void rebuild_and_fit_to(std::span<const T> objects){
      boundary = compute_bounds_of(objects);
      rebuild(objects);
   }

   void render() const noexcept{
      for(const auto& node : nodes){         
         DrawRectangleLinesEx(node.to_rect(), 1, GREEN);
      }
   }

   void query_range(const Rectangle& range, std::vector<const T*>& found) const{
      query_range_recursive(ROOT_ID, range, found);
   }
};