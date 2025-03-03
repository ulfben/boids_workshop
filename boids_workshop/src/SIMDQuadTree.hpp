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
      union{
#pragma warning(disable : 4201) //nonstandard extension used: nameless struct/union. TODO: fix the problem instead of suppressing the warning...
         struct{
            float minX, minY; //more suitable representation of a rect
            float maxX, maxY;
         };
#pragma warning(default : 4201)
         __m128 vec; // SSE vector for SIMD operations
      };
      SIMDRect() : minX(0), minY(0), maxX(0), maxY(0){}      
      SIMDRect(const Rectangle& r) : minX(r.x), minY(r.y), maxX(r.x + r.width), maxY(r.y + r.height){}      
      SIMDRect(float min_x, float min_y, float max_x, float max_y) : minX(min_x), minY(min_y), maxX(max_x), maxY(max_y) {}

      constexpr Rectangle to_rect() const noexcept{
         return {minX, minY, maxX - minX, maxY - minY};
      }
   };
   static_assert(sizeof(SIMDRect) == 16, "SIMDRect should be 16 bytes for good SIMD performance");

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
#if defined(__AVX__) || defined(__AVX2__) 
    // Load rectangle bounds into SSE registers
      __m128 r1v = r1.vec;
      __m128 r2v = r2.vec;
      // Extract min and max values from r2
      __m128 r2_min = _mm_shuffle_ps(r2v, r2v, _MM_SHUFFLE(1, 0, 1, 0)); // [minY, minX, minY, minX]
      __m128 r2_max = _mm_shuffle_ps(r2v, r2v, _MM_SHUFFLE(3, 2, 3, 2)); // [maxY, maxX, maxY, maxX]

      // Extract min and max values from r1
      __m128 r1_min = _mm_shuffle_ps(r1v, r1v, _MM_SHUFFLE(1, 0, 1, 0)); // [minY, minX, minY, minX]
      __m128 r1_max = _mm_shuffle_ps(r1v, r1v, _MM_SHUFFLE(3, 2, 3, 2)); // [maxY, maxX, maxY, maxX]

      // Perform comparisons
      __m128 cmp_min = _mm_cmple_ps(r1_min, r2_max); // r1.minX <= r2.maxX, r1.minY <= r2.maxY
      __m128 cmp_max = _mm_cmpge_ps(r1_max, r2_min); // r1.maxX >= r2.minX, r1.maxY >= r2.minY

      __m128 result = _mm_and_ps(cmp_min, cmp_max); // Combine results
      // Check if all four conditions are met
      return (_mm_movemask_ps(result) == 0b1111); // 0b1111 = all true
#else    
      return (r1.minX <= r2.maxX && r1.maxX >= r2.minX &&
         r1.minY <= r2.maxY && r1.maxY >= r2.minY);
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

      // Initialize min/max vectors with the highest/lowest possible values
      __m128 min_vec = _mm_set1_ps(FLT_MAX);
      __m128 max_vec = _mm_set1_ps(-FLT_MAX);

      const size_t batch_size = 4;
      size_t i = 0;

      // Process objects in batches of 4
      for(; i + batch_size <= objects.size(); i += batch_size){
         float batch_x[4], batch_y[4];

         // Load 4 objects into arrays
         for(size_t j = 0; j < batch_size; j++){
            batch_x[j] = objects[i + j].position.x;
            batch_y[j] = objects[i + j].position.y;
         }

         // Load into SIMD registers
         __m128 x_vec = _mm_loadu_ps(batch_x);
         __m128 y_vec = _mm_loadu_ps(batch_y);

         // Interleave x and y
         __m128 xy_low = _mm_unpacklo_ps(x_vec, y_vec);  // [x0, y0, x1, y1]
         __m128 xy_high = _mm_unpackhi_ps(x_vec, y_vec); // [x2, y2, x3, y3]

         // Update min/max
         min_vec = _mm_min_ps(min_vec, xy_low);
         min_vec = _mm_min_ps(min_vec, xy_high);
         max_vec = _mm_max_ps(max_vec, xy_low);
         max_vec = _mm_max_ps(max_vec, xy_high);
      }

      // Process remaining objects
      for(; i < objects.size(); i++){
         __m128 pos_vec = _mm_setr_ps(objects[i].position.x, objects[i].position.y, objects[i].position.x, objects[i].position.y);
         min_vec = _mm_min_ps(min_vec, pos_vec);
         max_vec = _mm_max_ps(max_vec, pos_vec);
      }

      // Horizontal reduction to extract min_x, min_y, max_x, max_y
      __m128 min_shuf = _mm_shuffle_ps(min_vec, min_vec, _MM_SHUFFLE(2, 3, 0, 1)); // Swap pairs
      __m128 max_shuf = _mm_shuffle_ps(max_vec, max_vec, _MM_SHUFFLE(2, 3, 0, 1));

      min_vec = _mm_min_ps(min_vec, min_shuf);
      max_vec = _mm_max_ps(max_vec, max_shuf);

      float min_values[4], max_values[4];
      _mm_storeu_ps(min_values, min_vec);
      _mm_storeu_ps(max_values, max_vec);

      float min_x = std::min(min_values[0], min_values[1]);
      float min_y = std::min(min_values[2], min_values[3]);
      float max_x = std::max(max_values[0], max_values[1]);
      float max_y = std::max(max_values[2], max_values[3]);

      const float padding = 1.0f;
      return SIMDRect(min_x - padding, min_y - padding, max_x + padding, max_y + padding);
   }


public:
   SIMDQuadTree() = default;
   SIMDQuadTree(const Rectangle& boundary_, std::span<const T> objects, count_t capacity_, count_t max_depth_ = 5)
      : boundary(boundary_), capacity(capacity_), max_depth(max_depth_){
      assert(capacity > 0);
      assert(max_depth > 0);
      rebuild(objects);
   }
   SIMDQuadTree(const SIMDRect& boundary_, std::span<const T> objects, count_t capacity_, count_t max_depth_ = 5)
      : SIMDQuadTree(boundary_.to_rect(), objects, capacity_, max_depth_){};

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
      query_range_recursive(ROOT_ID, SIMDRect(range), found);
   }
};