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

#include "raylib.h"
#include "raymath.h"
#include "Slider.h"
#include <array>
#include <cmath>
#include <cstdlib>
#include <span>
#include <string_view>
#include <vector>
#include "QuadTree.h"
#include "LinearQuadTree.hpp"
#include "SIMDQuadTree.hpp"

constexpr int STAGE_WIDTH = 1280;
constexpr int STAGE_HEIGHT = 720;
constexpr Vector2 STAGE_SIZE = {static_cast<float>(STAGE_WIDTH), static_cast<float>(STAGE_HEIGHT)};
constexpr Rectangle STAGE_RECT = {0.0f, 0.0f, STAGE_SIZE.x, STAGE_SIZE.y};
constexpr Vector2 ZERO = {0.0f, 0.0f};
constexpr auto CLEAR_COLOR = WHITE;
constexpr float TO_RAD = DEG2RAD;
constexpr float TO_DEG = RAD2DEG;
constexpr int BOID_COUNT = 80;
constexpr int OBSTACLE_COUNT = 6;
constexpr int TARGET_FPS = 60;
constexpr int FONT_SIZE = 20;
struct Boid;
using Tree = SIMDQuadTree<Boid>; //typedef for quickly changing between QuadTree, LinearQuadTree, and SIMDQuadTree

constexpr static float to_float(int value) noexcept{
   return static_cast<float>(value);
}

constexpr static float range01() noexcept{
   constexpr auto RAND_MAXF = to_float(RAND_MAX);
   return to_float(GetRandomValue(0, RAND_MAX)) / RAND_MAXF;
}

constexpr static float unit_range() noexcept{
   return (range01() * 2.0f) - 1.0f;
}

constexpr static float random_range(float min, float max) noexcept{
   return min + ((max - min) * range01());
}

constexpr static Vector2 random_range(const Vector2& min, const Vector2& max) noexcept{
   return {
      random_range(min.x, max.x),
      random_range(min.y, max.y)
   };
}

static Vector2 vector_from_angle(float angle, float magnitude) noexcept{
   return {std::cos(angle) * magnitude, std::sin(angle) * magnitude};
}

constexpr static Vector2 world_wrap(Vector2 pos, Vector2 world_size) noexcept{
   if(pos.x > world_size.x) pos.x -= world_size.x;
   else if(pos.x < 0) pos.x += world_size.x;
   if(pos.y > world_size.y) pos.y -= world_size.y;
   else if(pos.y < 0) pos.y += world_size.y;
   return pos;
}
struct Obstacle final{
   Vector2 position = random_range({50.0f, 50.0f}, STAGE_SIZE);
   float radius = random_range(15.0f, 50.0f);
   Color color = BLUE;

   void render() const noexcept{
      DrawCircleV(position, radius, color);
   }
};

struct BoidConfig final{
   using Slider = Slider<float>;
   Color color = RED;
   float size = 8.0f;
   float vision_range = 100.0f;     // how far a boid “sees” others
   float cohesion_weight = 2.3f;    // strength of moving toward group center
   float alignment_weight = 1.5f;   // strength of matching speed and direction (eg: velocity) of group
   float separation_weight = 2.0f;  // strength of keeping distance
   float separation_range = 100.0f; // the distance at which separation kicks in. the closer they get, the stronger the force
   float drag = 0.01f;              // simple drag applied to the velocity
   float min_speed = 50.0f;
   float max_speed = 150.0f;
   float obstacle_avoidance_margin = 110.0f;
   float obstacle_avoidance_weight = 3.5f; // strength of avoiding obstacles
   float wander_distance = 50.0f;  // distance ahead of the boid to project the wander circle
   float wander_radius = 25.0f;    // size of the wander circle
   float wander_jitter = 30.0f * TO_RAD;  // how much the wander angle changes each tick, in radians
   float wander_weight = 1.3f;     // steering force weight for wander behavior
   float seek_weight = 1.2f;       // steering force weight for seek behavior

   std::array<Slider, 9> sliders{
       Slider{"Vision", &vision_range, 0.0f, 180.0f},
       Slider{"Separation weight", &separation_weight, 0.0f, 20.0f},
       Slider{"Separation range", &separation_range, min_speed, 180},
       Slider{"Obstacle weight", &obstacle_avoidance_weight, 0.0f, 20.0f},
       Slider{"Obstacle margin", &obstacle_avoidance_margin, size, 180.0f},
       Slider{"Alignment weight", &alignment_weight, 0.0f, 20.0f},
       Slider{"Cohesion weight", &cohesion_weight, 0.0f, 20.0f},
       Slider{"Wander weight", &wander_weight, 0.0f, 20.0f},
       Slider{"Wander jitter", &wander_jitter, 0.0f, 2.0f * PI} // 2PI radians is a full circle
   };

   void update() noexcept{
      auto y = 40.0f;
      for(auto& slider : sliders){
         slider.top(y);
         slider.update();
         y = slider.bottom();
      }
   }
   void render() const noexcept{
      for(const auto& slider : sliders){
         slider.render();
      }
   }
};

BoidConfig globalConfig{}; // default configuration for all boids

struct Boid final{
   Vector2 position = random_range(ZERO, STAGE_SIZE);
   Vector2 velocity = vector_from_angle(random_range(0.0f, 360.0f) * TO_RAD, globalConfig.min_speed);
   std::vector<const Boid*> visible_boids; // non-owning pointers to nearby boids
   float wander_angle = 0.0f; // Persistent wandering angle

   void update_visible_boids(const Tree& quad_tree){
      visible_boids.clear();
      quad_tree.query_range(nearby(), visible_boids);      
   }

   Rectangle nearby() const noexcept{
      return {position.x - globalConfig.vision_range, position.y - globalConfig.vision_range, globalConfig.vision_range * 2, globalConfig.vision_range * 2};
   }

   void update(float deltaTime, std::span<const Obstacle> obstacles) noexcept{
      Vector2 acceleration = {0, 0};
      acceleration += obstacle_avoidance(obstacles);
      acceleration += separation();
      acceleration += alignment();
      acceleration += cohesion();
      acceleration += wander();
      acceleration += drag();

      velocity += acceleration * deltaTime;
      velocity = Vector2ClampValue(velocity, globalConfig.min_speed, globalConfig.max_speed);

      position += velocity * deltaTime;
      position = world_wrap(position, STAGE_SIZE);
   }
      
   Vector2 obstacle_avoidance(std::span<const Obstacle> obstacles) const noexcept{
      Vector2 steer{0, 0};
      int count = 0;
      for(const auto& obs : obstacles){
         float safe_distance = obs.radius + globalConfig.obstacle_avoidance_margin;
         float to_index = Vector2Distance(position, obs.position);
         if(to_index < safe_distance){
            Vector2 away = Vector2Normalize(position - obs.position);
            // Scale the force by how deep the boid is within the safe distance.
            steer += away * (safe_distance - to_index);
            ++count;
         }
      }
      if(count == 0){ return ZERO; }
      return (steer / to_float(count)) * globalConfig.obstacle_avoidance_weight;
   }

   Vector2 seek(Vector2 targetPos) const noexcept{
      auto toward = Vector2Normalize(targetPos - position);
      auto desired_velocity = toward * globalConfig.max_speed;
      return (desired_velocity - velocity) * globalConfig.seek_weight;
   }

   Vector2 wander() noexcept{
      Vector2 circle_center = Vector2Normalize(velocity) * globalConfig.wander_distance;
      wander_angle += unit_range() * globalConfig.wander_jitter;
      Vector2 displacement = {
          cos(wander_angle) * globalConfig.wander_radius,
          sin(wander_angle) * globalConfig.wander_radius
      };
      Vector2 wanderTarget = position + circle_center + displacement;
      return seek(wanderTarget) * globalConfig.wander_weight;
   }

   Vector2 separation() const noexcept{
      Vector2 steer{0, 0};
      int count = 0;
      for(auto other : visible_boids){
         Vector2 offset = position - other->position;
         float to_index = Vector2Length(offset);
         if(to_index < globalConfig.separation_range){
            steer += Vector2Normalize(offset) * (globalConfig.separation_range - to_index); // normalize a vector pointing away from other, and scale it by the inverse of the distance
            ++count;
         }
      }
      if(count == 0){ return ZERO; }
      return (steer / to_float(count)) * globalConfig.separation_weight; // average the contributions from all neighbors, and scale by separation weight
   }

   Vector2 alignment() const noexcept{
      Vector2 sum{0, 0};
      int count = 0;
      for(auto other : visible_boids){
         sum += other->velocity;
         count++;
      }
      if(count == 0){ return ZERO; }
      Vector2 average_velocity = sum / to_float(count);
      Vector2 steer = average_velocity - velocity;
      return steer * globalConfig.alignment_weight;
   }

   Vector2 cohesion() const noexcept{
      Vector2 sum = {0, 0};
      int count = 0;
      for(auto other : visible_boids){
         sum += other->position;
         count++;
      }
      if(count == 0){ return ZERO; }
      Vector2 average_position = sum / to_float(count);
      Vector2 steer = average_position - position;
      return steer * globalConfig.cohesion_weight;
   }

   Vector2 drag() const noexcept{
      return velocity * -globalConfig.drag;
   }

   void render() const noexcept{
      Vector2 local_x = (Vector2Length(velocity) != 0) ? Vector2Normalize(velocity) : Vector2{1, 0};
      Vector2 local_y = {-local_x.y, local_x.x};
      float L = globalConfig.size;
      float H = globalConfig.size;
      Vector2 tip = position + (local_x * L * 1.4f);
      Vector2 left = position - (local_x * L) + (local_y * H);
      Vector2 right = position - (local_x * L) - (local_y * H);
      DrawTriangle(tip, right, left, globalConfig.color);
   }

   void debug_render() const noexcept{
      const auto debug_color = Fade(globalConfig.color, 0.1f);
      render();
      DrawCircleV(position, globalConfig.vision_range, debug_color);
      for(auto other : visible_boids){
         DrawLineV(position, other->position, debug_color);
      }
      DrawCircleV(position, 1, BLACK);
   }
};




struct Window final{
   Window(int width, int height, std::string_view title, int fps = TARGET_FPS){
      InitWindow(width, height, title.data());
      SetTargetFPS(fps);
   }
   ~Window() noexcept{
      CloseWindow();
   }

   void render(std::span<const Boid> boids, std::span<const Obstacle> obstacles, const Tree& quad_tree) const noexcept{
      BeginDrawing();
      ClearBackground(CLEAR_COLOR);
      bool drawOnce = true;
      for(const auto& boid : boids){
         boid.render();
         if(drawOnce){
            boid.debug_render();
            drawOnce = false;
         }
      }
      for(const auto& obstacle : obstacles){
         obstacle.render();
      }
      quad_tree.render();
      DrawText("Press SPACE to pause/unpause", 10, STAGE_HEIGHT - FONT_SIZE, FONT_SIZE, DARKGRAY);
      DrawFPS(10, STAGE_HEIGHT - FONT_SIZE * 2);
      globalConfig.render();
      EndDrawing();
   }

   bool should_close() const noexcept{
      return WindowShouldClose() || IsKeyPressed(KEY_ESCAPE) || IsKeyPressed(KEY_Q);
   }
};

int main(){
   auto window = Window(STAGE_WIDTH, STAGE_HEIGHT, "Steering #7 - tweaking the quadtree");
   std::vector<Boid> boids(BOID_COUNT);
   std::vector<Obstacle> obstacles(OBSTACLE_COUNT); 
   int capacity = static_cast<int>(std::sqrt(BOID_COUNT)); //Square root of total objects is a good starting point. Profile and adjust as needed!
   //QuadTree<Boid> quad_tree(STAGE_RECT, boids, capacity); //If more than capacity boids are in a quad, it will subdivide     
   //LinearQuadTree<Boid> quad_tree(STAGE_RECT, boids, capacity, 5);
   Tree quad_tree(STAGE_RECT, boids, capacity, 5);
   bool isPaused = false;

   while(!window.should_close()){
      float deltaTime = GetFrameTime();
      if(IsKeyPressed(KEY_SPACE)) isPaused = !isPaused;
            
      quad_tree.rebuild(boids);
      globalConfig.update();     

      for(auto& boid : boids){
         boid.update_visible_boids(quad_tree);
         if(isPaused) continue;
         boid.update(deltaTime, obstacles);
      }

      window.render(boids, obstacles, quad_tree);
   }
   return 0;
}