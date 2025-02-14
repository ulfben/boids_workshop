#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <span>
#include <string_view>

constexpr bool INTERACT_MOUSE = true;
constexpr auto CLEAR_COLOR = WHITE;
constexpr int STAGE_WIDTH = 1080;
constexpr int STAGE_HEIGHT = 720;
constexpr int BOID_COUNT = 80;
constexpr int TARGET_FPS = 60;

constexpr static float to_float(int value) noexcept{
   return static_cast<float>(value);
}

constexpr static float range01() noexcept{
   constexpr auto RAND_MAXF = to_float(RAND_MAX);
   return to_float(GetRandomValue(0, RAND_MAX)) / RAND_MAXF;
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

class Boid{
public:
   static constexpr float MAX_VELOCITY = 5.0f;
   static constexpr float PERCEPTION_RADIUS = 200;
   static constexpr float COHESION_WEIGHT = 0.16f;
   static constexpr float ALIGNMENT_WEIGHT = 0.11f;
   static constexpr float BOID_SIZE = 5.0f;
   static constexpr float SEPARATION_DISTANCE = BOID_SIZE * 3;
   static constexpr float SEPARATION_WEIGHT = 0.18f;
   static constexpr float MOUSE_AVOID_DISTANCE = 120.0f;
   static constexpr float MOUSE_AVOID_WEIGHT = 0.14f;
   static constexpr float MOUSE_SEEK_WEIGHT = 0.3f;

   Vector2 position = random_range({0.0f, 0.0f}, {STAGE_WIDTH, STAGE_HEIGHT});
   Vector2 velocity = random_range({-MAX_VELOCITY, -MAX_VELOCITY}, {MAX_VELOCITY, MAX_VELOCITY});   

   void update(std::span<Boid> boids){
      apply_separation(boids, SEPARATION_DISTANCE);
      apply_alignment(boids, PERCEPTION_RADIUS);
      apply_cohesion(boids, PERCEPTION_RADIUS);

      if constexpr(INTERACT_MOUSE){
        seek_point(GetMousePosition(), MOUSE_SEEK_WEIGHT);
        /*if(is_inside(STAGE_WIDTH, STAGE_HEIGHT)){
           avoid_point(GetMousePosition(), MOUSE_AVOID_DISTANCE, MOUSE_AVOID_WEIGHT);
        } else{
           seek_point(GetMousePosition(), MOUSE_SEEK_WEIGHT);
        }*/
      }
      move();
   }

   void render() const{
      DrawCircleV(position, BOID_SIZE, RED);
      //DrawCircleLinesV(position, PERCEPTION_RADIUS, GRAY);
   }

private:
   bool is_inside(int width, int height) const noexcept{
      return !(position.x < BOID_SIZE || position.x > width - BOID_SIZE ||
         position.y < BOID_SIZE || position.y > height - BOID_SIZE);
   }

   void move(){
      velocity.x = Clamp(velocity.x, -MAX_VELOCITY, MAX_VELOCITY);
      velocity.y = Clamp(velocity.y, -MAX_VELOCITY, MAX_VELOCITY);
      position += velocity;
      if constexpr(!INTERACT_MOUSE){
         if(position.x > STAGE_WIDTH) position.x -= STAGE_WIDTH;
         if(position.x < 0) position.x += STAGE_WIDTH;
         if(position.y > STAGE_HEIGHT) position.y -= STAGE_HEIGHT;
         if(position.y < 0) position.y += STAGE_HEIGHT;
      }
   }

   float distance_to(const Boid& other) const{
      return Vector2Length(position - other.position);
   }

   void apply_separation(std::span<const Boid> boids, float minDistance){
      for(const auto& other : boids){
         if(&other == this) continue;
         float distance = distance_to(other);
         if(distance >= minDistance || distance == 0) continue;

         Vector2 diff = position - other.position;
         float repulsionFactor = (minDistance - distance) / distance;
         Vector2 repulsion = Vector2Normalize(diff) * (SEPARATION_WEIGHT * repulsionFactor);
         velocity += repulsion;
      }
   }

   void apply_alignment(std::span<const Boid> boids, float maxDistance){    
      Vector2 avgHeading{0, 0};
      int count = 0;
      for(const auto& other : boids){
         if(&other == this || distance_to(other) > maxDistance) continue;

         float magnitude = Vector2Length(other.velocity);
         if(magnitude <= 0){ continue; }

         Vector2 heading = other.velocity * (1.0f / magnitude);
         avgHeading += heading;
         count++;
      }

      if(count > 0){
         avgHeading *= (1.0f / to_float(count));
         velocity += Vector2Normalize(avgHeading) * ALIGNMENT_WEIGHT;
      }
   }

   void apply_cohesion(std::span<const Boid> boids, float maxDistance){
      Vector2 centerOfMass{0, 0};
      int count = 0;

      for(const auto& other : boids){
         if(&other == this || distance_to(other) > maxDistance) continue;
         centerOfMass += other.position;
         count++;
      }

      if(count > 0){
         centerOfMass *= (1.0f / to_float(count));
         seek_point(centerOfMass, COHESION_WEIGHT);
      }
   }

   void seek_point(Vector2 target, float weight){
      Vector2 desired = target - position;
      float distance = Vector2Length(desired);
      if(distance > 0){         
         velocity += Vector2Normalize(desired) * weight;
      }
   }

   void avoid_point(Vector2 point, float minDistance, float weight) noexcept{
      float distance = Vector2Length(position - point);
      if(distance > minDistance){
         return;
      }
      Vector2 diff = position - point;
      Vector2 repulsion = Vector2Normalize(diff) * (weight * (minDistance - distance) / distance);
      velocity += repulsion;
   }
};


struct Window final{
   Window(int width, int height, std::string_view title, int fps = TARGET_FPS){
      InitWindow(width, height, title.data());
      SetTargetFPS(fps);
   }
   ~Window(){
      CloseWindow();
   }
   void draw(std::span<Boid> boids) const noexcept{
      BeginDrawing();
      ClearBackground(CLEAR_COLOR);
      for(const auto& boid : boids){
         boid.render();
      }
      if constexpr(INTERACT_MOUSE){
         DrawCircleLinesV(GetMousePosition(), Boid::MOUSE_AVOID_DISTANCE, GRAY);
      }
      DrawFPS(10, 10);
      EndDrawing();
   }
   bool shouldClose() const noexcept{
      return WindowShouldClose() || IsKeyPressed(KEY_ESCAPE) || IsKeyPressed(KEY_Q);
   }
};

int main(){
   auto window = Window(STAGE_WIDTH, STAGE_HEIGHT, "Boids Simulation");
   std::vector<Boid> boids(BOID_COUNT);
   while(!window.shouldClose()){
      for(auto& boid : boids){
         boid.update(boids);
      }
      window.draw(boids);
   }
   return 0;
}