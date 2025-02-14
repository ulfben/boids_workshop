#include "raylib.h"
#include "raymath.h"
#include "Slider.h"
#include <vector>
#include <span>
#include <string_view>

constexpr bool INTERACT_MOUSE = false;
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
   static constexpr float MAX_VELOCITY = 200.0f; //pixels per second
   static constexpr float PERCEPTION_RADIUS = 200;
   static constexpr float BOID_SIZE = 5.0f;
   static constexpr float SEPARATION_DISTANCE = BOID_SIZE * 3;
   static constexpr float MOUSE_AVOID_DISTANCE = 120.0f;
   static constexpr float COHESION_WEIGHT = MAX_VELOCITY * 0.2f;  //I scale by MAX_VELOCITY to ensure that each beahavior's 
   static constexpr float ALIGNMENT_WEIGHT = MAX_VELOCITY * 0.2f;  //acceleration force is able to change velocity by a noticeable
   static constexpr float SEPARATION_WEIGHT = MAX_VELOCITY * 0.8f; //fraction each frame.
   static constexpr float MOUSE_AVOID_WEIGHT = MAX_VELOCITY * 0.14f;
   static constexpr float MOUSE_SEEK_WEIGHT = MAX_VELOCITY * 0.2f;

   Vector2 position = random_range({0.0f, 0.0f}, {STAGE_WIDTH, STAGE_HEIGHT});
   Vector2 velocity = {0.0f, 0.0f};

   void update(std::span<Boid> boids, float dt) noexcept{
      Vector2 acceleration = {0, 0};
      acceleration += get_cohesion(boids, PERCEPTION_RADIUS);
      acceleration += get_alignment(boids, PERCEPTION_RADIUS);
      acceleration += get_separation(boids, SEPARATION_DISTANCE);

      if constexpr(INTERACT_MOUSE){
         acceleration += seek_point(GetMousePosition(), MOUSE_SEEK_WEIGHT);         
      }     
      velocity += acceleration;
      velocity = Vector2ClampValue(velocity, 0, MAX_VELOCITY);
      position += velocity * dt;
      world_wrap();
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

   void world_wrap() noexcept{
      if(position.x > STAGE_WIDTH) position.x -= STAGE_WIDTH;
      if(position.x < 0) position.x += STAGE_WIDTH;
      if(position.y > STAGE_HEIGHT) position.y -= STAGE_HEIGHT;
      if(position.y < 0) position.y += STAGE_HEIGHT;
   }

   float distance_to(const Boid& other) const{
      return Vector2Length(position - other.position);
   }

   Vector2 get_separation(std::span<const Boid> boids, float minDistance) const{
      Vector2 totalSeparation{0, 0};
      for(const auto& other : boids){
         if(&other == this) continue;
         float distance = distance_to(other);
         if(distance >= minDistance || distance == 0) continue;

         Vector2 diff = position - other.position;
         float repulsionFactor = (minDistance - distance) / distance;
         totalSeparation += Vector2Normalize(diff) * repulsionFactor;
         //totalSeparation += Vector2Normalize(diff) * repulsionFactor * SEPARATION_WEIGHT;         
      }
      return Vector2Normalize(totalSeparation) * SEPARATION_WEIGHT;
   }

   Vector2 get_alignment(std::span<const Boid> boids, float maxDistance) const{
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
         return Vector2Normalize(avgHeading) * ALIGNMENT_WEIGHT;
      }
      return avgHeading;
   }

   Vector2 get_cohesion(std::span<const Boid> boids, float maxDistance) const noexcept{
      Vector2 centerOfMass{0, 0};
      int count = 0;

      for(const auto& other : boids){
         if(&other == this || distance_to(other) > maxDistance) continue;
         centerOfMass += other.position;
         count++;
      }

      if(count > 0){
         centerOfMass *= (1.0f / to_float(count));
         return seek_point(centerOfMass, COHESION_WEIGHT);
      }
      return {0, 0};
   }

   Vector2 seek_point(Vector2 target, float weight) const noexcept{
      Vector2 desired = target - position;
      float distance = Vector2Length(desired);
      if(distance > 0){
         return Vector2Normalize(desired) * weight;
      }
      return {0, 0};

   }

   Vector2 avoid_point(Vector2 point, float minDistance, float weight) const noexcept{
      float distance = Vector2Length(position - point);
      if(distance > minDistance){
         return {0,0};
      }
      Vector2 diff = position - point;
      return Vector2Normalize(diff) * (weight * (minDistance - distance) / distance);
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
   float test = 0.0f;
   Slider cohesionSlider("Cohesion", &test, 0, Boid::MAX_VELOCITY, 10, 30);

   while(!window.shouldClose()){
      for(auto& boid : boids){
         boid.update(boids, GetFrameTime());
      }
      window.draw(boids);
      cohesionSlider.update();
      cohesionSlider.draw();
   }
   return 0;
}