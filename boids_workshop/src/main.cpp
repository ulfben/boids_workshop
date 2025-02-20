#include "raylib.h"
#include "raymath.h"
#include "Slider.h"
#include <vector>
#include <span>
#include <string_view>
#include <cmath>

constexpr bool INTERACT_MOUSE = false;
constexpr auto CLEAR_COLOR = WHITE;
constexpr int STAGE_WIDTH = 1280;
constexpr int STAGE_HEIGHT = 720;
constexpr Vector2 STAGE_SIZE = {static_cast<float>(STAGE_WIDTH), static_cast<float>(STAGE_HEIGHT)};
constexpr Vector2 ZERO = {0.0f, 0.0f};
constexpr float TO_RAD = DEG2RAD;
constexpr float TO_DEG = RAD2DEG;
constexpr int BOID_COUNT = 80;
constexpr int TARGET_FPS = 60;
constexpr int FONT_SIZE = 20;

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

struct BoidConfig{
   Color color = RED;
   float size = 8.0f;
   float visionRange = 80.0f;       // how far a boid “sees” others
   float cohesionFactor = 0.2f;     // strength of moving toward group center
   float alignmentFactor = 0.3f;    // strength of matching velocity
   float separationFactor = 8.0f;  // strength of keeping distance
   float separationRange = 180.0f;   // how close is “too close”
   float drag = 0.01f;              // simple drag applied to the velocity
   float minSpeed = 50.0f;
   float maxSpeed = 150.0f;
};

BoidConfig globalConfig{}; // default configuration

struct Boid{
   Vector2 position = random_range(ZERO, STAGE_SIZE);
   Vector2 velocity = vector_from_angle(random_range(0.0f, 360.0f) * TO_RAD, globalConfig.minSpeed);
   std::vector<const Boid*> visibleBoids; // non-owning pointers to nearby boids

   void update_visible_boids(std::span<const Boid> boids){
      visibleBoids.clear();
      for(auto& other : boids){
         if(&other == this) continue; //don't add ourselves to the list
         if(Vector2Distance(position, other.position) < globalConfig.visionRange){
            visibleBoids.push_back(&other);
         }
      }
   }

   void update(float deltaTime) noexcept{
      Vector2 acceleration = {0, 0};
      acceleration += separation();
      acceleration += drag();

      velocity += acceleration * deltaTime;
      velocity = Vector2ClampValue(velocity, globalConfig.minSpeed, globalConfig.maxSpeed);

      position += velocity * deltaTime;
      position = world_wrap(position, STAGE_SIZE);
   }

   Vector2 separation() const noexcept{
      Vector2 steer = {0, 0};
      int count = 0;
      for(auto other : visibleBoids){
         Vector2 offset = position - other->position;
         float distance = Vector2Length(offset);
         if(distance < globalConfig.separationRange){
            Vector2 diff = Vector2Normalize(offset); // Compute a vector pointing away from the neighbor.
            steer += diff * (globalConfig.separationRange - distance); // The magnitude is (separationRange - distance) so that the force grows as the boids get closer.                     
            count++;
         }
      }
      // Average the contributions from all neighbors, and scale by separactionFactor
      return count > 0 ? (steer / to_float(count)) * globalConfig.separationFactor      
                       : steer;
   }

   Vector2 drag() const noexcept{
      return velocity * -globalConfig.drag;
   }

   void render() const noexcept{
      Vector2 dir = (Vector2Length(velocity) != 0) ? Vector2Normalize(velocity) : Vector2{1, 0};
      float degAngle = std::atan2(dir.y, dir.x) * TO_DEG;
      DrawPoly(position, 3, globalConfig.size, degAngle, globalConfig.color);
      //DrawCircle(static_cast<int>(position.x), static_cast<int>(position.y), 1, BLACK);
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

   void render(std::span<const Boid> boids) const noexcept{
      BeginDrawing();
      ClearBackground(CLEAR_COLOR);
      for(const auto& boid : boids){
         boid.render();
      }
      DrawFPS(10, 10);
      DrawText("Press SPACE to pause/unpause", 10, FONT_SIZE + 10, FONT_SIZE + 10, DARKGRAY);
      EndDrawing();
   }

   bool should_close() const noexcept{
      return WindowShouldClose() || IsKeyPressed(KEY_ESCAPE) || IsKeyPressed(KEY_Q);
   }
};

int main(){
   auto window = Window(STAGE_WIDTH, STAGE_HEIGHT, "Boids Simulation");
   std::vector<Boid> boids(BOID_COUNT);
   bool isPaused = false;
   while(!window.should_close()){
      float deltaTime = GetFrameTime();
      if(IsKeyPressed(KEY_SPACE)) isPaused = !isPaused;

      for(auto& boid : boids){
         boid.update_visible_boids(boids);
         if(isPaused) continue;
         boid.update(deltaTime);
      }

      window.render(boids);
   }
   return 0;
}