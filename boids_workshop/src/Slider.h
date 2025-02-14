#pragma once
#include "raylib.h"
#include "raymath.h"
#include <cassert>
#include <string_view>
#include <format>

class Slider{
   static constexpr int FONT_SIZE = 20;
   std::string_view label;
   float* value;
   float min;
   float max;
   Rectangle bounds;

public:
   Slider(std::string_view label, float* value, float min, float max,
      float x, float y, float width = 200, float height = FONT_SIZE) noexcept
      : label(label), value(value), min(min), max(max),
      bounds{x, y + FONT_SIZE, width, height}{
      assert(value != nullptr);
   }

   void update() noexcept{
      if(!IsMouseButtonDown(MOUSE_LEFT_BUTTON)){
         return;
      }
      Vector2 mouse = GetMousePosition();
      if(CheckCollisionPointRec(mouse, {bounds.x - 8, bounds.y,
                                       bounds.width + 16, bounds.height})){
         float newT = (mouse.x - bounds.x) / bounds.width;
         newT = Clamp(newT, 0.0f, 1.0f);
         *value = min + newT * (max - min);
      }

   };

   void draw() noexcept{
      DrawText(label.data(), (int) bounds.x, (int) bounds.y - FONT_SIZE, FONT_SIZE, BLACK);

      // Draw background line
      DrawRectangle((int) bounds.x, (int) bounds.y + (int) bounds.height / 2 - 1,
         (int) bounds.width, 2, GRAY);

      // Calculate knob position
      float t = (*value - min) / (max - min);
      float knobX = bounds.x + t * bounds.width;

      DrawCircle((int) knobX, (int) bounds.y + (int) bounds.height / 2, 8, BLACK);

      auto valueText = std::format("{:.2f}", *value);
      DrawText(valueText.c_str(), (int) bounds.x + (int) bounds.width + 10,
         (int) bounds.y, FONT_SIZE, BLACK);
   }
};