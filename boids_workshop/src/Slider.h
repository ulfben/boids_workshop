#pragma once
#include "raylib.h"
#include "raymath.h"
#include <cassert>
#include <string>
#include <string_view>
#include <format>
#include <concepts>

template<typename T>
concept Numeric = std::integral<T> || std::floating_point<T>;

template<Numeric T>
class Slider {
   static constexpr int FONT_SIZE = 20;   
   std::string_view label;
   T* value;
   T min;
   T max;
   Rectangle bounds;

public:
   Slider(std::string_view label, T* value, T min, T max,
      float x, float y, float width = 200, float height = FONT_SIZE) noexcept
      : label(label), value(value), min(min), max(max),
      bounds{x, y + FONT_SIZE, width, height} {
      assert(value != nullptr);
   }

   void update() noexcept {
      if(!IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
         return;
      }
      Vector2 mouse = GetMousePosition();
      if(CheckCollisionPointRec(mouse, {bounds.x - 8, bounds.y,
                                       bounds.width + 16, bounds.height})) {
         float newT = (mouse.x - bounds.x) / bounds.width;
         newT = Clamp(newT, 0.0f, 1.0f);
         if constexpr(std::integral<T>) {
            *value = static_cast<T>(min + static_cast<float>(max - min) * newT + 0.5f);
         } else {
            *value = min + (max - min) * newT;
         }
      }
   }

   void draw() noexcept {
      DrawText(label.data(), static_cast<int>(bounds.x), 
               static_cast<int>(bounds.y - FONT_SIZE), FONT_SIZE, BLACK);
      
      DrawRectangle(static_cast<int>(bounds.x), 
                    static_cast<int>(bounds.y + bounds.height / 2 - 1),
                    static_cast<int>(bounds.width), 2, GRAY);

      float t = static_cast<float>(*value - min) / static_cast<float>(max - min);
      float knobX = bounds.x + t * bounds.width;
      
      DrawCircle(static_cast<int>(knobX), 
                static_cast<int>(bounds.y + bounds.height / 2), 8, BLACK);

      std::string valueText;
      if constexpr(std::integral<T>) {
         valueText = std::format("{}", *value);
      } else {
         valueText = std::format("{:.2f}", *value);
      }

      DrawText(valueText.c_str(), 
               static_cast<int>(bounds.x + bounds.width + 10),
               static_cast<int>(bounds.y), FONT_SIZE, BLACK);
   }

   float bottom() const noexcept{
      return bounds.y + bounds.height;
   }
};