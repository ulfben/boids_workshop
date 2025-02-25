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
#pragma once
#include "raylib.h"
#include "raymath.h"
#include <string_view>
#include <cassert>
#include <cmath>
#include <concepts>
#include <format>
#include <string>

template<typename T>
concept Numeric = std::integral<T> || std::floating_point<T>;

// float versions of raylib drawing functions
static void DrawRectangleF(float x, float y, float width, float height, Color color){
   DrawRectangle(static_cast<int>(x), static_cast<int>(y),
      static_cast<int>(width), static_cast<int>(height), color);
}

static void DrawTextF(std::string_view text, float x, float y, int fontSize, Color color){
   DrawText(text.data(), static_cast<int>(x), static_cast<int>(y), fontSize, color);
}

static void DrawCircleF(float x, float y, float radius, Color color){
   DrawCircle(static_cast<int>(x), static_cast<int>(y), radius, color);
}

template<Numeric T>
class Slider{
   static constexpr int FONT_SIZE = 20;
   std::string_view label;
   T* value;
   T min;
   T max;
   Rectangle bounds;

public:
   Slider(std::string_view label, T* value, T min, T max,
      float x = 10.0f, float y = 10.0f, float width = 200, float height = FONT_SIZE) noexcept
      : label(label), value(value), min(min), max(max),
      bounds{x, y + FONT_SIZE, width, height}{
      assert(value != nullptr);
      assert(min < max);
   }

   void update() noexcept{
      if(!IsMouseButtonDown(MOUSE_LEFT_BUTTON)){
         return;
      }
      Vector2 mouse = GetMousePosition();
      if(!CheckCollisionPointRec(mouse, {bounds.x - 8, bounds.y,
                                       bounds.width + 16, bounds.height})){
         return;
      }
      float newT = (mouse.x - bounds.x) / bounds.width;
      newT = Clamp(newT, 0.0f, 1.0f);
      if constexpr(std::integral<T>){         
         *value = static_cast<T>(std::roundf(min + static_cast<float>(max - min) * newT));
      } else{
         *value = min + (max - min) * newT;
      }      
   }

   void render() const noexcept{
      DrawTextF(label, bounds.x, bounds.y - FONT_SIZE, FONT_SIZE, BLACK);

      //draw the slider center line
      DrawRectangleF(bounds.x, bounds.y + bounds.height / 2.0f - 1.0f, bounds.width, 2.0f, GRAY);

      float knobX = getKnobX();
      DrawCircleF(knobX, bounds.y + bounds.height / 2.0f, 8.0f, BLACK);
      
      DrawTextF(formatValue(), bounds.x + bounds.width + 10.0f, bounds.y, FONT_SIZE, BLACK);
   }

   void top(float y) noexcept{
      bounds.y = y;
   }

   float bottom() const noexcept{
      return bounds.y + bounds.height + FONT_SIZE;
   }
private:
   float getKnobX() const noexcept{
      float t = static_cast<float>(*value - min) / static_cast<float>(max - min);
      return bounds.x + t * bounds.width;
   }

   std::string formatValue() const{
      if constexpr(std::integral<T>){
         return std::format("{}", *value);
      } else{
         return std::format("{:.2f}", *value);
      }
   }
};