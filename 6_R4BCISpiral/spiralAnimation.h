#ifndef SPIRAL_ANIMATION_H
#define SPIRAL_ANATION_H

#include "Arduino_LED_Matrix.h"

// Matrix dimensions
static constexpr uint8_t SP_ROWS = 8;
static constexpr uint8_t SP_COLS = 12;
static constexpr uint16_t SP_NUM  = SP_ROWS * SP_COLS;

// Simple point struct
struct SPA_Point { uint8_t x, y; };

class SpiralAnimation {
public:
  // Call once in setup()
  static void init() {
    matrix.begin();
    matrix.clear();
    currentIndex = 0;
    buildSpiral();
  }

  // Step one LED ON in spiral order
  static void stepForward() {
    if (currentIndex < SP_NUM) {
      auto &p = spiral[currentIndex];
      frame[p.y][p.x] = 1;
      matrix.renderBitmap(frame, SP_ROWS, SP_COLS);
      currentIndex++;
    }
  }

  // Step one LED OFF in reverse spiral
  static void stepBackward() {
    if (currentIndex > 0) {
      currentIndex--;
      auto &p = spiral[currentIndex];
      frame[p.y][p.x] = 0;
      matrix.renderBitmap(frame, SP_ROWS, SP_COLS);
    }
  }

private:
  // Underlying matrix driver
  static ArduinoLEDMatrix matrix;

  // 2D frame buffer: 0=off, 1=on
  static uint8_t frame[SP_ROWS][SP_COLS];

  // Precomputed spiral sequence
  static SPA_Point spiral[SP_NUM];

  // Current position in spiral[]
  static int currentIndex;

  // Build a clockwise spiral of (x,y)
  static void buildSpiral() {
    int top    = 0, bottom = SP_ROWS - 1;
    int left   = 0, right  = SP_COLS - 1;
    int idx    = 0;

    while (top <= bottom && left <= right) {
      for (int x = left;  x <= right;  x++) spiral[idx++] = { (uint8_t)x, (uint8_t)top };
      top++;
      for (int y = top;   y <= bottom; y++) spiral[idx++] = { (uint8_t)right, (uint8_t)y };
      right--;
      if (top <= bottom) {
        for (int x = right; x >= left;  x--) spiral[idx++] = { (uint8_t)x, (uint8_t)bottom };
        bottom--;
      }
      if (left <= right) {
        for (int y = bottom; y >= top;   y--) spiral[idx++] = { (uint8_t)left,  (uint8_t)y };
        left++;
      }
    }
  }
};

// Static member definitions
ArduinoLEDMatrix     SpiralAnimation::matrix;
uint8_t              SpiralAnimation::frame[SP_ROWS][SP_COLS] = {{0}};
SPA_Point            SpiralAnimation::spiral[SP_NUM];
int                   SpiralAnimation::currentIndex = 0;

#endif // SPIRAL_ANIMATION_H
