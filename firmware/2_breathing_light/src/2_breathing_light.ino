/*
  WDIY ESP32-CAM Plugin Robot
  Control the brightness of the RGB LED
  https://github.com/abourdim/wdiy_esp32_cam_robot
*/
#include <Adafruit_NeoPixel.h>

#define RGB_PIN   4
#define RGB_COUNT 1

Adafruit_NeoPixel pixel(RGB_COUNT, RGB_PIN, NEO_GRB + NEO_KHZ800);

// Paint the single pixel white at brightness `v` (0..255). setBrightness()
// would do the same job, but it is applied at show() time against the stored
// colour and repeated rescaling of an already-scaled value loses steps -- so
// the level goes straight into the colour instead.
static void level(uint8_t v) {
  pixel.setPixelColor(0, pixel.Color(v, v, v));
  pixel.show();
}

void setup() {
  pixel.begin();
  pixel.show();
}

void loop() {
  for (int i = 0; i < 255; i++) {  // fade up
    level(i);
    delay(10);
  }
  for (int i = 255; i > 1; i--) {  // fade down
    level(i);
    delay(10);
  }
}
