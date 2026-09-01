/*
  WDIY ESP32-CAM Plugin Robot
  Blink the RGB LED
  https://github.com/abourdim/wdiy_esp32_cam_robot
*/
#include <Adafruit_NeoPixel.h>

#define RGB_PIN   4  // D3, the TX1812DB pixel on the carrier board
#define RGB_COUNT 1  // DOUT is left unconnected, so there is exactly one

// NEO_GRB is the usual byte order for TX1812. If red and green come out
// swapped on your board, change this to NEO_RGB -- nothing else needs to move.
Adafruit_NeoPixel pixel(RGB_COUNT, RGB_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixel.begin();
  pixel.setBrightness(40);  // full brightness on a 5V pixel is genuinely painful
  pixel.show();             // start dark
}

void loop() {
  pixel.setPixelColor(0, pixel.Color(255, 0, 0));  // red
  pixel.show();
  delay(1000);
  pixel.clear();                                   // off
  pixel.show();
  delay(1000);
}
