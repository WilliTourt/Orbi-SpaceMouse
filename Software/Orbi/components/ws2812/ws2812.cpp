#include "ws2812.h"
#include "led_strip.h"

WS2812::WS2812(uint8_t pin, uint16_t num_leds)
    : _pin(pin), _num_leds(num_leds), _brightness(50), _strip(nullptr) {}

bool WS2812::begin() { return true; }
void WS2812::setColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {}
void WS2812::setColor(uint8_t r, uint8_t g, uint8_t b) {}
void WS2812::setBrightness(uint16_t index, uint8_t percent) {}
void WS2812::setBrightness(uint8_t percent) {}

void WS2812::refresh() {}
void WS2812::reset() {}
