#pragma once

#include <cstdint>

class WS2812 {
    public:
        WS2812(uint8_t pin, uint16_t num_leds);

        bool begin();
        void setColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
        void setColor(uint8_t r, uint8_t g, uint8_t b);
        void setBrightness(uint16_t index, uint8_t percent);
        void setBrightness(uint8_t percent);

        void refresh();
        void reset();

    private:
        uint8_t _pin;
        uint16_t _num_leds;
        uint8_t _brightness;
        void *_strip;        // led_strip_handle_t
};
