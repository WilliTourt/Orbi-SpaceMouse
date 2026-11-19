#pragma once

#include <cstdint>

#include "driver/gpio.h"

// One physical key. Debounce is counter based and needs a steady tick, so
// Key::tick() must be called from one task only.
class Key {
    public:
        enum class Event : uint8_t {
            None,
            Press,
            LongPress,
            Release,
        };

        // active_low: pin reads 0 while pressed (keys wired to GND)
        void attach(gpio_num_t pin, bool active_low = true);
        bool begin();

        Event tick();                 // call every KEY_TICK_MS
        bool isDown() const { return _down; }
        uint32_t heldMs() const;

        static constexpr uint32_t KEY_TICK_MS     = 10;
        static constexpr uint32_t LONG_PRESS_MS   = 700;

    private:
        gpio_num_t _pin = GPIO_NUM_NC;
        bool _active_low = true;
        bool _down = false;
        uint8_t _stability = 0;
        uint32_t _held_ticks = 0;
        bool _long_fired = false;
        bool _enabled = false;
};
