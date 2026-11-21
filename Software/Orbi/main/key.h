#pragma once

#include "driver/gpio.h"
#include "driver/gpio_filter.h"

class Key {
    public:

        Key(gpio_num_t pin) : _pin(pin) {}
        ~Key();

        bool begin();
        void update();

        bool isSingleClicked() const { return _single_click; }
        bool isDoubleClicked() const { return _double_click; }
        bool isLongPressed()   const { return _long_press; }

    private:
        static constexpr uint32_t DEBOUNCE_MS = 15;

        // maximum intenal between two presses(clicks). if the gap between presses(clicks)
        // is less than this, it is considered a double click
        static constexpr uint32_t DOUBLE_PRESS_INTERVAL_MS = 350;
        // minimum time to hold for a long press
        static constexpr uint32_t LONG_PRESS_MS            = 700;

        enum class State : uint8_t {
            Idle,
            Holding,
            ReleaseWaiting,
            DoubleHolding,
            LongHolding
        };

        gpio_num_t _pin = GPIO_NUM_NC;
        gpio_glitch_filter_handle_t _filter = nullptr;
        bool _ok  = false;

        State    _state        = State::Idle;

        bool     _last_lvl     = true;
        bool     _stable_lvl   = true;
        bool     _long_lock    = false;

        uint32_t _t_iochange   = 0; // for debouncing
        uint32_t _t_pressed    = 0; // for long press
        uint32_t _t_released   = 0; // for double click

        bool     _single_click = false;
        bool     _double_click = false;
        bool     _long_press   = false;
};
