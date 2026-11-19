#include "key.h"

#include "esp_log.h"

static const char *TAG = "key";

// 3 consecutive equal samples before we believe a change (30 ms @ 10 ms tick)
static constexpr uint8_t DEBOUNCE_SAMPLES = 3;

void Key::attach(gpio_num_t pin, bool active_low) {
    _pin = pin;
    _active_low = active_low;
    _enabled = (pin != GPIO_NUM_NC);
}

bool Key::begin() {
    if (!_enabled) return false;

    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << _pin;
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = _active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    io.pull_down_en = _active_low ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    if (gpio_config(&io) != ESP_OK) {
        ESP_LOGE(TAG, "gpio %d config failed", (int)_pin);
        _enabled = false;
        return false;
    }

    _down = (gpio_get_level(_pin) == 0) == _active_low;
    return true;
}

Key::Event Key::tick() {
    if (!_enabled) return Event::None;

    const bool raw_pressed = (gpio_get_level(_pin) == 0) == _active_low;

    if (raw_pressed != _down) {
        if (++_stability < DEBOUNCE_SAMPLES) return Event::None;
        _stability = 0;
        _down = raw_pressed;

        if (_down) {
            _held_ticks = 0;
            _long_fired = false;
            return Event::Press;
        } else {
            if (!_long_fired) return Event::Release;
            return Event::Release;
        }
    }

    _stability = 0;

    if (_down) {
        _held_ticks++;
        if (!_long_fired && (_held_ticks * KEY_TICK_MS) >= LONG_PRESS_MS) {
            _long_fired = true;
            return Event::LongPress;
        }
    }
    return Event::None;
}

uint32_t Key::heldMs() const {
    return _held_ticks * KEY_TICK_MS;
}
