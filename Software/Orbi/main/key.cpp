#include "key.h"

#include "esp_timer.h"

static inline uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }

Key::~Key() {
    if (_filter) {
        gpio_glitch_filter_disable(_filter);
        gpio_del_glitch_filter(_filter);
    }
}

bool Key::begin() {
    if (_pin == GPIO_NUM_NC) return false;
    _ok = true;

    gpio_config_t io = {};
    io.intr_type = GPIO_INTR_DISABLE;
    io.mode = GPIO_MODE_INPUT;
    io.pin_bit_mask = 1ULL << _pin;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    _ok &= (gpio_config(&io) == ESP_OK);

    gpio_pin_glitch_filter_config_t filter_cfg = {
        .clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT,
        .gpio_num = _pin
    };
    _ok &= (gpio_new_pin_glitch_filter(&filter_cfg, &_filter) == ESP_OK);
    if (_filter) _ok &= (gpio_glitch_filter_enable(_filter) == ESP_OK);

    _last_lvl = gpio_get_level(_pin);
    _stable_lvl = _last_lvl;

    return _ok;
}

void Key::update() {
    if (!_ok) return;

    _single_click = false;
    _double_click = false;
    _long_press   = false;

    const uint32_t now = now_ms();
    const bool lvl = gpio_get_level(_pin);

    if (lvl != _last_lvl) {
        _t_iochange = now;
        _last_lvl = lvl;
    }

    bool pressed_edge  = false;
    bool released_edge = false;
    if (((now - _t_iochange) >= DEBOUNCE_MS) && (lvl != _stable_lvl)) {
        _stable_lvl = lvl;
        pressed_edge  = !lvl;
        released_edge = lvl;
    }

    switch (_state) {
        case State::Idle:
            if (pressed_edge) {
                _long_lock = false;
                _t_pressed = now;
                _state = State::Holding;
            }
            break;

        case State::Holding:
            if (released_edge) {
                _t_released = now;
                _state = State::ReleaseWaiting;
            } else if ((now - _t_pressed) >= LONG_PRESS_MS) {
                _state = State::LongHolding;
                if (!_long_lock) {
                    _long_lock = true;
                    _long_press = true;
                }
            }
            break;

        case State::ReleaseWaiting:
            if (pressed_edge) {
                _state = State::DoubleHolding;
            } else if (now - _t_released > DOUBLE_PRESS_INTERVAL_MS) {
                _single_click = true;
                _state = State::Idle;
            }
            break;
        
        case State::DoubleHolding:
            if (released_edge) {
                _double_click = true;
                _state = State::Idle;
            }
            break;

        case State::LongHolding:
            if (released_edge) _state = State::Idle;
            break;
    }
}
