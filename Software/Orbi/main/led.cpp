#include "led.h"
#include "config.h"

#include <cmath>

#include "driver/gpio.h"
#include "esp_timer.h"

static inline uint32_t nowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }

bool Led::begin(uint8_t pin, uint16_t count) {
    _led_count = count;
    _strip = new WS2812(pin, count);
    if (_strip == nullptr) return false;

    if (!_strip->begin()) {
        delete _strip;
        _strip = nullptr;
        return false;
    }
    _strip->setBrightness(_brightness);

    // on board 0603 led
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << CFG_PIN_STATUS_LED;
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io);
    _setStatusLed(false);

    return true;
}

void Led::setBrightness(uint8_t percent) {
    _brightness = percent > 100 ? 100 : percent;
    if (_strip) _strip->setBrightness(_brightness);
}

void Led::setSysEffects(SysEffect effect) {
    Effects &e = _effects;
    if (e.sys == effect) return;
    e.sys = effect;
    e.sys_started = nowMs();

    if (effect == SysEffect::ReadyUsrEffect) {
        e.usr_started = e.sys_started;   // animation starts at its first frame
        e.key_len_ms = 0;
    }
}

void Led::setUsrEffects(const UsrEffect &usr) {
    Effects &e = _effects;
    if (usr.joystick != e.usr.joystick) e.usr_started = nowMs();
    e.usr = usr;
}

void Led::setPlainColor(uint8_t r, uint8_t g, uint8_t b) {
    _effects.color_plain[0] = r;
    _effects.color_plain[1] = g;
    _effects.color_plain[2] = b;
}

void Led::reactTilt(float x, float y) {
    _effects.tilt_x = (x > 1.0f) ? 1.0f : ((x < -1.0f) ? -1.0f : x);
    _effects.tilt_y = (y > 1.0f) ? 1.0f : ((y < -1.0f) ? -1.0f : y);
}

void Led::reactKey(KeyEvent event) {
    Effects &e = _effects;
    switch (event) {
        case KeyEvent::ShortL:
        case KeyEvent::ShortR:
            e.key_len_ms = KEY_SHORT_MS;
            break;
        case KeyEvent::LongL:
        case KeyEvent::LongR:
            e.key_len_ms = KEY_LONG_MS;
            break;
        default: return;
    }

    e.key_event = event;
    e.key_started_ms = nowMs();
}

void Led::update() {
    if (!_strip) return;

    Effects &e = _effects;
    const uint32_t now = nowMs();

    if (e.sys != SysEffect::ReadyUsrEffect) {
        _applySys(now);
    } else {
        if (e.key_len_ms != 0 && (now - e.key_started_ms) >= e.key_len_ms) e.key_len_ms = 0;
        if (e.key_len_ms != 0) _applyKey(now);
        else                _applyJoystick(now);
    }
    _strip->refresh();
}

void Led::_fill(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < _led_count) _strip->setColor(index, r, g, b);
}

void Led::_fillAll(uint8_t r, uint8_t g, uint8_t b) {
    for (uint16_t i = 0; i < _led_count; i++) _fill(i, r, g, b);
}

void Led::_setStatusLed(bool on) {
    gpio_set_level((gpio_num_t)CFG_PIN_STATUS_LED, on ? 1 : 0);
}

void Led::_hsv(uint8_t h, uint8_t s, uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b) {
    if (s == 0) { r = g = b = v; return; }

    uint8_t region = h / 43;
    uint8_t rem    = (uint8_t)((h - region * 43) * 6);
    uint8_t p = (uint8_t)((v * (255 - s)) >> 8);
    uint8_t q = (uint8_t)((v * (255 - ((s * rem) >> 8))) >> 8);
    uint8_t t = (uint8_t)((v * (255 - ((s * (255 - rem)) >> 8))) >> 8);

    switch (region) {
        case 0:  r = v; g = t; b = p; break;
        case 1:  r = q; g = v; b = p; break;
        case 2:  r = p; g = v; b = t; break;
        case 3:  r = p; g = q; b = v; break;
        case 4:  r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }
}



void Led::_applySys(uint32_t now) {
    const Effects &e = _effects;
    const uint32_t t = now - e.sys_started;
    uint8_t r = 0, g = 0, b = 0;

    switch (e.sys) {
        case SysEffect::Off:
            _fillAll(0, 0, 0);
            return;

        case SysEffect::Boot: {
            // TODO
            return;
        }

        case SysEffect::Idle: {
            // TODO
            return;
        }

        case SysEffect::ReadyUsrEffect:
            // TODO
            return;

        case SysEffect::Calibrating: {
            // TODO
            return;
        }

        case SysEffect::LowBattery: {
            // TODO
            return;
        }

        case SysEffect::Charging: {
            // TODO
            return;
        }

        case SysEffect::Error: {
            // TODO
            return;
        }
    }
}

void Led::_applyJoystick(uint32_t now) {
    const Effects &e = _effects;
    const uint32_t t = now - e.usr_started;
    uint8_t r = 0, g = 0, b = 0;

    float mag = sqrtf(e.tilt_x * e.tilt_x + e.tilt_y * e.tilt_y);
    if (mag > 1.0f) mag = 1.0f;

    switch (e.usr.joystick) {
        case UsrEffect::Joystick::Default:
            _fillAll(e.color_plain[0], e.color_plain[1], e.color_plain[2]);
            return;

        case UsrEffect::Joystick::BreathePlain: {
            // TODO
            return;
        }

        case UsrEffect::Joystick::BreatheWave: {
            // TODO
            return;
        }

        case UsrEffect::Joystick::RainbowPlain:
            // TODO
            return;

        case UsrEffect::Joystick::RainbowSpin: {
            // TODO
            return;
        }

        case UsrEffect::Joystick::TiltFollowPlain: {
            // TODO
            return;
        }

        case UsrEffect::Joystick::TiltFollowRainbow: {
            // TODO
            return;
        }
    }
}

void Led::_applyKey(uint32_t now) {
    const Effects &e = _effects;

    // uint8_t r, g, b;
    // e.key_event == KeyEvent::Short ? /*TODO*/ : /*TODO*/;
    // _fillAll(r, g, b);
}
