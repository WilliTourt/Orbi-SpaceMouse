#include "ws2812.h"
#include <new>
#include <cstring>

WS2812::WS2812(uint8_t pin, uint16_t num_leds)
    : _pin(pin), _num_leds(num_leds), _brightness(50), _strip(nullptr), _color(nullptr) {}

WS2812::~WS2812() {
    if (_strip) led_strip_del(_strip);
    delete[] _color;
}

bool WS2812::begin() {
    if (_strip) return true;

    led_strip_config_t cfg = {
        .strip_gpio_num = _pin,
        .max_leds = _num_leds,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .timings = {},
        .flags = {
            .invert_out = 0
        }
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 0,
        .mem_block_symbols = 0,
        .flags = {
            .with_dma = true
        }
    };

    _color = new (std::nothrow) RGB[_num_leds]();

    if (led_strip_new_rmt_device(&cfg, &rmt_config, &_strip) != ESP_OK) {
        delete[] _color;
        _color = nullptr;
        return false;
    }
    return true;
}

void WS2812::setColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (!_strip) return;
    if (index >= _num_leds) return;

    _color[index][0] = r;
    _color[index][1] = g;
    _color[index][2] = b;
}

void WS2812::setColor(uint8_t r, uint8_t g, uint8_t b) {
    if (!_strip) return;
    for (uint16_t i = 0; i < _num_leds; ++i) {
        _color[i][0] = r;
        _color[i][1] = g;
        _color[i][2] = b;
    }
}

// void WS2812::setBrightness(uint16_t index, uint8_t percent) {}

void WS2812::setBrightness(uint8_t percent) {
    percent = (percent > 100) ? 100 : percent;
    _brightness = percent;
}

void WS2812::refresh() {
    if (!_strip) return;
    for (uint16_t i = 0; i < _num_leds; ++i) {
        led_strip_set_pixel(_strip, i,
            _mb(_color[i][0]), _mb(_color[i][1]), _mb(_color[i][2]));
    }

    led_strip_refresh(_strip);
}

void WS2812::reset() {
    memset(_color, 0, _num_leds * sizeof(RGB));

    if (!_strip) return;
    led_strip_clear(_strip);
}

uint8_t WS2812::_mb(uint8_t chn) {
    return chn * _brightness / 100;
}
