#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c_master.h"

#include "i2c_buspool.h"
#include "tmag3001.h"
#include "ws2812.h"



#define PIN_LED        GPIO_NUM_10
#define PIN_WS2812     GPIO_NUM_45
#define WS2812_COUNT   10

#define IIC_A_SDA      GPIO_NUM_6
#define IIC_A_SCL      GPIO_NUM_7
#define IIC_B_SDA      GPIO_NUM_13
#define IIC_B_SCL      GPIO_NUM_12

TMAG3001 s1(IIC_A_SDA, IIC_A_SCL, TMAG3001::AddrPin::toGND); // IIC_A 0x34
TMAG3001 s2(IIC_A_SDA, IIC_A_SCL, TMAG3001::AddrPin::toVCC); // IIC_A 0x35
TMAG3001 s3(IIC_B_SDA, IIC_B_SCL, TMAG3001::AddrPin::toVCC); // IIC_B 0x35

WS2812 strip(PIN_WS2812, WS2812_COUNT);

void led_init() {
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << PIN_LED);
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io);
    gpio_set_level(PIN_LED, 1);
}

// 扫一遍 7 位地址空间，把有 ACK 的地址打出来
void scan_bus(const char *name, i2c_master_bus_handle_t bus) {
    if (bus == nullptr) {
        printf("[IIC_%s] bus is NULL, skip scan\n", name);
        return;
    }

    printf("[IIC_%s] scanning 0x08~0x77 ...\n", name);
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
            printf("    ACK 0x%02X\n", addr);
            found++;
        }
    }
    printf("[IIC_%s] %d device(s) answered\n", name, found);
}

// ---------------- 彩虹 ----------------

// HSV -> RGB，h/s/v 都是 0~255
static void hsv2rgb(uint8_t h, uint8_t s, uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b) {
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

// 整条灯按相位铺满彩虹，phase 加一格就转一格
static void rainbow_step(uint8_t phase) {
    for (uint16_t i = 0; i < WS2812_COUNT; ++i) {
        uint8_t h = (uint8_t)(phase + (uint32_t)i * 255 / WS2812_COUNT);
        uint8_t r, g, b;
        hsv2rgb(h, 255, 160, r, g, b);
        strip.setColor(i, r, g, b);
    }
    strip.refresh();
}

static void rainbow_task(void *) {
    uint8_t phase = 0;
    while (1) {
        rainbow_step(phase++);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void dump(const char *name, TMAG3001 &s) {
    if (!s.read()) {
        printf("%s  read FAIL\n", name);
        return;
    }
    TMAG3001::Raw    r = s.getRaw();
    TMAG3001::Sample v = s.get();
    printf("%s raw(%6d,%6d,%6d) %7.2f %7.2f %7.2f mT  %.1fC\n",
           name, r.x, r.y, r.z, v.x_mT, v.y_mT, v.z_mT, s.getTemperature());
}

extern "C" void app_main(void) {
    led_init();

    printf("\n=== Orbi bring-up ===\n");

    bool ok1 = s1.begin();
    bool ok2 = s2.begin();
    bool ok3 = s3.begin();
    printf("TMAG U1(0x34@A): %s\n", ok1 ? "OK" : "FAIL");
    printf("TMAG U2(0x35@A): %s\n", ok2 ? "OK" : "FAIL");
    printf("TMAG U3(0x35@B): %s\n", ok3 ? "OK" : "FAIL");

    // 总线扫描：确认 U1 到底在不在、在哪个地址
    i2c_master_bus_handle_t busA = i2c_buspool_acquire(IIC_A_SDA, IIC_A_SCL);
    i2c_master_bus_handle_t busB = i2c_buspool_acquire(IIC_B_SDA, IIC_B_SCL);
    printf("--- bus scan (expect A:34,35  B:34*AXP,35) ---\n");
    scan_bus("A", busA);
    scan_bus("B", busB);
    if (busA) i2c_buspool_release(busA);
    if (busB) i2c_buspool_release(busB);
    printf("--- scan done ---\n");

    bool ok_strip = strip.begin();
    printf("WS2812 x%d on IO45: %s\n", WS2812_COUNT, ok_strip ? "OK" : "FAIL");

    if (ok_strip) {
        strip.setBrightness(10);
        // 彩虹自己跑一个任务，主循环还要打印传感器，别互相拖
        xTaskCreate(rainbow_task, "rainbow", 3072, nullptr, 3, nullptr);
    }
    printf("=== ready ===\n");

    int tick = 0;
    while (1) {
        gpio_set_level(PIN_LED, (tick & 1) ? 1 : 0);

        if (ok1) dump("U1", s1);
        if (ok2) dump("U2", s2);
        if (ok3) dump("U3", s3);
        printf("----\n");

        tick++;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
