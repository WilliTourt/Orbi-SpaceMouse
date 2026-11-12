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
#include "axp202.h"



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

AXP202 pmu(IIC_B_SDA, IIC_B_SCL);                            // IIC_B 0x34

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

void readMag(const char *name, TMAG3001 &s) {
    if (!s.read()) {
        printf("%s  read FAIL\n", name);
        return;
    }
    TMAG3001::Raw    raw = s.getRaw();
    TMAG3001::Sample tesla = s.getMt();
    printf("%s raw XYZ(%6d,%6d,%6d) (%7.2f,%7.2f,%7.2f) mT  %.1fC\n",
           name, raw.x, raw.y, raw.z, tesla.x_mT, tesla.y_mT, tesla.z_mT, s.getTemperature());
}

extern "C" void app_main(void) {
    // led_init();

    printf("\n=== Orbi bring-up ===\n");

    bool ok_u1 = s1.begin();
    bool ok_u2 = s2.begin();
    bool ok_u3 = s3.begin();
    printf("TMAG U1(0x34@A): %s\n", ok_u1 ? "OK" : "FAIL");
    printf("TMAG U2(0x35@A): %s\n", ok_u2 ? "OK" : "FAIL");
    printf("TMAG U3(0x35@B): %s\n", ok_u3 ? "OK" : "FAIL");

    bool ok_strip = strip.begin();
    printf("WS2812 x%d on IO45: %s\n", WS2812_COUNT, ok_strip ? "OK" : "FAIL");

    if (ok_strip) {
        strip.setBrightness(30);
        xTaskCreate(rainbow_task, "rainbow", 3072, nullptr, 3, nullptr);
    }

    bool ok_pmu = pmu.begin();
    printf("AXP202(0x34@B): %s\n", ok_pmu ? "OK" : "FAIL");
    if (ok_pmu) {
        pmu.setVbusCurrentLimit(AXP202::VbusCurrentLimit::Unlimited);
        pmu.setChargeEnabled(true);
        pmu.setChargeCurrent(1000);
        pmu.setVhold(AXP202::Vhold::V4_7);
        pmu.setVoff(AXP202::Voff::V2_8);
        pmu.enableIrq(AXP202_IRQ1_VBUS_INSERT | AXP202_IRQ1_VBUS_REMOVE,
                      0,
                      AXP202_IRQ3_CHIP_OVER_TEMP);
        printf("PMU rails: DCDC3(3V3)=%s\n", pmu.isRailEnabled(AXP202::PwrRail::DCDC3) ? "on" : "off");
        pmu.enableCoulomb(true);
        uint32_t chg = 0, dis = 0;
        if (pmu.readCoulomb(chg, dis)) {
            printf("PMU coulomb: chg=%lu dis=%lu\n",
                   (unsigned long)chg, (unsigned long)dis);
        }
    }

    printf("=== ready ===\n");

    uint8_t tick = 0;
    while (1) {
        gpio_set_level(PIN_LED, (tick & 1) ? 1 : 0);

        if (ok_u1) readMag("U1", s1);
        if (ok_u2) readMag("U2", s2);
        if (ok_u3) readMag("U3", s3);

        if ((tick % 20) == 0 && ok_pmu && pmu.update()) {
            printf("Battery: %umV   OCV: %u%%   FuelGauge: %u%%\n",
                pmu.getBatteryMillivolts(), pmu.getBatteryPercent(), pmu.getFuelGaugePercent());

            uint32_t chg = 0, dis = 0;
            if (pmu.readCoulomb(chg, dis)) {
                printf("PMU coulomb: chg=%lu dis=%lu net=%.2fmAh\n",
                        (unsigned long)chg, (unsigned long)dis,
                        pmu.getBatteryMah(chg, dis));
            }
        }

        printf("\n");

        tick++;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
