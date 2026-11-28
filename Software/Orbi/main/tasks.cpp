#include "tasks.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sys.h"


static void sysTask(void *) {
    Sys::instance().sysLoop();
    vTaskDelete(nullptr);
}

static void keyTask(void *) {
    Sys::instance().keyLoop();
    vTaskDelete(nullptr);
}

static void joyTask(void *) {
    Sys::instance().joyLoop();
    vTaskDelete(nullptr);
}

static void ledTask(void *) {
    Sys::instance().ledLoop();
    vTaskDelete(nullptr);
}

static void hidTask(void *) {
    Sys::instance().hidLoop();
    vTaskDelete(nullptr);
}

bool tasks::start() {
    BaseType_t ok = pdPASS;
    TaskHandle_t h_joy = nullptr;
    TaskHandle_t h_led = nullptr;
    TaskHandle_t h_hid = nullptr;

    ok &= xTaskCreatePinnedToCore(keyTask, "key", KEY_STACK, nullptr, KEY_PRIO, nullptr, 0);
    ok &= xTaskCreatePinnedToCore(joyTask, "joy", JOY_STACK, nullptr, JOY_PRIO, &h_joy, 0);
    ok &= xTaskCreatePinnedToCore(hidTask, "hid", HID_STACK, nullptr, HID_PRIO, &h_hid, 1);
    ok &= xTaskCreatePinnedToCore(ledTask, "led", LED_STACK, nullptr, LED_PRIO, &h_led, 0);
    ok &= xTaskCreatePinnedToCore(sysTask, "sys", SYS_STACK, nullptr, SYS_PRIO, nullptr, 1);

    if (ok != pdPASS) return false;

    // the notify targets must be known
    Sys::instance().attachTasks(h_joy, h_led, h_hid);
    return true;
}
