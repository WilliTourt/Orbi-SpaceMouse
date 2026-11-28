#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// One place for the five task bodies
namespace tasks {

constexpr uint32_t SYS_STACK  = 4096;
constexpr uint32_t KEY_STACK  = 3072;
constexpr uint32_t JOY_STACK  = 6144;   // Motion holds a lot on the stack
constexpr uint32_t LED_STACK  = 3072;
constexpr uint32_t HID_STACK  = 3072;

constexpr UBaseType_t SYS_PRIO = 3;
constexpr UBaseType_t KEY_PRIO = 2;
constexpr UBaseType_t JOY_PRIO = 4;
constexpr UBaseType_t LED_PRIO = 1;
constexpr UBaseType_t HID_PRIO = 2;

constexpr uint32_t KEY_PERIOD_MS = 8;
constexpr uint32_t JOY_PERIOD_MS = 2;
constexpr uint32_t LED_PERIOD_MS = 25;
constexpr uint32_t HID_PERIOD_MS = 5;


// Events for messaging sys
enum class KeyEvent : uint8_t {
    None,
    Key1_Click, Key1_Double, Key1_Long,
    Key2_Click, Key2_Double, Key2_Long
};

enum class JoystickEvent : uint8_t {
    None,
    Active, Idle
};

enum class HidEvent : uint8_t {
    None,
    Connected, Disconnected,
    TransChangedToUSB, TransChangedToBLE
};

bool start();

} // namespace tasks
