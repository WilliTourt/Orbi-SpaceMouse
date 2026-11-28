#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "config.h"
#include "hid.h"
#include "joystick.h"
#include "key.h"
#include "led.h"
#include "pwr.h"

#include "tasks.h"

// Composition root and state machine. Everything else posts events here and
// lets this decide what the board should be doing.
class Sys {
    public:
        enum class State : uint8_t {
            Boot,
            SelfTest,
            Calibrating,
            Running,
            Idle,       // no host, waiting
            Fault
        };

        // enum class Event : uint8_t {
        //     KeyEvent,
        //     HostConnected, HostDisconnected,
        //     CalibrationDone, CalibrationFailed,
        //     BatteryLow,
        //     UsbInserted, UsbRemoved
        // };

        // queue struct
        struct Message {
            enum class Source : uint8_t {
                Key, Joystick, Hid
            } src;
            union Event {
                tasks::KeyEvent key;
                tasks::JoystickEvent joy;
                tasks::HidEvent hid;
            } event;
        };

        static Sys& instance();

        // creates the objects and the five tasks; call once from app_main
        bool start();
        bool post(const Message& msg);

        // called by tasks.cpp after the tasks exist
        void attachTasks(TaskHandle_t joy, TaskHandle_t led, TaskHandle_t hid);

        State state() const  { return _state; }

        Config& config()     { return _cfg; }
        Led& led()           { return _led; }
        Joystick& joystick() { return _joy; }
        HID& hid()           { return _hid; }
        PWR& pwr()           { return _pwr; }

        // task bodies, called by tasks.cpp
        void sysLoop();
        void keyLoop();
        void joyLoop();
        void ledLoop();
        void hidLoop();

    private:
        Sys() : _hid(&_cfg.get()),
                _key1((gpio_num_t)CFG_PIN_KEY1),
                _key2((gpio_num_t)CFG_PIN_KEY2),
                _pwr(CFG_IIC_B_SDA, CFG_IIC_B_SCL) {}

        void _enterState(State s);
        // void _handleEvent(Event e);

        constexpr static float JOYSTICK_ACTIVE_DEADZONE_MM = 0.1f;
        constexpr static float JOYSTICK_ACTIVE_DEADZONE_RAD = 0.003f;

        Config   _cfg;
        HID      _hid;
        Led      _led;
        Key      _key1;
        Key      _key2;
        Joystick _joy;
        PWR      _pwr;

        // queues
        QueueHandle_t _event_queue = nullptr;    // joy/key/hid -> sys
        QueueHandle_t _pose_hid_queue = nullptr; // joystick -> hid
        QueueHandle_t _pose_led_queue = nullptr; // joystick -> led for light effects
        QueueHandle_t _bat_hid_queue = nullptr;  // sys -> hid, battery val

        // notifications
        // sys -> hid   key events (bits 5 - 0) and connection settings (bits higher)
        // sys -> led   key events (bits 5 - 0) and lp alarm (bit 6) and connection settings (bits higher)
        // sys -> joy   calibration notification (bit 0)

        // hid notification bits
        static constexpr uint32_t HID_BIT_KEY1_SHORT  = 1u << 0;
        static constexpr uint32_t HID_BIT_KEY1_DOUBLE = 1u << 1;
        static constexpr uint32_t HID_BIT_KEY1_LONG   = 1u << 2;
        static constexpr uint32_t HID_BIT_KEY2_SHORT  = 1u << 3;
        static constexpr uint32_t HID_BIT_KEY2_DOUBLE = 1u << 4;
        static constexpr uint32_t HID_BIT_KEY2_LONG   = 1u << 5;
        // TODO static constexpr uint32_t HID_CONN_XXX

        // led notification bits
        static constexpr uint32_t LED_BIT_KEY1_SHORT  = HID_BIT_KEY1_SHORT;
        static constexpr uint32_t LED_BIT_KEY1_DOUBLE = HID_BIT_KEY1_DOUBLE;
        static constexpr uint32_t LED_BIT_KEY1_LONG   = HID_BIT_KEY1_LONG;
        static constexpr uint32_t LED_BIT_KEY2_SHORT  = HID_BIT_KEY2_SHORT;
        static constexpr uint32_t LED_BIT_KEY2_DOUBLE = HID_BIT_KEY2_DOUBLE;
        static constexpr uint32_t LED_BIT_KEY2_LONG   = HID_BIT_KEY2_LONG;
        static constexpr uint32_t LED_BIT_LOW_PWR     = 1u << 6;
        // TODO static constexpr uint32_t LED_CONN_XXX

        // joy notification bits
        static constexpr uint32_t JOY_BIT_RECALIBRATE = 1u << 0;

        TaskHandle_t _hid_task = nullptr;
        TaskHandle_t _led_task = nullptr;
        TaskHandle_t _joy_task = nullptr;

        State _state = State::Boot;
};
