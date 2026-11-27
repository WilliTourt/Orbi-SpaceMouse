#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "config.h"
#include "hid.h"
#include "joystick.h"
#include "key.h"
#include "led.h"
#include "pwr.h"

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

        enum class Event : uint8_t {
            KeyEvent,

            HostConnected,
            HostDisconnected,

            CalibrationDone,
            CalibrationFailed,

            BatteryLow,

            UsbInserted,
            UsbRemoved
        };

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

        State state() const  { return _state; }

        Config& config()     { return _cfg; }
        Led& led()           { return _led; }
        Joystick& joystick() { return _joy; }
        HID& hid()           { return _hid; }
        PWR& pwr()           { return _pwr; }

        // task bodies, called from tasks.cpp
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
        void _handleEvent(Event e);

        Config   _cfg;
        HID      _hid;
        Led      _led;
        Key      _key1;
        Key      _key2;
        Joystick _joy;
        PWR      _pwr;

        QueueHandle_t _queue = nullptr;         // joy/key/hid -> sys
        QueueHandle_t _pose_queue = nullptr;    // joystick -> hid
        State _state = State::Boot;
};
