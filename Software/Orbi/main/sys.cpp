#include "sys.h"

#include "esp_timer.h"

#include "tasks.h"

static inline uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }

Sys& Sys::instance() {
    static Sys s;
    return s;
}

bool Sys::start() {

    // 1. config first, everything else reads it
    if (!_cfg.begin()) {
        // TODO
    }

    // 2. lights
    if (!_led.begin(CFG_PIN_WS2812, CFG_WS2812_COUNT)) { }
    _led.setBrightness(_cfg.get().led_brightness);
    _led.setSysEffects(Led::SysEffect::Boot);

    // 3. power, so the rails and charging are known good
    if (!_pwr.begin()) { } // TODO

    // 4. keys
    if (!_key1.begin()) { } // TODO
    if (!_key2.begin()) { } // TODO

    // 5. calibrate right away, the stick is at rest on boot. (TODO: or detecting still state?)
    _enterState(State::SelfTest); // TODO: difference light between selftest and boot? 
    if (!_joy.begin()) {
        _enterState(State::Fault);
    } else {
        _enterState(State::Calibrating);
        if (_joy.calibrate()) {
            if (!_cfg.get().calib_calibrated) {
                _cfg.get().calib_calibrated = true;
                _cfg.save();
            }
            _enterState(State::Running);
        } else {
            _enterState(State::Fault);
        }
    }

    // 6. host link   TODO: how about cheking wire connection to decide transtype
    if (!_hid.begin(_cfg.get().hid_transport == 2 ? HID::TransType::Usb : HID::TransType::Ble)) { } // TODO

    _queue = xQueueCreate(16, sizeof(Message));
    _pose_queue = xQueueCreate(1, sizeof(Motion::Pose)); 
    if (_queue == nullptr || _pose_queue == nullptr) {
        _enterState(State::Fault);
        return false;
    }

    if (!tasks::start()) { } // TODO
    return true;
}

bool Sys::post(const Message& msg) {
    if (_queue == nullptr) return false;

    // never block the caller
    return (xQueueSend(_queue, &msg, 0) == pdTRUE);
}

void Sys::_enterState(State s) {
    if (_state == s) return;

    _state = s;

    switch (s) {
        case State::Boot:        _led.setSysEffects(Led::SysEffect::Boot);             break;
        case State::SelfTest:    _led.setSysEffects(Led::SysEffect::Boot);             break;
        case State::Calibrating: _led.setSysEffects(Led::SysEffect::Calibrating);      break;
        case State::Running:     _led.setSysEffects(Led::SysEffect::ReadyUsrEffect);   break;
        case State::Idle:        _led.setSysEffects(Led::SysEffect::Idle);             break;
        case State::Fault:       _led.setSysEffects(Led::SysEffect::Error);            break;
    }
}

void Sys::_handleEvent(Event e) {
    switch (e) { 
            case Event::KeyEvent: {

                break;
            }

            case Event::HostConnected: {

                break;
            }
            case Event::HostDisconnected: {
                
                break;
            }

            case Event::CalibrationDone: {
                
                break;
            }
            case Event::CalibrationFailed: {
                
                break;
            }

            case Event::BatteryLow: {
                
                break;
            }

            case Event::UsbInserted: {
                
                break;
            }
            case Event::UsbRemoved: {
                
                break;
            }
    }
}





void Sys::sysLoop() {

    for (;;) {

    }
}

void Sys::keyLoop() {
    TickType_t last = xTaskGetTickCount();
    Key keys[2] = { _key1, _key2 };
    Message msg;

    for (;;) {
        for (int i = 0; i < 2; i++) {
            keys[i].update();

            msg.src = Message::Source::Key;
            if (keys[i].isSingleClicked()) { msg.event.key = (i ? tasks::KeyEvent::Key2_Click  : tasks::KeyEvent::Key1_Click);  }
            if (keys[i].isDoubleClicked()) { msg.event.key = (i ? tasks::KeyEvent::Key2_Double : tasks::KeyEvent::Key1_Double); }
            if (keys[i].isLongPressed())   { msg.event.key = (i ? tasks::KeyEvent::Key2_Long   : tasks::KeyEvent::Key1_Long);   }

            post(msg);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(tasks::KEY_PERIOD_MS));
    }
}

void Sys::joyLoop() {
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        
        vTaskDelayUntil(&last, pdMS_TO_TICKS(tasks::JOY_PERIOD_MS));
    }
}

void Sys::ledLoop() {
    TickType_t last = xTaskGetTickCount();

    for (;;) {

        vTaskDelayUntil(&last, pdMS_TO_TICKS(tasks::LED_PERIOD_MS));
    }
}

void Sys::hidLoop() {
    TickType_t last = xTaskGetTickCount();

    for (;;) {

        vTaskDelayUntil(&last, pdMS_TO_TICKS(tasks::HID_PERIOD_MS));
    }
}
