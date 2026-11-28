#include "sys.h"

#include <cmath>

#include "esp_timer.h"


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

    _event_queue    = xQueueCreate(16, sizeof(Message));
    _pose_hid_queue = xQueueCreate( 1, sizeof(Motion::Pose)); 
    _pose_led_queue = xQueueCreate( 1, sizeof(Motion::Pose)); 
    _bat_hid_queue  = xQueueCreate( 4, sizeof(uint8_t));
    if (_event_queue == nullptr || _pose_hid_queue == nullptr ||
        _pose_led_queue == nullptr || _bat_hid_queue == nullptr) {
        _enterState(State::Fault);
        return false;
    }

    // WARNING: AFTER tasks::start(), ALL OBJECTS SETTER MUST NOT BE CALLED IN SYSTEM TASK
    // EACH OBJECT (_key, _led, _hid, _joystick) IS ONLY MANAGED BY THAT SPECIAIZED TASK 

    if (!tasks::start()) { } // TODO
    return true;
}

bool Sys::post(const Message& msg) {
    if (_event_queue == nullptr) return false;

    // never block the caller
    return (xQueueSend(_event_queue, &msg, 0) == pdTRUE);
}

void Sys::attachTasks(TaskHandle_t joy, TaskHandle_t led, TaskHandle_t hid) {
    _joy_task = joy;
    _led_task = led;
    _hid_task = hid;
}

void Sys::_enterState(State s) {
    if (_state == s) return;

    _state = s;

    Led::SysEffect effect = Led::SysEffect::Boot;
    switch (s) {
        case State::Boot:        effect = Led::SysEffect::Boot;           break;
        case State::SelfTest:    effect = Led::SysEffect::Boot;           break;
        case State::Calibrating: effect = Led::SysEffect::Calibrating;    break;
        case State::Running:     effect = Led::SysEffect::ReadyUsrEffect; break;
        case State::Idle:        effect = Led::SysEffect::Idle;           break;
        case State::Fault:       effect = Led::SysEffect::Error;          break;
    }

    // Before the led task exists the object belongs to nobody, safe to touch.
    // Afterwards it is owned by ledLoop, so hand the request over instead.
    if (_led_task == nullptr) {
        _led.setSysEffects(effect);
    } else {
        _led_effect_req = effect;
        xTaskNotify(_led_task, LED_BIT_EFFECT, eSetBits);
    }
}

// void Sys::_handleEvent(Event e) {
//     switch (e) { 
//             case Event::KeyEvent: {
//                 break;
//             }
//             case Event::HostConnected: {
//                 break;
//             }
//             case Event::HostDisconnected: {
//                 break;
//             }
//             case Event::CalibrationDone: {       
//                 break;
//             }
//             case Event::CalibrationFailed: {      
//                 break;
//             }
//             case Event::BatteryLow: {           
//                 break;
//             }
//             case Event::UsbInserted: {                
//                 break;
//             }
//             case Event::UsbRemoved: {               
//                 break;
//             }
//     }
// }





void Sys::sysLoop() {
    Message msg;

    for (;;) {
        if (xQueueReceive(_event_queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (msg.src) {
                case Message::Source::Hid: {
                    switch (msg.event.hid) {
                        case tasks::HidEvent::Connected:
// TODO
                            break;

                        case tasks::HidEvent::Disconnected:
// TODO
                            break;

                        case tasks::HidEvent::TransChangedToUSB:
// TODO
                            break;

                        case tasks::HidEvent::TransChangedToBLE:
// TODO
                            break;
                        
                        default: break;
                    }
                    break;
                }

                case Message::Source::Joystick: {
                    switch (msg.event.joy) {
                        case tasks::JoystickEvent::Active:
// TODO
                            break;

                        case tasks::JoystickEvent::Idle:
// TODO
                            break;

                        default: break;
                    }
                    break;
                }

                case Message::Source::Key: {
                    switch (msg.event.key) {
                        case tasks::KeyEvent::Key1_Click:
// TODO
                            break;

                        case tasks::KeyEvent::Key1_Double:
// TODO
                            break;

                        case tasks::KeyEvent::Key1_Long:
// TODO
                            break;

                        case tasks::KeyEvent::Key2_Click:
// TODO
                            break;

                        case tasks::KeyEvent::Key2_Double:
// TODO
                            break;

                        case tasks::KeyEvent::Key2_Long:
// TODO
                            break;

                        default: break;
                    }
                    break;
                }
            }
        }

        // TODO: PWR AND ELSE
        
    }
}

void Sys::keyLoop() {
    TickType_t last = xTaskGetTickCount();
    Key *keys[2] = { &_key1, &_key2 };
    Message msg = {};

    typedef tasks::KeyEvent tskKeyEvent;

    for (;;) {
        for (int i = 0; i < 2; i++) {
            keys[i]->update();

            msg.src = Message::Source::Key;
            if      (keys[i]->isSingleClicked()) { msg.event.key = (i ? tskKeyEvent::Key2_Click  : tskKeyEvent::Key1_Click);  }
            else if (keys[i]->isDoubleClicked()) { msg.event.key = (i ? tskKeyEvent::Key2_Double : tskKeyEvent::Key1_Double); }
            else if (keys[i]->isLongPressed())   { msg.event.key = (i ? tskKeyEvent::Key2_Long   : tskKeyEvent::Key1_Long);   }
            else { msg.event.key = tskKeyEvent::None; }

            if (msg.event.key != tskKeyEvent::None) post(msg);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(tasks::KEY_PERIOD_MS));
    }
}

void Sys::joyLoop() {
    TickType_t last = xTaskGetTickCount();
    Motion::Pose pose_now;
    Message msg = {};

    bool active = false;

    typedef tasks::JoystickEvent tskJoyEvent;

    for (;;) {
        // sys asked for a recalibration
        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            _joy.calibrate();   // TODO: report the result back to sys
        }

        _joy.update();
        pose_now = _joy.getPose();

        if (xQueueOverwrite(_pose_hid_queue, &pose_now) == pdPASS) {
            // TODO
        } 

        if (xQueueOverwrite(_pose_led_queue, &pose_now) == pdPASS) {
            // TODO
        }

        const bool now_active = (fabsf(pose_now.x)  > JOYSTICK_ACTIVE_DEADZONE_MM  ||
                                 fabsf(pose_now.y)  > JOYSTICK_ACTIVE_DEADZONE_MM  ||
                                 fabsf(pose_now.z)  > JOYSTICK_ACTIVE_DEADZONE_MM  ||
                                 fabsf(pose_now.rx) > JOYSTICK_ACTIVE_DEADZONE_RAD ||
                                 fabsf(pose_now.ry) > JOYSTICK_ACTIVE_DEADZONE_RAD ||
                                 fabsf(pose_now.rz) > JOYSTICK_ACTIVE_DEADZONE_RAD);
        
        msg.src = Message::Source::Joystick;
        msg.event.joy = now_active ? tskJoyEvent::Active : tskJoyEvent::Idle;
        if (now_active != active) {
            active = now_active;
            post(msg);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(tasks::JOY_PERIOD_MS));
    }
}

void Sys::ledLoop() {
    TickType_t last = xTaskGetTickCount();
    Motion::Pose pose_now;
    uint32_t notif_bits = 0;

    typedef Led::UsrEffect::Joystick LedJoystickEffect;

    for (;;) {
        // sys notification
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notif_bits, 0) == pdTRUE) {
            if (notif_bits & LED_BIT_EFFECT) _led.setSysEffects(_led_effect_req);
            if (notif_bits & LED_BIT_KEY1_SHORT) _led.reactKey(Led::KeyEvent::ShortR);
            if (notif_bits & LED_BIT_KEY1_LONG)  _led.reactKey(Led::KeyEvent::LongR);
            if (notif_bits & LED_BIT_KEY2_SHORT) _led.reactKey(Led::KeyEvent::ShortL);
            if (notif_bits & LED_BIT_KEY2_LONG)  _led.reactKey(Led::KeyEvent::LongL);
            // TODO: bits OR operation in reactKey?
        }

        Led::UsrEffect usrEff = _led.getUsrEffects();
        if (usrEff.joystick == LedJoystickEffect::TiltFollowPlain ||
            usrEff.joystick == LedJoystickEffect::TiltFollowRainbow) {
            if (xQueueReceive(_pose_led_queue, &pose_now, pdMS_TO_TICKS(1)) == pdTRUE) {
                float norm_rx, norm_ry;
                norm_rx = pose_now.x / _cfg.get().joystick_full_scale_mm;
                norm_ry = pose_now.y / _cfg.get().joystick_full_scale_mm;

                _led.reactTilt(norm_rx, norm_ry);
            }
        }

        _led.update();

        vTaskDelayUntil(&last, pdMS_TO_TICKS(tasks::LED_PERIOD_MS));
    }
}

void Sys::hidLoop() {
    TickType_t last = xTaskGetTickCount(); // TODO
    Motion::Pose pose_now;
    uint8_t bat_val = 0;

    for (;;) {
        while (xQueueReceive(_bat_hid_queue, &bat_val, 0) == pdTRUE) {
            _hid.setBattery(bat_val);
        }

        if (xQueueReceive(_pose_hid_queue, &pose_now, portMAX_DELAY) == pdTRUE) {

        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(tasks::HID_PERIOD_MS)); // TODO
    }
}
