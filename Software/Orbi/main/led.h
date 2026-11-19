#pragma once

#include <cstdint>
#include "ws2812.h"



class Led {
    public:
        enum class KeyEvent : uint8_t {
            Short,
            Long
        };

        struct Effects {

            enum class SysEffect : uint8_t {
                Off,
                Boot,           // rainbow sweep, once
                Idle,           // very dim breathing, tells "alive, not connected"
                ReadyUsrEffect, // connected, using user effect
                Calibrating,    // blue blink
                LowBattery,     // amber pulse
                Charging,       // green breathing
                Error           // red fast blink
            };

            struct UsrEffect {
                enum class Joystick : uint8_t {
                    Default,            // Default effect (single color always on)
                    BreathePlain,       // Single color breathing
                    BreatheWave,        // Single color breathing but two brightness center spins around the strip
                    RainbowPlain,       // Static rainbow
                    RainbowSpin,        // Spining rainbow
                    TiltFollowPlain,    // Single color, brightness follows tilt
                    TiltFollowRainbow   // Rainbow color, brightness follows tilt

                    // ... and more
                };

                struct Key {
                    enum class ShortPress : uint8_t {
                        None,
                        ShortPress
                    };
                    enum class LongPress : uint8_t {
                        None,
                        LongPress
                    };
                };

                // the two independent settings: what the stick does while it
                // rests, and how the keys react
                Joystick        joystick    = Joystick::Default;
                Key::ShortPress short_press = Key::ShortPress::None;
                Key::LongPress  long_press  = Key::LongPress::None;
            };

            // Live state of the light. Led owns exactly one Effects and every
            // renderer works off it, so nothing effect related is scattered.
            SysEffect sys = SysEffect::Off;
            UsrEffect usr = UsrEffect{};

            uint32_t sys_started = 0;
            uint32_t usr_started = 0;

            // key reaction in progress, key_len == 0 means there is none
            uint32_t key_started = 0;
            uint32_t key_len = 0;
            KeyEvent key_event = KeyEvent::Short;

            // caculated and scaled from joystick rx and ry
            float tilt_x = 0.0f;
            float tilt_y = 0.0f;

            uint8_t color_plain[3] = { 0, 0, 0 }; // single color R,G,B, from user config
        };

        // short names
        using SysEffect = Effects::SysEffect;
        using UsrEffect = Effects::UsrEffect;

        bool begin(uint8_t pin, uint16_t count);
        bool isReady() { return _strip != nullptr; }

        void setBrightness(uint8_t percent);
        void setSysEffects(SysEffect effect);
        void setUsrEffects(const UsrEffect &usr);
        // color_plain color used by the single color effects, comes from user config
        void setPlainColor(uint8_t r, uint8_t g, uint8_t b);
        
        // tilt x,y are [-1.0, 1.0]
        void reactTilt(float x, float y);
        // key reactions, only shown while ReadyUsrEffect owns the strip
        void reactKey(KeyEvent event);

        // call at a steady 20..30 ms; drives all animation
        void update();

        // These two getters can be used when PC reads config.
        // Now is not used
        // SysEffect getSys() const { return _effects.sys; }
        // const UsrEffect& getUsr() const { return _effects.usr; }

    private:
        void _fill(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
        void _fillAll(uint8_t r, uint8_t g, uint8_t b);
        void _setStatusLed(bool on); // extra single led on board

        // tool to convert hsv to rgb
        void _hsv(uint8_t h, uint8_t s, uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b);
        
        void _applySys(uint32_t now);
        void _applyJoystick(uint32_t now);
        void _applyKey(uint32_t now);

        // how long a key reaction sits over the joystick animation
        static constexpr uint32_t KEY_SHORT_MS = 120;
        static constexpr uint32_t KEY_LONG_MS  = 200;

        WS2812      *_strip = nullptr;
        uint16_t    _led_count = 0;

        Effects     _effects;
        uint8_t     _brightness = 30; // base brightness, different from effect brightness
};
