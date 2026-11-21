#pragma once

#include <cstdint>

#include "axp202.h"

class PWR {
    public:
        // Latest sampled and filtered values
        struct Status {
            bool     usbin           = false;
            bool     charging        = false;
            bool     battery_present = true;
            uint16_t battery_mV;
            uint8_t  battery_percent;
            float    charge_mA;
            float    discharge_mA;
            float    chip_temp_C;
        };

        // event flags, should be cleared by clearEvents() after reading
        struct Events {
            bool usb_plugged        = false;
            bool usb_unplugged      = false;
            bool charge_started     = false;
            bool charge_done        = false;
            bool low_battery        = false;
            bool critical_battery   = false;
            bool over_temp          = false;
        };

        PWR(uint8_t sda, uint8_t scl) : _pmu(sda, scl) {}

        bool begin();
        bool isReady() const { return _ok; }

        // Sample, filter and track the edges. Call about once a second
        bool update();

        const Status& getStatus() const { return _status; }
        const Events& getEvents() const { return _events; }
        void clearEvents();

        bool     isUsbIn() const { return _status.usbin; }
        bool     isCharging() const { return _status.charging; }
        uint8_t  getPercent() const { return _status.battery_percent; }
        uint16_t getMillivolts() const { return _status.battery_mV; }

        // Asked by the state machine when it needs to make a decision
        bool     isUsbPlugged() const { return _events.usb_plugged; }
        bool     isUsbUnplugged() const { return _events.usb_unplugged; }
        bool     isChargeStarted() const { return _events.charge_started; }
        bool     isChargeDone() const { return _events.charge_done; }
        bool     isLowBattery() const { return _events.low_battery; }
        bool     isCriticalBattery() const { return _events.critical_battery; }
        bool     isOverTemp() const { return _events.over_temp; }

        // the board dies if calls this
        void shutdown();

        AXP202& pmu() { return _pmu; }

    private:
        AXP202 _pmu;
        bool   _ok = false;

        Status _status = {};
        Events _events = {};

        static constexpr uint8_t PWR_LOW_PERCENT  = 20;
        static constexpr uint8_t PWR_CRITICAL_PERCENT = 5;
        static constexpr float   PWR_PERCENT_ALPHA = 0.12f; // low pass weight per update
        static constexpr float   PWR_OVER_TEMP_C = 60.0f;

        float _percent_f = 0.0f;   // float bat percentage for smoothing
        bool  _prev_usbin = false;
        bool  _prev_charging = false;
        bool  _prev_bat_present = false;

        void _latchEdges();
};
