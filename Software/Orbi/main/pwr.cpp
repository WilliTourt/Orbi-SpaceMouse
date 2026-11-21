#include "pwr.h"

bool PWR::begin() {
    if (!_pmu.begin()) {
        _ok = false;
        return false;
    }

    _pmu.setChargeTarget(AXP202::ChargeTarget::V4_20);
    _pmu.setChargeCurrent(1500);
    _pmu.setVbusCurrentLimit(AXP202::VbusCurrentLimit::Unlimited);
    _pmu.setVhold(AXP202::Vhold::V4_7);
    _pmu.setVoff(AXP202::Voff::V2_8);

    // Coulomb counter for battery capacity estimate
    _pmu.enableCoulomb(true);
    _pmu.clearCoulomb();

    // TODO
    // not used now. in future low power mode wakeup will be used
    // _pmu.enableIrq(AXP202_IRQ1_VBUS_INSERT | AXP202_IRQ1_VBUS_REMOVE,
    //                AXP202_IRQ2_CHG_START | AXP202_IRQ2_CHG_DONE,
    //                AXP202_IRQ3_CHIP_OVER_TEMP);

    _status = Status();
    _events = Events();
    _percent_f = 0.0f;
    _prev_usbin = false;
    _prev_charging = false;
    _prev_bat_present = false;

    _ok = true;
    if (!update()) {
        _ok = false;
        return false;
    }
    return true;
}

bool PWR::update() {
    if (!_ok) return false;
    if (!_pmu.update()) return false;

    const AXP202::Measurement& meas = _pmu.getMeasurement();
    // AXP202::Irq irq = {};
    // if (_pmu.readIrq(irq)) {
    //     if (irq.vbus_insert)    _events.usb_plugged = true;
    //     if (irq.vbus_remove)    _events.usb_unplugged = true;
    //     if (irq.chg_start)      _events.charge_started = true;
    //     if (irq.chg_done)       _events.charge_done = true;
    //     if (irq.chip_over_temp) _events.over_temp = true;
    //     _pmu.clearIrq();
    // }

    _status.usbin           = meas.vbus_present;
    _status.charging        = meas.charging;
    _status.battery_present = meas.bat_present;
    _status.battery_mV      = meas.battery_mV;
    _status.charge_mA       = meas.charge_mA;
    _status.discharge_mA    = meas.discharge_mA;
    _status.chip_temp_C     = meas.chip_temp_C;

    // Percentage. The hardware gauge is already averaged by the chip, the
    // voltage table is the fallback before the gauge has settled.
    const uint8_t old_percent = _status.battery_percent;

    if (!_status.battery_present) {
        _percent_f = 0.0f;
    } else {
        float target = static_cast<float>(meas.battery_percent);
        const uint8_t gauge = _pmu.getFuelGaugePercent();
        if (gauge <= 100) target = static_cast<float>(gauge);

        // Seed on the first sample, then low pass. Without this the number
        // jumps a few percent whenever the load changes.
        if (_percent_f <= 0.0f) _percent_f = target;
        else _percent_f += PWR_PERCENT_ALPHA * (target - _percent_f);

        if (_percent_f < 0.0f)   _percent_f = 0.0f;
        if (_percent_f > 100.0f) _percent_f = 100.0f;
    }

    _status.battery_percent = static_cast<uint8_t>(_percent_f + 0.5f);

    // falling edge trigger
    if (old_percent > PWR_LOW_PERCENT && _status.battery_percent <= PWR_LOW_PERCENT)
        _events.low_battery = true;
    if (old_percent > PWR_CRITICAL_PERCENT && _status.battery_percent <= PWR_CRITICAL_PERCENT)
        _events.critical_battery = true;

    _latchEdges();
    return true;
}

void PWR::_latchEdges() {
    if (_status.usbin && !_prev_usbin)                          _events.usb_plugged = true;
    if (!_status.usbin && _prev_usbin)                          _events.usb_unplugged = true;

    if (_status.charging && !_prev_charging)                    _events.charge_started = true;

    // Unplugging also stops the charge, that is not a charge done
    if (!_status.charging && _prev_charging && _status.usbin)   _events.charge_done = true;

    if (_status.chip_temp_C > PWR_OVER_TEMP_C)                  _events.over_temp = true;

    _prev_usbin = _status.usbin;
    _prev_charging = _status.charging;
    _prev_bat_present = _status.battery_present;
}

void PWR::clearEvents() {
    _events = Events();
}

void PWR::shutdown() {
    if (!_ok) return;

    // Stop the counter first, otherwise it keeps integrating while the rails are still up.
    _pmu.enableCoulomb(false);
    _pmu.shutdown();
}
