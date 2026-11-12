#include "axp202.h"

#include "esp_rom_sys.h"

#include "i2c_buspool.h"

#define AXP202_TIMEOUT_MS 100

AXP202::AXP202(uint8_t sda, uint8_t scl) :
    _sda(sda), _scl(scl) {}

AXP202::~AXP202() {
    // Remove the device first, then hand the bus back (order matters)
    if (_dev != nullptr) {
        i2c_master_bus_rm_device(_dev);
        _dev = nullptr;
    }
    if (_bus != nullptr) {
        i2c_buspool_release(_bus);
        _bus = nullptr;
    }
}

/* ------------------------- Low level register access ------------------------- */

bool AXP202::_readReg(uint8_t reg, uint8_t* data, size_t len) {
    if (_dev == nullptr || data == nullptr || len == 0) return false;
    return i2c_master_transmit_receive(_dev, &reg, 1, data, len, AXP202_TIMEOUT_MS) == ESP_OK;
}

bool AXP202::_writeReg(uint8_t reg, uint8_t data) {
    if (_dev == nullptr) return false;
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(_dev, buf, 2, AXP202_TIMEOUT_MS) == ESP_OK;
}

// Modify only the bits covered by mask, leave the rest untouched
bool AXP202::_updateReg(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t old = 0;
    if (!_readReg(reg, &old, 1)) return false;
    uint8_t now = static_cast<uint8_t>((old & ~mask) | (value & mask));
    if (now == old) return true;
    return _writeReg(reg, now);
}

/* ------------------------- Init ------------------------- */

bool AXP202::begin() {
    if (_dev != nullptr) return true;

    if (!i2c_buspool_init()) return false;

    _bus = i2c_buspool_acquire(_sda, _scl);
    if (_bus == nullptr) return false;

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = AXP202_I2C_ADDR;
    dev_cfg.scl_speed_hz = 400 * 1000;
    dev_cfg.scl_wait_us = 0;
    dev_cfg.flags.disable_ack_check = 0;

    if (i2c_master_bus_add_device(_bus, &dev_cfg, &_dev) != ESP_OK) {
        _dev = nullptr;
        i2c_buspool_release(_bus);
        _bus = nullptr;
        return false;
    }

    uint8_t id = 0;
    
    if (!_readReg(AXP202_REG_IC_TYPE, &id, 1) || (id != AXP202_CHIP_ID && id != AXP202_CHIP_ID_ALT)) {
        i2c_master_bus_rm_device(_dev);
        _dev = nullptr;
        i2c_buspool_release(_bus);
        _bus = nullptr;
        return false;
    }

    // Turn on the ADC channels we read from (only battery ones are on by default)
    const uint8_t adc1 = AXP202_ADC1_BAT_VOL | AXP202_ADC1_BAT_CUR |
                         AXP202_ADC1_VBUS_VOL | AXP202_ADC1_VBUS_CUR;
    _updateReg(AXP202_REG_ADC_EN1, adc1, adc1);
    _updateReg(AXP202_REG_ADC_EN2, AXP202_ADC2_TEMP, AXP202_ADC2_TEMP);

    // Lock the charge profile instead of relying on the factory default nibble
    setChargeTarget(ChargeTarget::V4_20);
    setChargeCurrent(500);
    setVbusCurrentLimit(VbusCurrentLimit::mA900);

    // Sample rate is needed by the coulomb counter formula: 25 << n Hz
    uint8_t rate = 0;
    if (_readReg(AXP202_REG_ADC_RATE, &rate, 1)) {
        _adc_rate_hz = static_cast<uint16_t>(25 << ((rate >> 6) & 0x03));
    }

    return true;
}

/* ------------------------- Sampling ------------------------- */

bool AXP202::update() {
    // 0x00..0x01: VBUS present + charge status
    uint8_t status[2] = {0, 0};
    if (!_readReg(AXP202_REG_STATUS, status, sizeof(status))) return false;

    // 0x5A..0x5F: VBUS voltage/current, die temperature (6 bytes, auto-increment)
    uint8_t vbus[6] = {0};
    if (!_readReg(AXP202_REG_VBUS_ADC, vbus, sizeof(vbus))) return false;

    // 0x78..0x7D: battery voltage, charge current, discharge current (6 bytes)
    uint8_t battery[6] = {0};
    if (!_readReg(AXP202_REG_BAT_ADC, battery, sizeof(battery))) return false;

    _meas.vbus_present = (status[0] & AXP202_STATUS_VBUS_PRESENT) != 0;
    _meas.over_temp    = (status[1] & AXP202_MODE_OVER_TEMP) != 0;
    _meas.charging     = (status[1] & AXP202_MODE_CHARGING) != 0;
    _meas.bat_present  = (status[1] & AXP202_MODE_BAT_PRESENT) != 0;

    // 12-bit: high 8 bits + low 4 bits
    _meas.vbus_mV    = static_cast<uint16_t>(((vbus[0] << 4) | (vbus[1] & 0x0F)) * 1.7f);
    _meas.vbus_mA    = ((vbus[2] << 4) | (vbus[3] & 0x0F)) * 0.375f;
    _meas.chip_temp_C = ((vbus[4] << 4) | (vbus[5] & 0x0F)) * 0.1f - 144.7f;

    _meas.battery_mV = static_cast<uint16_t>(((battery[0] << 4) | (battery[1] & 0x0F)) * 1.1f);
    // 13-bit: high 8 bits + low 5 bits
    _meas.charge_mA    = ((battery[2] << 5) | (battery[3] & 0x1F)) * 0.5f;
    _meas.discharge_mA = ((battery[4] << 5) | (battery[5] & 0x1F)) * 0.5f;

    _meas.battery_percent = _meas.bat_present ? _percentFromMillivolts(_meas.battery_mV) : 0;
    return true;
}

uint8_t AXP202::_percentFromMillivolts(uint16_t mv) const {
    static const uint16_t adc_table[11] = { // Rough
        3000, 3650, 3700, 3740, 3760, 3795,
        3840, 3910, 3980, 4070, 4150
    };
    if (mv < adc_table[0]) return 0;

    for (int i = 1; i < 11; ++i) {
        if (mv < adc_table[i]) {
            uint16_t span = static_cast<uint16_t>(adc_table[i] - adc_table[i - 1]);
            return static_cast<uint8_t>(i * 10 - (10 * (adc_table[i] - mv)) / span);
        }
    }
    return 100;
}

/* ------------------------- Charging ------------------------- */

bool AXP202::setChargeTarget(ChargeTarget target) {
    return _updateReg(AXP202_REG_CHARGE_CTL1, AXP202_CHG_TARGET_MASK,
                      static_cast<uint8_t>(static_cast<uint8_t>(target) << 5));
}

bool AXP202::setVbusCurrentLimit(VbusCurrentLimit limit) {
    return _updateReg(AXP202_REG_VBUS_IPSOUT, AXP202_VBUS_ILIM_MASK,
                      static_cast<uint8_t>(static_cast<uint8_t>(limit) & AXP202_VBUS_ILIM_MASK));
}

bool AXP202::setVhold(Vhold level) {
    if (level == Vhold::Off) {
        return _updateReg(AXP202_REG_VBUS_IPSOUT, AXP202_VHOLD_EN, 0x00);
    }
    uint8_t bits = static_cast<uint8_t>(
        AXP202_VHOLD_EN | ((static_cast<uint8_t>(level) << 3) & AXP202_VHOLD_MASK));
    return _updateReg(AXP202_REG_VBUS_IPSOUT, AXP202_VHOLD_EN | AXP202_VHOLD_MASK, bits);
}

bool AXP202::setVoff(Voff level) {
    return _updateReg(AXP202_REG_VOFF, AXP202_VOFF_MASK,
                      static_cast<uint8_t>(static_cast<uint8_t>(level) & AXP202_VOFF_MASK));
}

bool AXP202::setChargeLedMode(ChargeLedMode mode) {
    const uint8_t bits = static_cast<uint8_t>(
        (static_cast<uint8_t>(mode) << 4) | AXP202_CHGLED_CTL_BY_REG);
    return _updateReg(AXP202_REG_SHUTDOWN, AXP202_CHGLED_MODE_MASK | AXP202_CHGLED_CTL_BY_REG, bits);
}

bool AXP202::setChargeCurrent(uint16_t milliamps) {
    if (milliamps < AXP202_CHG_CURRENT_MIN) milliamps = AXP202_CHG_CURRENT_MIN;
    if (milliamps > AXP202_CHG_CURRENT_MAX) milliamps = AXP202_CHG_CURRENT_MAX;

    const uint16_t steps = static_cast<uint16_t>((milliamps - AXP202_CHG_CURRENT_MIN) /
                                                 AXP202_CHG_CURRENT_STEP);
    return _updateReg(AXP202_REG_CHARGE_CTL1, AXP202_CHG_CURRENT_MASK,
                      static_cast<uint8_t>(steps & AXP202_CHG_CURRENT_MASK));
}

bool AXP202::setChargeEnabled(bool on) {
    return _updateReg(AXP202_REG_CHARGE_CTL1, AXP202_CHG_EN, on ? AXP202_CHG_EN : 0x00);
}

/* ------------------------- Power rails ------------------------- */

bool AXP202::isRailEnabled(PwrRail rail) {
    const uint8_t bit = static_cast<uint8_t>(rail);
    uint8_t vbus = 0;
    if (!_readReg(AXP202_REG_LDO234_DC23_CTL, &vbus, 1)) return false;
    return (vbus & bit) != 0;
}

bool AXP202::powerUp(PwrRail rail) {
    const uint8_t bit = static_cast<uint8_t>(rail);
    return _updateReg(AXP202_REG_LDO234_DC23_CTL, bit, bit);
}

bool AXP202::powerDown(PwrRail rail) {
    return _updateReg(AXP202_REG_LDO234_DC23_CTL, static_cast<uint8_t>(rail), 0x00);
}

bool AXP202::shutdown() {
    return _writeReg(AXP202_REG_SHUTDOWN, AXP202_SHUTDOWN_BIT);
}

/* ------------------------- Coulomb counter ------------------------- */

bool AXP202::enableCoulomb(bool on) {
    return _updateReg(AXP202_REG_COULOMB_CTL, AXP202_COULOMB_EN,
                      on ? AXP202_COULOMB_EN : 0x00);
}

bool AXP202::clearCoulomb() {
    // Bit5 clears itself once the counters are zeroed
    return _updateReg(AXP202_REG_COULOMB_CTL, AXP202_COULOMB_CLEAR, AXP202_COULOMB_CLEAR);
}

bool AXP202::readCoulomb(uint32_t& charge, uint32_t& discharge) {
    uint8_t ctl = 0;
    if (!_readReg(AXP202_REG_COULOMB_CTL, &ctl, 1)) return false;

    // Writing the unlock bit makes the counter registers readable; bit0 reports done
    if (!_writeReg(AXP202_REG_COULOMB_CTL,
                   static_cast<uint8_t>(ctl | AXP202_COULOMB_UNLOCK))) return false;

    for (int i = 0; i < 5; ++i) {
        if (_readReg(AXP202_REG_COULOMB_CTL, &ctl, 1) && (ctl & AXP202_COULOMB_UNLOCKED)) break;
        esp_rom_delay_us(200);
    }

    // 0xB0..0xB7: charge counter then discharge counter, both big endian
    uint8_t buf[8] = {0};
    if (!_readReg(AXP202_REG_COULOMB_CHG, buf, sizeof(buf))) return false;

    charge    = (static_cast<uint32_t>(buf[0]) << 24) |
                (static_cast<uint32_t>(buf[1]) << 16) |
                (static_cast<uint32_t>(buf[2]) << 8)  | buf[3];
    discharge = (static_cast<uint32_t>(buf[4]) << 24) |
                (static_cast<uint32_t>(buf[5]) << 16) |
                (static_cast<uint32_t>(buf[6]) << 8)  | buf[7];
    return true;
}

float AXP202::getBatteryMah(uint32_t charge, uint32_t discharge) const {
    // Datasheet: C = 65536 * current_lsb * (charge - discharge) / 3600 / ADC rate
    const float delta = static_cast<float>(static_cast<int64_t>(charge) -
                                          static_cast<int64_t>(discharge));
    const float rate = (_adc_rate_hz == 0) ? 25.0f : static_cast<float>(_adc_rate_hz);
    return 65536.0f * 0.5f * delta / 3600.0f / rate;
}

uint8_t AXP202::getFuelGaugePercent() {
    uint8_t percent = 0;
    if (!_readReg(AXP202_REG_FUEL_GAUGE, &percent, 1)) return 0;
    return static_cast<uint8_t>(percent & AXP202_FUELGAUGE_PERCENT);
}

/* ------------------------- Interrupts ------------------------- */

bool AXP202::readIrq(Irq& irq) {
    if (!_readReg(AXP202_REG_IRQ_STATUS1, _irq_raw, sizeof(_irq_raw))) return false;

    const uint8_t s1 = _irq_raw[0];
    const uint8_t s2 = _irq_raw[1];
    const uint8_t s3 = _irq_raw[2];

    irq.vbus_insert    = (s1 & AXP202_IRQ1_VBUS_INSERT) != 0;
    irq.vbus_remove    = (s1 & AXP202_IRQ1_VBUS_REMOVE) != 0;
    irq.bat_insert     = (s2 & AXP202_IRQ2_BAT_INSERT) != 0;
    irq.bat_remove     = (s2 & AXP202_IRQ2_BAT_REMOVE) != 0;
    irq.chg_start      = (s2 & AXP202_IRQ2_CHG_START) != 0;
    irq.chg_done       = (s2 & AXP202_IRQ2_CHG_DONE) != 0;
    irq.pekey_short    = (s3 & AXP202_IRQ3_PEKEY_SHORT) != 0;
    irq.pekey_long     = (s3 & AXP202_IRQ3_PEKEY_LONG) != 0;
    irq.chip_over_temp = (s3 & AXP202_IRQ3_CHIP_OVER_TEMP) != 0;

    irq.any = (s1 | s2 | s3) != 0;
    return true;
}

bool AXP202::clearIrq() {
    // Status bits are cleared by writing 1, so write back what we just read
    if ((_irq_raw[0] | _irq_raw[1] | _irq_raw[2]) == 0) return true;

    bool ok = true;
    ok &= _writeReg(AXP202_REG_IRQ_STATUS1, _irq_raw[0]);
    ok &= _writeReg(AXP202_REG_IRQ_STATUS2, _irq_raw[1]);
    ok &= _writeReg(AXP202_REG_IRQ_STATUS3, _irq_raw[2]);
    _irq_raw[0] = _irq_raw[1] = _irq_raw[2] = 0;
    return ok;
}

bool AXP202::enableIrq(uint8_t mask_en1, uint8_t mask_en2, uint8_t mask_en3) {
    bool ok = true;
    if (mask_en1) ok &= _updateReg(AXP202_REG_IRQ_EN1, mask_en1, mask_en1);
    if (mask_en2) ok &= _updateReg(AXP202_REG_IRQ_EN2, mask_en2, mask_en2);
    if (mask_en3) ok &= _updateReg(AXP202_REG_IRQ_EN3, mask_en3, mask_en3);
    return ok;
}
