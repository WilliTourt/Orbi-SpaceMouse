#include "axp202.h"

#include "i2c_buspool.h"

#define AXP202_I2C_ADDR   0x34
#define AXP202_CHIP_ID    0x41
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

// 12-bit ADC: high 8 bits + low 4 bits
uint16_t AXP202::_readH8L4(uint8_t reg_h8) {
    // Address auto-increments, so H8 and the following L4 come in one transaction
    uint8_t buf[2] = {0, 0};
    if (!_readReg(reg_h8, buf, sizeof(buf))) return 0;
    return static_cast<uint16_t>((static_cast<uint16_t>(buf[0]) << 4) | (buf[1] & 0x0F));
}

// 13-bit ADC: high 8 bits + low 5 bits (battery charge/discharge current)
uint16_t AXP202::_readH8L5(uint8_t reg_h8) {
    uint8_t buf[2] = {0, 0};
    if (!_readReg(reg_h8, buf, sizeof(buf))) return 0;
    return static_cast<uint16_t>((static_cast<uint16_t>(buf[0]) << 5) | (buf[1] & 0x1F));
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
    if (!_readReg(AXP202_REG_IC_TYPE, &id, 1) || id != AXP202_CHIP_ID) {
        i2c_master_bus_rm_device(_dev);
        _dev = nullptr;
        i2c_buspool_release(_bus);
        _bus = nullptr;
        return false;
    }

    // Turn on the ADC channels we read from (only battery ones are on by default)
    _updateReg(AXP202_REG_ADC_EN1,
               AXP202_ADC1_BAT_VOL | AXP202_ADC1_BAT_CUR |
               AXP202_ADC1_VBUS_VOL | AXP202_ADC1_VBUS_CUR,
               AXP202_ADC1_BAT_VOL | AXP202_ADC1_BAT_CUR |
               AXP202_ADC1_VBUS_VOL | AXP202_ADC1_VBUS_CUR);
    _updateReg(AXP202_REG_ADC_EN2, AXP202_ADC2_TEMP, AXP202_ADC2_TEMP);

    return true;
}

/* ------------------------- Status ------------------------- */

bool AXP202::isUsbIn() {
    uint8_t v = 0;
    if (!_readReg(AXP202_REG_STATUS, &v, 1)) return false;
    return (v & AXP202_STATUS_VBUS_PRESENT) != 0;
}

bool AXP202::isBatteryConnected() {
    uint8_t v = 0;
    if (!_readReg(AXP202_REG_MODE_CHGSTATUS, &v, 1)) return false;
    return (v & AXP202_MODE_BAT_PRESENT) != 0;
}

bool AXP202::isCharging() {
    uint8_t v = 0;
    if (!_readReg(AXP202_REG_MODE_CHGSTATUS, &v, 1)) return false;
    return (v & AXP202_MODE_CHARGING) != 0;
}

bool AXP202::isOverTemperature() {
    uint8_t v = 0;
    if (!_readReg(AXP202_REG_MODE_CHGSTATUS, &v, 1)) return false;
    return (v & AXP202_MODE_OVER_TEMP) != 0;
}

/* ------------------------- Measurements ------------------------- */

uint16_t AXP202::batteryMillivolts() {
    return static_cast<uint16_t>(_readH8L4(AXP202_REG_BAT_VOL_H8) * 1.1f);
}

uint8_t AXP202::batteryPercent() {
    if (!isBatteryConnected()) return 0;

    // Rough Li-ion curve: 3.0V empty ... 4.15V full, 11 breakpoints
    static const uint16_t table[11] = {
        3000, 3650, 3700, 3740, 3760, 3795,
        3840, 3910, 3980, 4070, 4150
    };
    uint16_t mv = batteryMillivolts();
    if (mv < table[0]) return 0;

    for (int i = 1; i < 11; ++i) {
        if (mv < table[i]) {
            uint16_t span = static_cast<uint16_t>(table[i] - table[i - 1]);
            return static_cast<uint8_t>(i * 10 - (10 * (table[i] - mv)) / span);
        }
    }
    return 100;
}

float AXP202::chargeCurrentmA() {
    return _readH8L5(AXP202_REG_BAT_CHGCUR_H8) * 0.5f;
}

float AXP202::dischargeCurrentmA() {
    return _readH8L5(AXP202_REG_BAT_DISCHGCUR_H8) * 0.5f;
}

uint16_t AXP202::vbusMillivolts() {
    return static_cast<uint16_t>(_readH8L4(AXP202_REG_VBUS_VOL_H8) * 1.7f);
}

float AXP202::vbusCurrentmA() {
    return _readH8L4(AXP202_REG_VBUS_CUR_H8) * 0.375f;
}

float AXP202::chipTemperature() {
    return _readH8L4(AXP202_REG_INTERNAL_TEMP_H8) * 0.1f - 144.7f;
}

/* ------------------------- Power ------------------------- */

bool AXP202::shutdown() {
    return _writeReg(AXP202_REG_SHUTDOWN, AXP202_SHUTDOWN_BIT);
}

bool AXP202::isMainRailEnabled() {
    uint8_t v = 0;
    if (!_readReg(AXP202_REG_LDO234_DC23_CTL, &v, 1)) return false;
    return (v & AXP202_PWR_DCDC3) != 0;
}

bool AXP202::powerDownUnusedRails() {
    // All these rails are unloaded on this board (only their filter caps are attached).
    // 0x12 is a shared register, so DCDC3 (bit1) must be preserved -> masked update
    const uint8_t unused = AXP202_PWR_LDO3 | AXP202_PWR_DCDC2 |
                           AXP202_PWR_LDO4 | AXP202_PWR_LDO2 | AXP202_PWR_EXTEN;
    bool ok = _updateReg(AXP202_REG_LDO234_DC23_CTL, unused, 0x00);

    // Safety net: if DCDC3 got cleared, the board is already dead -> restore it
    if (!isMainRailEnabled()) {
        _updateReg(AXP202_REG_LDO234_DC23_CTL, AXP202_PWR_DCDC3, AXP202_PWR_DCDC3);
        return false;
    }
    return ok;
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

    irq.alarmed = (s1 | s2 | s3) != 0;
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
