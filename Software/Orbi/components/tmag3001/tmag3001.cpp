#include "tmag3001.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

TMAG3001::TMAG3001(uint8_t sda, uint8_t scl, AddrPin addr_pin) :
    _sda(sda), _scl(scl), _addr(addr_pin) {}

/****************** Config ******************/

TMAG3001::Config TMAG3001::Config::standard() {
    return Config{};
}

TMAG3001::Config TMAG3001::Config::lowPower() {
    Config c;
    c.frontend_mode  = NoiseFrontEnd::LowPower;
    c.sample_average = Averaging::Avg2;
    return c;
}

TMAG3001::Config TMAG3001::Config::highPrecision() {
    Config c;
    c.frontend_mode  = NoiseFrontEnd::LowNoise;
    c.sample_average = Averaging::Avg16;
    return c;
}

TMAG3001::~TMAG3001() {
    if (_dev != nullptr) {
        i2c_master_bus_rm_device(_dev);
        _dev = nullptr;
    }
    if (_bus != nullptr) {
        i2c_buspool_release(_bus);
        _bus = nullptr;
    }
}

bool TMAG3001::begin() {
    Config c = Config::standard();
    return begin(&c);
}

bool TMAG3001::begin(const Config *cfg) {
    if (_dev != nullptr) return true;
    _cfg = (cfg != nullptr) ? *cfg : Config{};

    // release resourses before failure returns false
    auto _freeOnFailure = [this]() {
        if (_dev != nullptr) {
            i2c_master_bus_rm_device(_dev);
            _dev = nullptr;
        }
        if (_bus != nullptr) {
            i2c_buspool_release(_bus);
            _bus = nullptr;
        }
        return false;
    };

    if (!i2c_buspool_init()) return false;
    _bus = i2c_buspool_acquire(_sda, _scl);
    if (_bus == nullptr) return false;

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = static_cast<uint8_t>(_addr);
    dev_cfg.scl_speed_hz = _cfg.scl_hz;
    dev_cfg.scl_wait_us = 0;
    dev_cfg.flags.disable_ack_check = 0;

    if (i2c_master_bus_add_device(_bus, &dev_cfg, &_dev) != ESP_OK) {
        _dev = nullptr;
        return _freeOnFailure();
    }

    if (!_checkId()) {
        return _freeOnFailure();
    }

    // 0x02 Sensor_Config_1
    uint8_t reg02 = 0;
    reg02 |= _cfg.enable_x << 4;
    reg02 |= _cfg.enable_y << 5;
    reg02 |= _cfg.enable_z << 6;
    reg02 |= (_cfg.wake_sleep_interval & 0xF);
    if (!_writeReg(TMAG3001_REG_SENSOR_CONFIG_1, reg02)) {
        return _freeOnFailure();
    }

    // 0x03 Sensor_Config_2
    uint8_t reg03 = 0;
    reg03 |= (_cfg.filter_temperature << 7);
    reg03 |= (_cfg.int_active_high << 6);       
    reg03 |= ((!_cfg.threshold_triggers_above) << 5);
    reg03 |= (_cfg.gain_correction_channel << 4);
    reg03 |= ((static_cast<uint8_t>(_cfg.angle_calculation) & 0b11) << 2);
    reg03 |= (static_cast<uint8_t>(_cfg.range_xy) << 1);
    reg03 |= (static_cast<uint8_t>(_cfg.range_z) << 0);
    if (!_writeReg(TMAG3001_REG_SENSOR_CONFIG_2, reg03)) {
        return _freeOnFailure();
    }

    // 0x04~0x06 THR_Config_1~3
    if (!_writeReg(TMAG3001_REG_THR_CONFIG_1, _cfg.threshold_1) ||
        !_writeReg(TMAG3001_REG_THR_CONFIG_2, _cfg.threshold_2) ||
        !_writeReg(TMAG3001_REG_THR_CONFIG_3, _cfg.threshold_3)) {
        return _freeOnFailure();
    }

    // 0x07 Sensor_Config_3
    uint8_t reg07 = 0;
    reg07 |= ((static_cast<uint8_t>(_cfg.wake_on_change) & 0b11) << 6);
    reg07 |= ((static_cast<uint8_t>(_cfg.threshold_mode) & 0b11) << 4);
    reg07 |= ((_cfg.angle_hysteresis & 0b11) << 2);
    reg07 |= (_cfg.enable_angle_offset << 1);
    reg07 |= (_cfg.angle_offset_negative << 0);
    if (!_writeReg(TMAG3001_REG_SENSOR_CONFIG_3, reg07)) {
        return _freeOnFailure();
    }

    // 0x08 INT_Config_1
    uint8_t reg08 = 0;
    reg08 |= (_cfg.interrupt_on_drdy << 7);
    reg08 |= (_cfg.interrupt_on_threshold << 6);
    reg08 |= (_cfg.int_pulse_mode << 5);
    reg08 |= ((static_cast<uint8_t>(_cfg.int_route) & 0b111) << 2);
    reg08 |= (_cfg.enable_int_polarity << 1);
    reg08 |= (_cfg.disable_int_pin << 0);
    if (!_writeReg(TMAG3001_REG_INT_CONFIG_1, reg08)) {
        return _freeOnFailure();
    }

    // 0x09~0x0B
    if (!_writeReg(TMAG3001_REG_SENSOR_CONFIG_4, _cfg.gain_config) ||
        !_writeReg(TMAG3001_REG_SENSOR_CONFIG_5, _cfg.offset_config_1) ||
        !_writeReg(TMAG3001_REG_SENSOR_CONFIG_6, _cfg.offset_config_2)) {
        return _freeOnFailure();
    }

    // 0x00 Device_Config_1
    uint8_t reg00 = 0;
    reg00 |= (_cfg.append_crc_on_read << 7);
    reg00 |= ((static_cast<uint8_t>(_cfg.magnet_tempco) & 0b11) << 5);
    reg00 |= ((static_cast<uint8_t>(_cfg.sample_average) & 0b111) << 2);
    if (!_writeReg(TMAG3001_REG_DEVICE_CONFIG_1, reg00)) {
        return _freeOnFailure();
    }

    // 0x01 Device_Config_2
    uint8_t reg01 = 0;
    reg01 |= ((_cfg.switch_hysteresis & 0b111) << 5);
    reg01 |= ((_cfg.frontend_mode == NoiseFrontEnd::LowNoise) << 4);
    reg01 |= ((!_cfg.enable_glitch_filter) << 3);
    reg01 |= ((_cfg.trigger_source == TriggerSource::IntPin) << 2);
    reg01 |= (static_cast<uint8_t>(_cfg.operating_mode) & 0b11);
    if (!_writeReg(TMAG3001_REG_DEVICE_CONFIG_2, reg01)) {
        return _freeOnFailure();
    }

    vTaskDelay(pdMS_TO_TICKS(3));
    return true;
}

bool TMAG3001::readRaw() {
    if (_dev == nullptr) {
        return false;
    }

    uint8_t buf[9] = {0};   // 0x10~0x18 Temp(2) + X(2) + Y(2) + Z(2) + Conv_Status(1)
    if (!_readReg(TMAG3001_REG_T_RESULT_MSB, buf, sizeof(buf))) {
        return false;
    }

    _last_raw.t = static_cast<int16_t>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]);
    _last_raw.x = static_cast<int16_t>((static_cast<uint16_t>(buf[2]) << 8) | buf[3]);
    _last_raw.y = static_cast<int16_t>((static_cast<uint16_t>(buf[4]) << 8) | buf[5]);
    _last_raw.z = static_cast<int16_t>((static_cast<uint16_t>(buf[6]) << 8) | buf[7]);
    _last_raw.conv_status = buf[8];
    return true;
}

bool TMAG3001::read() {
    if (!readRaw()) {
        return false;
    }

    const float lsb_xy = _lsbPerMT(_cfg.range_xy);
    const float lsb_z  = _lsbPerMT(_cfg.range_z);

    _last_sample.x_mT = static_cast<float>(_last_raw.x) / lsb_xy;
    _last_sample.y_mT = static_cast<float>(_last_raw.y) / lsb_xy;
    _last_sample.z_mT = static_cast<float>(_last_raw.z) / lsb_z;

    // T = 25 + (Temp_Result - 17512) / 58.2
    _temp_C = 25.0f + (static_cast<float>(_last_raw.t) - 17512.0f) / 58.2f;
    return true;
}

bool TMAG3001::_checkId() {
    uint8_t buf[3] = {0};
    if (!_readReg(TMAG3001_REG_DEVICE_ID, buf, sizeof(buf))) {
        return false;
    }
    uint16_t manufacturer = static_cast<uint16_t>((static_cast<uint16_t>(buf[2]) << 8) | buf[1]);
    return (manufacturer == 0x5449); // TI
}

float TMAG3001::_lsbPerMT(Range range) const {
    const bool large = (range == Range::Large);

    if (_cfg.chip_type == ChipType::A1) {
        if (large) return 446.0f;   // ±80 mT
        return 885.0f;              // ±40 mT
    }

    // A2：±120 mT -> 273 LSB/mT
    // ±240 mT -> 241 LSB/mT ??? TO BE CHECKED. Using 137.0 in datasheet 6.5.2.1
    // TODO: TEST THE COEFFCIENT
    if (large) return 137.0f;        // ±240 mT
    return 273.0f;                   // ±120 mT
}

bool TMAG3001::_readReg(uint8_t reg, uint8_t *data, size_t len) {
    return (i2c_master_transmit_receive(_dev, &reg, 1, data, len, 200) == ESP_OK);
}

bool TMAG3001::_writeReg(uint8_t reg, const uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return (i2c_master_transmit(_dev, buf, 2, 50) == ESP_OK);
}
