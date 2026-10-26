#pragma once

#include <cstdint>

#include "i2c_buspool.h"
#include "driver/i2c_master.h"

/****************** R/W Registers ******************/

#define TMAG3001_REG_DEVICE_CONFIG_1        0x00
#define TMAG3001_REG_DEVICE_CONFIG_2        0x01

#define TMAG3001_REG_SENSOR_CONFIG_1        0x02
#define TMAG3001_REG_SENSOR_CONFIG_2        0x03
#define TMAG3001_REG_SENSOR_CONFIG_3        0x07
#define TMAG3001_REG_SENSOR_CONFIG_4        0x09
#define TMAG3001_REG_SENSOR_CONFIG_5        0x0A
#define TMAG3001_REG_SENSOR_CONFIG_6        0x0B

#define TMAG3001_REG_THR_CONFIG_1           0x04
#define TMAG3001_REG_THR_CONFIG_2           0x05
#define TMAG3001_REG_THR_CONFIG_3           0x06

#define TMAG3001_REG_INT_CONFIG_1           0x08

#define TMAG3001_REG_I2C_ADDRESS            0x0C

/******************* R Registers *******************/

#define TMAG3001_REG_DEVICE_ID              0x0D
#define TMAG3001_REG_MANUFACTURER_ID_LSB    0x0E
#define TMAG3001_REG_MANUFACTURER_ID_MSB    0x0F

#define TMAG3001_REG_T_RESULT_MSB           0x10
#define TMAG3001_REG_T_RESULT_LSB           0x11
#define TMAG3001_REG_X_RESULT_MSB           0x12
#define TMAG3001_REG_X_RESULT_LSB           0x13
#define TMAG3001_REG_Y_RESULT_MSB           0x14
#define TMAG3001_REG_Y_RESULT_LSB           0x15
#define TMAG3001_REG_Z_RESULT_MSB           0x16
#define TMAG3001_REG_Z_RESULT_LSB           0x17
#define TMAG3001_REG_CONV_STATUS            0x18
#define TMAG3001_REG_ANGLE_RESULT_MSB       0x19
#define TMAG3001_REG_ANGLE_RESULT_LSB       0x1A
#define TMAG3001_REG_MAGNITUDE_RESULT       0x1B
#define TMAG3001_REG_DEVICE_STATUS          0x1C


class TMAG3001 {
    public:
        enum class AddrPin : uint8_t {
            toGND = 0x34,   // ADDR -> GND
            toVCC = 0x35,   // ADDR -> 3V3
            toSDA = 0x36,   // ADDR -> SDA
            toSCL = 0x37,   // ADDR -> SCL
        };

        enum class ChipType : uint8_t {
            A1 = 0,
            A2 = 1,
        };

        enum class MagnetType : uint8_t {
            Any = 0,
            NdBFe = 1,
            SmCo = 2,
            Ceramic = 3
        };

        // Measure Range
        enum class Range : uint8_t {
            Small = 0, // 40mT@A1, 120mT@A2
            Large = 1, // 80mT@A1, 240mT@A2
        };

        enum class Averaging : uint8_t {
            Avg1 = 0, Avg2 = 1, Avg4 = 2, Avg8 = 3, Avg16 = 4, Avg32 = 5
        };

        // Analog Frontend
        enum class NoiseFrontEnd : uint8_t {
            LowPower = 0,
            LowNoise = 1,
        };

        enum class OperatingMode : uint8_t {
            Standby    = 0,  // Waiting for trigger
            Sleep      = 1,
            Continuous = 2,
            WakeSleep  = 3,  // Convert once and sleep for a while, loop
        };

        // Which trigger source to be used when using standby operation mode
        enum class TriggerSource : uint8_t {
            I2cCommand = 0,
            IntPin     = 1,
        };

        // Wake IIC master when magnetic field changes. 0x07 bit7-6
        enum class WakeOnChange : uint8_t {
            Disabled    = 0,
            AngleChange = 1,
            FieldChange = 2,
        };

        // Determine the usage of threshold and gain/offset registers. 0x07 bit5-4
        enum class ThresholdMode : uint8_t {
            None          = 0,  // 0x09/0x0A/0x0B as gain_config / offset_config_1 / offset_config_2
            Angle         = 1,  // Angle threshold window. threshold_1 = A_THR_REF, thresold_2 = A_THR_BAND.
            MagneticField = 2,  // X/Y/Z have their own threshold. threshol_1/2/3 = min, gain_config/offset_config_1/2 = max.
            Magnitude     = 3,  // √(X²+Y²+Z²) as one threshold. threshold_1 = min, threshold_2 = max. Can be used as button press detection.
        };

        // Let chip itself calculate abs angle. 0x03 bit3-2
        enum class AnglePair : uint8_t {
            None = 0,
            XY   = 1,
            YZ   = 2,
        };

        // Interrupt route.
        enum class InterruptRoute : uint8_t {
            Disabled            = 0,
            IntPin              = 1,
            IntPinIgnoreI2CBusy = 2,
            SclWidth            = 3,
            SclWidthIgnoreAddr  = 4,
            UnipolarSwitch      = 5, // Disable INT and turns the sensor to a switch
            OmnipolarSwitch     = 6, // Disable INT and turns the sensor to a switch
        };

        // Sensor configuration
        struct Config {
            ChipType     chip_type = ChipType::A2;

            // 0x00 Device_Config_1
            bool         append_crc_on_read = false;
            MagnetType   magnet_tempco = MagnetType::NdBFe;
            Averaging    sample_average = Averaging::Avg4;

            // 0x01 Device_Config_2   MUST WRITE LASTLY
            uint8_t       switch_hysteresis = 0;
            NoiseFrontEnd frontend_mode = NoiseFrontEnd::LowNoise;
            bool          enable_glitch_filter = true; // notice that 1 in register means off
            TriggerSource trigger_source = TriggerSource::I2cCommand;
            OperatingMode operating_mode = OperatingMode::Continuous;

            // 0x02 Sensor_Config_1
            bool          enable_x = true;
            bool          enable_y = true;
            bool          enable_z = true;
            uint8_t       wake_sleep_interval = 0;    // 0=1ms 1=5ms ... 12=20s

            // 0x03 Sensor_Config_2
            bool          filter_temperature = false;  // T_Rate
            bool          int_active_high = false;
            bool          threshold_triggers_above = true; // notice that true corresponding to 0h reg val
            uint8_t       gain_correction_channel = 0; // used in angle mode
            AnglePair     angle_calculation = AnglePair::None;
            Range         range_xy = Range::Small;
            Range         range_z  = Range::Small;

            // 0x04~0x06
            uint8_t       threshold_1 = 0;
            uint8_t       threshold_2 = 0;
            uint8_t       threshold_3 = 0;

            // 0x07 Sensor_Config_3
            WakeOnChange  wake_on_change = WakeOnChange::Disabled;
            ThresholdMode threshold_mode = ThresholdMode::None;
            uint8_t       angle_hysteresis = 0;        // 0=1° 1=2° 2=4° 3=8°
            bool          enable_angle_offset = false;
            bool          angle_offset_negative = false;

            // 0x08 INT_Config_1
            bool           interrupt_on_drdy = false;
            bool           interrupt_on_threshold  = false;
            bool           int_pulse_mode = false;     // false=latch true=5us pulse
            InterruptRoute int_route = InterruptRoute::Disabled;
            bool           enable_int_polarity = false;
            bool           disable_int_pin = false;

            // 0x09~0x0B
            uint8_t       gain_config = 0;
            uint8_t       offset_config_1 = 0;
            uint8_t       offset_config_2 = 0;

            uint32_t      scl_hz     = 400000;
            uint16_t      timeout_ms = 50;

            // Presets
            static Config standard();       // 4x average + low noise
            static Config lowPower();       // 2x average + low power
            static Config highPrecision();  // 16x average + low power
        };

        struct Raw {
            int16_t t;
            int16_t x;
            int16_t y;
            int16_t z;
            uint8_t conv_status;   // 0x18 Conv_Status
        };

        struct Sample {
            float x_mT;
            float y_mT;
            float z_mT;
        };



        TMAG3001(uint8_t sda, uint8_t scl, AddrPin addr_pin);
        ~TMAG3001();

        bool    begin();                            // Config::standard()
        bool    begin(const Config *cfg);           // Customized
        bool    isReady() const { return _dev != nullptr; }

        bool    readRaw();
        bool    read();

        Raw     getRaw() { return _last_raw; };
        Sample  get() { return _last_sample; };
        float   getTemperature() { return _temp_C; };
        uint8_t getAddr() const { return static_cast<uint8_t>(_addr); }

    private:
        bool    _checkId();
        float   _lsbPerMT(Range range) const;

        bool    _readReg(uint8_t reg, uint8_t *data, size_t len);
        bool    _writeReg(uint8_t reg, const uint8_t data);

        int     _sda;
        int     _scl;
        AddrPin _addr;
        Config  _cfg = {};

        Raw     _last_raw = Raw{};
        Sample  _last_sample = Sample{};
        float   _temp_C = 0.0f;

        i2c_master_bus_handle_t _bus = nullptr; // i2c_master_bus_t*
        i2c_master_dev_handle_t _dev = nullptr; // i2c_master_dev_t*
};
