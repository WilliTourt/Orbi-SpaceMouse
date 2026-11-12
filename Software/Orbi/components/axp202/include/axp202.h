#pragma once

#include <cstdint>

#include "driver/i2c_master.h"

#define AXP202_I2C_ADDR    0x34
#define AXP202_CHIP_ID     0x41  // genuine AXP202
#define AXP202_CHIP_ID_ALT 0x21  // seen on my chip (AXP202 compatible?)

/******************* Register addresses *******************/

#define AXP202_REG_STATUS            0x00   // Power input status
#define AXP202_REG_MODE_CHGSTATUS    0x01   // Working mode / charge status
#define AXP202_REG_IC_TYPE           0x03   // Chip ID, AXP202 reads 0x41
#define AXP202_REG_LDO234_DC23_CTL   0x12   // LDO2/3/4 + DCDC2/3 + EXTEN enable
#define AXP202_REG_VBUS_IPSOUT       0x30   // VHOLD limit + VBUS input current limit
#define AXP202_REG_VOFF              0x31   // power-off voltage + sleep wakeup bit
#define AXP202_REG_SHUTDOWN          0x32   // shutdown + battery detect + CHGLED
#define AXP202_REG_CHARGE_CTL1       0x33   // charge enable / target / current
#define AXP202_REG_IRQ_EN1           0x40
#define AXP202_REG_IRQ_EN2           0x41
#define AXP202_REG_IRQ_EN3           0x42
#define AXP202_REG_IRQ_STATUS1       0x48
#define AXP202_REG_IRQ_STATUS2       0x49
#define AXP202_REG_IRQ_STATUS3       0x4A
#define AXP202_REG_VBUS_ADC          0x5A   // 0x5A..0x5F: VBUS V/I, die temperature
#define AXP202_REG_BAT_ADC           0x78   // 0x78..0x7D: battery voltage and currents
#define AXP202_REG_ADC_EN1           0x82
#define AXP202_REG_ADC_EN2           0x83
#define AXP202_REG_ADC_RATE          0x84   // bit7-6: sample rate 25/50/100/200 Hz
#define AXP202_REG_COULOMB_CHG       0xB0   // 0xB0..0xB3: charge counter (32 bit)
#define AXP202_REG_COULOMB_DIS       0xB4   // 0xB4..0xB7: discharge counter (32 bit)
#define AXP202_REG_COULOMB_CTL       0xB8   // enable / suspend / clear / unlock
#define AXP202_REG_FUEL_GAUGE        0xB9   // bit6-0: hardware percentage

/******************* Bit masks *******************/

// 0x00 STATUS
#define AXP202_STATUS_VBUS_PRESENT   0x20   // bit5: VBUS present

// 0x01 MODE_CHGSTATUS
#define AXP202_MODE_OVER_TEMP        0x80   // bit7: over temperature
#define AXP202_MODE_CHARGING         0x40   // bit6: charging in progress
#define AXP202_MODE_BAT_PRESENT      0x20   // bit5: battery connected

// 0x30 VBUS_IPSOUT
#define AXP202_VBUS_ILIM_MASK        0x03   // bit1-0: 0=900mA 1=500mA 2=100mA 3=unlimited
#define AXP202_VHOLD_EN              0x40   // bit6: 1 = enable the input voltage limit
#define AXP202_VHOLD_MASK            0x38   // bit5-3: VHOLD = 4.0 + n * 0.1 V

// 0x31 VOFF
#define AXP202_VOFF_MASK             0x07   // bit2-0: VOFF = 2.6 + n * 0.1 V

// 0x32 CHGLED
#define AXP202_CHGLED_CTL_BY_REG     0x08   // bit3: 1 = register controls the LED
#define AXP202_CHGLED_MODE_MASK      0x30   // bit5-4: 0=HiZ 1=25%/1Hz 2=25%/4Hz 3=on
#define AXP202_SHUTDOWN_BIT          0x80   // bit7: write 1 to power off

// 0x33 CHARGE_CTL1
#define AXP202_CHG_EN                0x80   // bit7: 1 = charging enabled
#define AXP202_CHG_TARGET_MASK       0x60   // bit6-5: target voltage
#define AXP202_CHG_CURRENT_MASK      0x0F   // bit3-0: I = 300 + n * 100 mA
#define AXP202_CHG_CURRENT_MIN       300
#define AXP202_CHG_CURRENT_MAX       1800
#define AXP202_CHG_CURRENT_STEP      100

// 0x82 ADC_EN1 / 0x83 ADC_EN2
#define AXP202_ADC1_BAT_VOL          0x80   // bit7: battery voltage
#define AXP202_ADC1_BAT_CUR          0x40   // bit6: battery current
#define AXP202_ADC1_VBUS_VOL         0x08   // bit3: VBUS voltage
#define AXP202_ADC1_VBUS_CUR         0x04   // bit2: VBUS current
#define AXP202_ADC2_TEMP             0x80   // bit7: die temperature

// 0xB8 COULOMB_CTL
#define AXP202_COULOMB_EN            0x80   // bit7: counter enable
#define AXP202_COULOMB_SUSPEND       0x40   // bit6: write 1 to suspend
#define AXP202_COULOMB_CLEAR         0x20   // bit5: write 1 to clear
#define AXP202_COULOMB_UNLOCK        0x02   // bit1: start reading the counters
#define AXP202_COULOMB_UNLOCKED      0x01   // bit0: 1 = counters readable

// 0xB9 FUEL_GAUGE
#define AXP202_FUELGAUGE_SUSPEND     0x80   // bit7: 1 = suspend fuel gauge
#define AXP202_FUELGAUGE_PERCENT     0x7F   // bit6-0: percentage

// 0x48 status / 0x40 enable (same bit position)
#define AXP202_IRQ1_VBUS_REMOVE      0x04   // bit2
#define AXP202_IRQ1_VBUS_INSERT      0x08   // bit3
#define AXP202_IRQ2_CHG_DONE         0x04   // bit2
#define AXP202_IRQ2_CHG_START        0x08   // bit3
#define AXP202_IRQ2_BAT_REMOVE       0x40   // bit6
#define AXP202_IRQ2_BAT_INSERT       0x80   // bit7
#define AXP202_IRQ3_PEKEY_LONG       0x01   // bit0
#define AXP202_IRQ3_PEKEY_SHORT      0x02   // bit1
#define AXP202_IRQ3_CHIP_OVER_TEMP   0x80   // bit7

// AXP202 PMU, Orbi bottom board U5, 7-bit address 0x34
//
// Board facts (netlist Netlist_Schematic_BottomBrd._2026-10-26.tel):
//   - 3V3 is the DCDC3 output, turning it off kills the whole system
//   - AXP_IRQ goes to MCU IO11 (external 51K pull-up, active low)
//   - LDO2/3/4, DCDC2 and EXTEN are unloaded, only their filter caps are attached
//   - PWRON has no key attached, so the PEKey interrupts never fire here
//
// Usage: call update() first, then read the cached getters.
class AXP202 {
    public:
        // Power rails behind register 0x12, the value is the enable bit
        enum class PwrRail : uint8_t {
            LDO2  = 0x04,
            LDO3  = 0x40,
            LDO4  = 0x08,
            DCDC2 = 0x10,
            DCDC3 = 0x02,   // 3V3 main rail, never power it down
            EXTEN = 0x01
        };

        // Charge target voltage, 0x33 bit6-5
        enum class ChargeTarget : uint8_t {
            V4_10 = 0,
            V4_15 = 1,
            V4_20 = 2,
            V4_36 = 3,   // DO NOT use for a lipo
        };

        // VBUS input current limit, 0x30 bit1-0
        enum class VbusCurrentLimit : uint8_t {
            mA900     = 0,
            mA500     = 1,
            mA100     = 2,
            Unlimited = 3
        };

        // Input voltage hold threshold, 0x30 bit5-3, 4.0V..4.7V in 0.1V steps
        enum class Vhold : uint8_t {
            V4_0 = 0, V4_1, V4_2, V4_3, V4_4, V4_5, V4_6, V4_7,
            Off  = 8   // clears the enable bit, no input voltage limit
        };

        // Power-off threshold, 0x31 bit2-0, 2.6V..3.3V in 0.1V steps
        enum class Voff : uint8_t {
            V2_6 = 0, V2_7, V2_8, V2_9, V3_0, V3_1, V3_2, V3_3
        };

        // Charge LED behaviour, 0x32 bit5-4, takes over control from the charger
        enum class ChargeLedMode : uint8_t {
            HighZ    = 0,   // off / high impedance
            Blink1Hz = 1,   // 25% duty, 1 Hz
            Blink4Hz = 2,   // 25% duty, 4 Hz
            AlwaysOn = 3
        };

        // Interrupt state captured from REG48H~4AH in one shot
        struct Irq {
            bool vbus_insert;
            bool vbus_remove;
            bool bat_insert;
            bool bat_remove;
            bool chg_start;
            bool chg_done;
            bool pekey_short;
            bool pekey_long;
            bool chip_over_temp;
            bool any;          // true if any bit is set
        };

        // Everything update() samples
        struct Measurement {
            bool     vbus_present;
            bool     bat_present;
            bool     charging;
            bool     over_temp;
            uint16_t battery_mV;
            uint8_t  battery_percent;
            float    charge_mA;
            float    discharge_mA;
            uint16_t vbus_mV;
            float    vbus_mA;
            float    chip_temp_C;
        };

        AXP202(uint8_t sda, uint8_t scl);
        ~AXP202();

        // Acquire bus, add device, verify chip ID, set the charge profile
        bool begin();

        bool isReady() const { return _dev != nullptr; }

        // Sample status + all ADCs (3 I2C transactions) and refresh the cache
        bool update();

        // Getters, valid after update()
        bool     isUsbConnected() const      { return _meas.vbus_present; }
        bool     isBatteryConnected() const  { return _meas.bat_present; }
        bool     isCharging() const          { return _meas.charging; }
        bool     isOverTemperature() const   { return _meas.over_temp; }

        uint16_t getBatteryMillivolts() const   { return _meas.battery_mV; }
        uint8_t  getBatteryPercent() const      { return _meas.battery_percent; }
        float    getChargeCurrentMa() const     { return _meas.charge_mA; }
        float    getDischargeCurrentMa() const  { return _meas.discharge_mA; }
        uint16_t getVbusMillivolts() const      { return _meas.vbus_mV; }
        float    getVbusCurrentMa() const       { return _meas.vbus_mA; }
        float    getChipTemperature() const     { return _meas.chip_temp_C; }

        const Measurement& getMeasurement() const { return _meas; }

        // Power rails
        bool isRailEnabled(PwrRail rail);
        bool powerUp(PwrRail rail);
        bool powerDown(PwrRail rail);   // refuses DCDC3, clearing it powers the board off

        // Write 0x32 bit7, real power off
        bool shutdown();

        // Charging
        bool setChargeTarget(ChargeTarget target);
        bool setChargeCurrent(uint16_t milliamps);   // 300..1800, 100 mA steps
        bool setChargeEnabled(bool on);
        bool setVbusCurrentLimit(VbusCurrentLimit limit);
        bool setVhold(Vhold level);
        bool setVoff(Voff level);
        bool setChargeLedMode(ChargeLedMode mode);

        // Coulomb counter
        bool enableCoulomb(bool on);
        bool clearCoulomb();   // write 1 to the clear bit, counters restart from 0
        bool readCoulomb(uint32_t& charge, uint32_t& discharge);   // unlock + read
        float getBatteryMah(uint32_t charge, uint32_t discharge) const;
        uint8_t getFuelGaugePercent();

        // Interrupts
        bool readIrq(Irq& irq);        // IRQ pin stays low until cleared
        bool clearIrq();               // write back the bits just read
        bool enableIrq(uint8_t mask_en1, uint8_t mask_en2, uint8_t mask_en3);

    private:
        bool _readReg(uint8_t reg, uint8_t* data, size_t len);
        bool _writeReg(uint8_t reg, uint8_t data);
        bool _updateReg(uint8_t reg, uint8_t mask, uint8_t value);   // read-modify-write
        uint8_t _percentFromMillivolts(uint16_t mv) const;

        uint8_t _sda;
        uint8_t _scl;
        i2c_master_bus_handle_t _bus = nullptr;
        i2c_master_dev_handle_t _dev = nullptr;

        Measurement _meas = {};
        uint16_t _adc_rate_hz = 25;        // from 0x84[7:6], needed by the coulomb formula
        uint8_t _irq_raw[3] = {0, 0, 0};   // kept for clearIrq() to write back
};
