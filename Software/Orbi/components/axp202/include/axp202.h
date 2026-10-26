#pragma once

#include <cstdint>

#include "driver/i2c_master.h"

/******************* Register addresses *******************/

#define AXP202_REG_STATUS            0x00   // Power input status
#define AXP202_REG_MODE_CHGSTATUS    0x01   // Working mode / charge status
#define AXP202_REG_IC_TYPE           0x03   // Chip ID, AXP202 reads 0x41

#define AXP202_REG_LDO234_DC23_CTL   0x12   // LDO2/3/4 + DCDC2/3 + EXTEN enable
#define AXP202_REG_SHUTDOWN          0x32   // Write 1 to bit7 to power off

#define AXP202_REG_IRQ_EN1           0x40
#define AXP202_REG_IRQ_EN2           0x41
#define AXP202_REG_IRQ_EN3           0x42
#define AXP202_REG_IRQ_STATUS1       0x48
#define AXP202_REG_IRQ_STATUS2       0x49
#define AXP202_REG_IRQ_STATUS3       0x4A

#define AXP202_REG_VBUS_VOL_H8       0x5A
#define AXP202_REG_VBUS_VOL_L4       0x5B
#define AXP202_REG_VBUS_CUR_H8       0x5C
#define AXP202_REG_VBUS_CUR_L4       0x5D

#define AXP202_REG_INTERNAL_TEMP_H8  0x5E
#define AXP202_REG_INTERNAL_TEMP_L4  0x5F

#define AXP202_REG_BAT_VOL_H8        0x78
#define AXP202_REG_BAT_VOL_L4        0x79
#define AXP202_REG_BAT_CHGCUR_H8     0x7A
#define AXP202_REG_BAT_CHGCUR_L4     0x7B
#define AXP202_REG_BAT_DISCHGCUR_H8  0x7C
#define AXP202_REG_BAT_DISCHGCUR_L4  0x7D

#define AXP202_REG_ADC_EN1           0x82
#define AXP202_REG_ADC_EN2           0x83

/******************* Bit masks (For _updateReg()) *******************/

// 0x00 AXP202_REG_STATUS
#define AXP202_STATUS_VBUS_PRESENT   0x20   // bit5: VBUS present
#define AXP202_STATUS_BAT_CHARGING   0x04   // bit2: battery current direction

// 0x01 AXP202_REG_MODE_CHGSTATUS
#define AXP202_MODE_OVER_TEMP        0x80   // bit7: over temperature
#define AXP202_MODE_CHARGING         0x40   // bit6: charging in progress
#define AXP202_MODE_BAT_PRESENT      0x20   // bit5: battery connected

// 0x12 AXP202_REG_LDO234_DC23_CTL
#define AXP202_PWR_LDO3              0x40   // bit6
#define AXP202_PWR_DCDC2             0x10   // bit4
#define AXP202_PWR_LDO4              0x08   // bit3
#define AXP202_PWR_LDO2              0x04   // bit2
#define AXP202_PWR_DCDC3             0x02   // bit1: 3V3 main rail, never turn this off
#define AXP202_PWR_EXTEN             0x01   // bit0

// 0x32 AXP202_REG_SHUTDOWN
#define AXP202_SHUTDOWN_BIT          0x80   // bit7: write 1 to power off

// 0x82 AXP202_REG_ADC_EN1
#define AXP202_ADC1_BAT_VOL          0x80   // bit7: battery voltage
#define AXP202_ADC1_BAT_CUR          0x40   // bit6: battery current
#define AXP202_ADC1_VBUS_VOL         0x08   // bit3: VBUS voltage
#define AXP202_ADC1_VBUS_CUR         0x04   // bit2: VBUS current

// 0x83 AXP202_REG_ADC_EN2
#define AXP202_ADC2_TEMP             0x80   // bit7: die temperature

// 0x48 AXP202_REG_IRQ_STATUS1 / 0x40 AXP202_REG_IRQ_EN1 (same bit position)
#define AXP202_IRQ1_VBUS_REMOVE      0x04   // bit2
#define AXP202_IRQ1_VBUS_INSERT      0x08   // bit3

// 0x49 AXP202_REG_IRQ_STATUS2 / 0x41 AXP202_REG_IRQ_EN2
#define AXP202_IRQ2_CHG_DONE         0x04   // bit2
#define AXP202_IRQ2_CHG_START        0x08   // bit3
#define AXP202_IRQ2_BAT_REMOVE       0x40   // bit6
#define AXP202_IRQ2_BAT_INSERT       0x80   // bit7

// 0x4A AXP202_REG_IRQ_STATUS3 / 0x42 AXP202_REG_IRQ_EN3
#define AXP202_IRQ3_PEKEY_LONG       0x01   // bit0
#define AXP202_IRQ3_PEKEY_SHORT      0x02   // bit1
#define AXP202_IRQ3_CHIP_OVER_TEMP   0x80   // bit7



class AXP202 {
    public:

        // Interrupt captured from REG48H~4AH
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
            bool alarmed; // true if any bit is set
        };

        AXP202(uint8_t sda, uint8_t scl);
        ~AXP202();

        bool begin();
        bool isReady() const { return _dev != nullptr; }

        // Status

        bool isUsbIn();
        bool isBatteryConnected();
        bool isCharging();
        bool isOverTemperature();

        // Measurements

        uint16_t batteryMillivolts();     // mV
        uint8_t  batteryPercent();        // 0~100
        float    chargeCurrentmA();       // mA
        float    dischargeCurrentmA();    // mA
        uint16_t vbusMillivolts();        // mV
        float    vbusCurrentmA();         // mA
        float    chipTemperature();       // degC

        // Power

        bool shutdown();                  // write 0x32 bit7
        bool isMainRailEnabled();         // DCDC3 (3V3) on or off
        bool powerDownUnusedRails();      // disable unloaded rails, self-checks DCDC3

        // Interrupts/Events
        
        bool readIrq(Irq& irq); // Read all 3 status registers; the IRQ pin stays low until cleared
        bool clearIrq(); // Write 1 to clear
        bool enableIrq(uint8_t mask_en1, uint8_t mask_en2, uint8_t mask_en3); // Enable irqs: masks map to 0x40 / 0x41 / 0x42

    private:
        bool _readReg(uint8_t reg, uint8_t* data, size_t len);
        bool _writeReg(uint8_t reg, uint8_t data);
        bool _updateReg(uint8_t reg, uint8_t mask, uint8_t value);  // read-modify-write
        uint16_t _readH8L4(uint8_t reg_h8);
        uint16_t _readH8L5(uint8_t reg_h8);

        uint8_t _sda;
        uint8_t _scl;
        i2c_master_bus_handle_t _bus = nullptr;
        i2c_master_dev_handle_t _dev = nullptr;

        uint8_t _irq_raw[3] = {0, 0, 0};   // kept for clearIrq() to write back
};
