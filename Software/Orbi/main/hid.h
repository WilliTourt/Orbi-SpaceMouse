#pragma once

#include "config.h"

#include "esp_hidd.h"

/*** GATT Note *******************************************************************************************************************
 * 
 *  Basic sturcture:
 * 
 *  Profile
 *   └── Service
 *       └── Characteristic
 *               └── Descriptor
 * 
 *  Example tree:
 * 
 *  Profile
 *  │
 *  ├── [Handle: 0x0001] [UUID: 0x2800] (Service Decl) —— Generic Access (GAP)
 *  │   │   Value: 0x1800 (GAP Service "tag")
 *  │   │
 *  │   ├── [Handle: 0x0002] [UUID: 0x2803] (Characteristic Decl) —— Device Name
 *  │   │   │   Value: { Target value handle 0x0003, Property is Read, UUID is 0x2A00 }
 *  │   │   │   // five bytes: uint8_t property, uint16_t handle and uint16_t UUID
 *  │   │   │   // ↑ this is just a declaration to tell you that the handle, property, and UUID of value of this characteristic
 *  │   │   │
 *  │   │   └── [Handle: 0x0003] [UUID: 0x2A00] (Characteristic Value)
 *  │   │       Value: "Orbi" // this is true value
 *  │   │
 *  │   └── [Handle: 0x0004] [UUID: 0x2803] (Characteristic Decl) —— Appearance
 *  │       │   Value: { Target value handle 0x0005, Property is Read, UUID is 0x2A01 }
 *  │       │
 *  │       └── [Handle: 0x0005] [UUID: 0x2A01] (Characteristic Value)
 *  │           Value: 0x03C0 (HID Mouse tag)
 *  │
 *  ├── [Handle: 0x0006] [UUID: 0x2800] (Service Decl) —— Device Information (DIS)
 *  │   │   Value: 0x180A
 *  │   │
 *  │   ├── [Handle: 0x0007] [UUID: 0x2803] (Characteristic Decl) —— Manufacturer Name
 *  │   │   │   Value: { Target value handle 0x0008, Property is Read, UUID is 0x2A29 }
 *  │   │   │
 *  │   │   └── [Handle: 0x0008] [UUID: 0x2A29] (Characteristic Value)
 *  │   │       Value: "WilliTourt"
 *  │   │
 *  │   └── [Handle: 0x0009] [UUID: 0x2803] (Characteristic Decl) —— Model Number
 *  │       │   Value: { Target value handle 0x000A, Property is Read, UUID is 0x2A24 }
 *  │       │
 *  │       └── [Handle: 0x000A] [UUID: 0x2A24] (Characteristic Value)
 *  │           Value: "V1.0.0"
 *  │
 *  ├── [Handle: 0x000B] [UUID: 0x2800] (Service Decl) —— Battery Service (BAS)
 *  │   │   Value: 0x180F
 *  │   │
 *  │   └── [Handle: 0x000C] [UUID: 0x2803] (Characteristic Decl) —— Battery Level
 *  │       │   Value: { Target value handle 0x000D, Property is Read/Notify, UUID is 0x2A19 }
 *  │       │
 *  │       └── [Handle: 0x000D] [UUID: 0x2A19] (Characteristic Value)
 *  │           Value: 0-100 (uint8_t)
 *  │           │
 *  │           └── [Handle: 0x000E] [UUID: 0x2902] (Descriptor - CCCD) // Note: descriptor has no decl but a standalone attribute
 *  │               Value: [Host write 0x0001 (GATT Write Request) to allow battery notification]
 *  │
 *  └── [Handle: 0x000F] [UUID: 0x2800] (Service Decl) —— HID Service (HOGP)
 *      │   Value: 0x1812
 *      │
 *      ├── [Handle: 0x0010] [UUID: 0x2803] (Characteristic Decl) —— HID Information
 *      │   │   Value: { Target value handle 0x0011, Property is Read, UUID is 0x2A4A }
 *      │   │
 *      │   └── [Handle: 0x0011] [UUID: 0x2A4A] (Characteristic Value)
 *      │       Value: [...] // something about HID version and else
 *      │
 *      ├── [Handle: 0x0012] [UUID: 0x2803] (Characteristic Decl) —— Report Map
 *      │   │   Value: { Target value handle 0x0013, Property is Read, UUID is 0x2A4B }
 *      │   │
 *      │   └── [Handle: 0x0013] [UUID: 0x2A4B] (Characteristic Value)
 *      │       Value: [...]
 *      │       // report map is an array tells host which byte indicates what
 *      │       // (the report format definition)
 *      │
 *      ├── [Handle: 0x0014] [UUID: 0x2803] (Characteristic Decl) —— HID Control Point // host to device
 *      │   │   Value: { Target value handle 0x0015, Property is WriteNoResp, UUID is 0x2A4C }
 *      │   │
 *      │   └── [Handle: 0x0015] [UUID: 0x2A4C] (Characteristic Value)
 *      │       Value: [0x00 Suspend / 0x01 Exit Suspend]
 *      │
 *      ├── [Handle: 0x0016] [UUID: 0x2803] (Characteristic Decl) —— Protocol Mode
 *      │   │   Value: { Target value handle 0x0017, Property is Read/WriteNoResp, UUID is 0x2A4E }
 *      │   │
 *      │   └── [Handle: 0x0017] [UUID: 0x2A4E] (Characteristic Value)
 *      │       Value: [0x01 Report Protocol / 0x00 Boot Protocol]
 *      │
 *      ├── [Handle: 0x0018] [UUID: 0x2803] (Characteristic Decl) —— Input Report // this is REAL mouse data 0x2A4D
 *      │   │   Value: { Target value handle 0x0019, Property is Read/Notify, UUID is 0x2A4D }
 *      │   │
 *      │   └── [Handle: 0x0019] [UUID: 0x2A4D] (Characteristic Value)
 *      │      Value: [X/Y/Z/Rx/Ry/Rz and button byte]
 *      │      │
 *      │      ├── [Handle: 0x001A] [UUID: 0x2908] (Descriptor - Report Reference)
 *      │      │   Value: [Report ID=1, Report Type=1(Input)]
 *      │      │
 *      │      └── [Handle: 0x001B] [UUID: 0x2902] (Descriptor - CCCD)
 *      │          Value: [Host write 0x0001 to allow notification]
 *      │   
 *      └── (Output Report and else)
 * 
*********************************************************************************************************************************/

class HID {
    public:
        enum class TransType : uint8_t {
            None,
            Ble,
            Usb
        };

        // use a pointer instead of copying it
        HID(const UserConfig *cfg) : _cfg(cfg) {}

        bool begin(TransType type);
        void end(TransType type);

        bool isReady() const { return _dev != nullptr; }
        bool isConnected() const;
        TransType getTransType() const { return _trans_type; }

        bool send(const float axis[6]);
        void setButton(uint8_t bit, bool pressed);
        void setBattery(uint8_t percent); // only available for ble

    private:
        static constexpr char* DEVICE_NAME = "Orbi";
        static constexpr char* MANUFACTURER_NAME = "WilliTourt";

        bool _beginBle();
        bool _beginUsb();
        bool _endBle();
        bool _endUsb();

        const UserConfig *_cfg = nullptr;

        TransType _trans_type = TransType::None;
        esp_hidd_dev_t *_dev = nullptr;

        uint8_t _battery = 100;

        // the six axes after deadzone and speed scaling, already in HID units
        struct Report {
            int16_t dx, dy, dz;
            int16_t drx, dry, drz;
            uint8_t buttons;        // bit 7-6: reserved, bit 5-3: key2, bit 2-0: key1
            // every key: single, double, and long press from MSB to LSB
        } _report = {};
        uint8_t _report_buf[13] = {};

        static constexpr uint8_t REPORT_MAP[] = {
            0x05, 0x01,                         // Usage Page (Generic Desktop)
            0x09, 0x08,                         // Usage (Multi-axis Controller)
            0xA1, 0x01,                         // Collection (Application)

            // 6-axis
            0x85, 0x01,                         // Report ID (1)
            0x16, 0xA2, 0xFE,                   // Logical Minimum (-350)
            0x26, 0x5E, 0x01,                   // Logical Maximum (350)
            0x09, 0x30, 0x09, 0x31, 0x09, 0x32, // X Y Z
            0x09, 0x33, 0x09, 0x34, 0x09, 0x35, // Rx Ry Rz
            0x75, 0x10,                         // Report Size (16)
            0x95, 0x06,                         // Report Count (6)
            0x81, 0x02,                         // Input (Data is Var, Abs)

            // buttons
            0x05, 0x09,                         // Usage Page (Button)
            0x19, 0x01,                         // Usage Minimum (1)
            0x29, 0x08,                         // Usage Maximum (8)
            0x15, 0x00,                         // Logical Minimum (0)
            0x25, 0x01,                         // Logical Maximum (1)
            0x75, 0x01,                         // Report Size (1)
            0x95, 0x08,                         // Report Count (8), but 2 bits from MSB are resserved
            0x81, 0x02,                         // Input (Data is Var, Abs)

            0xC0                                // End Collection
        };
};
