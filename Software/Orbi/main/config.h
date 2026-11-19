#pragma once

#include <cstdint>



#define ORBI_VERSION "1.0.0"



/*** Physical Parameters **********************************************/

// NOTE: The origin is on the center axis of PCB, on sensor hall plane

#define X 0
#define Y 1
#define Z 2

constexpr float CFG_MAGNET_RADIUS = 3.0f; // mm
constexpr float CFG_MAGNET_LENGTH = 6.0f; // mm
constexpr float CFG_MAGNET_BR_DEFAULT = 1450.0f; // mT. Standard NdFeB N52 magnet.

constexpr float CFG_U1_HALL_COORD[3] = { -14.289f, 8.25f, 0.0f }; // mm
constexpr float CFG_U2_HALL_COORD[3] = {  14.289f, 8.25f, 0.0f }; // mm
constexpr float CFG_U3_HALL_COORD[3] = {  0.0f   ,-16.5f, 0.0f }; // mm
constexpr const float* const CFG_SENSOR_COORDS[3] = {
    CFG_U1_HALL_COORD, CFG_U2_HALL_COORD, CFG_U3_HALL_COORD
};

constexpr float CFG_JOYSTICK_PIVOT_COORD[3] = { 0.0f, 0.0f, 21.85f };

constexpr float CFG_HALL_TO_MAGNET_BOTTOM_Z = 5.85f; // mm

/**********************************************************************/

/*** IO Pins **********************************************************/

constexpr uint8_t CFG_PIN_STATUS_LED = 10;   // on board 0603 led
constexpr uint8_t CFG_PIN_WS2812     = 45;   // RGB strip DI
constexpr uint8_t CFG_WS2812_COUNT   = 10;

constexpr uint8_t CFG_IIC_A_SDA = 6;         // sensor bus A: U1, U2
constexpr uint8_t CFG_IIC_A_SCL = 7;
constexpr uint8_t CFG_IIC_B_SDA = 13;        // sensor bus B: U3 + PMU
constexpr uint8_t CFG_IIC_B_SCL = 12;

constexpr uint8_t CFG_PIN_KEY1  = 39;
constexpr uint8_t CFG_PIN_KEY2  = 8;

/**********************************************************************/

/*** Runtime Config ***************************************************/

// Everything below is what the PC-side app is allowed to change at runtime.
struct UserConfig {
    // joystick
    float deadzone_mm   = 0.05f;   // per axis displacement deadzone
    float deadzone_rad  = 0.0008f; // per axis rotation deadzone
    float speed_mm_s    = 60.0f;   // full travel of a 1 mm push (mm/s)
    float speed_rad_s   = 2.0f;    // 1 rad tilt (rad/s)
    uint8_t invert_mask = 0;       // bit0..2 = xyz, bit3..5 = rx,ry,rz

    // RGB LED
    uint8_t led_brightness = 30;   // percent
    uint32_t led_color_plain = 0x000000; // 0xRRGGBB
    bool    led_enabled    = true;

    // HID
    uint8_t transport = 0;         // 0 = auto, 1 = BLE only, 2 = USB only

    // calibration (written back by Joystick::calibrate)
    uint8_t calibrated = 0;
    float   br[3] = { 0.0f, 0.0f, 0.0f };
    float   pose_zero[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
};

/**********************************************************************/



class Config {
    public:
        // mounts NVS and loads whatever is stored
        bool begin();

        bool load();
        bool save();
        void reset() { _blob.cfg = UserConfig(); }

        UserConfig&       get()       { return _blob.cfg; }
        const UserConfig& get() const { return _blob.cfg; }

    private:
        // blob = magic + version + UserConfig
        struct Blob {
            uint16_t   magic;
            uint16_t   version;
            UserConfig cfg;
        } _blob;

        static constexpr char *TAG = "config";

        static constexpr char *NVS_NAMESPACE = "orbi";
        static constexpr char *NVS_KEY       = "user_cfg";
        static constexpr uint16_t CFG_MAGIC  = 0x0B01;

        bool _ready = false;
};
