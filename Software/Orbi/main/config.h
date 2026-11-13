#pragma once



#define ORBI_VERSION "1.0.0"



/****************** Physical Parameters ******************/

// NOTE: The origin is on the center axis of PCB, on sensor hall plane

#define X 0
#define Y 1
#define Z 2

constexpr float CFG_MAGNET_RADIUS = 3.0f; // mm
constexpr float CFG_MAGNET_LENGTH = 6.0f; // mm
constexpr float CFG_MAGNET_BR_DEFAULT = 1450.0; // mT. Standard NdFeB N52 magnet.

constexpr float CFG_U1_HALL_COORD[3] = { -14.289f, 8.25f, 0.0f }; // mm
constexpr float CFG_U2_HALL_COORD[3] = {  14.289f, 8.25f, 0.0f }; // mm
constexpr float CFG_U3_HALL_COORD[3] = {  0.0f   ,-16.5f, 0.0f }; // mm
constexpr const float* const CFG_SENSOR_COORDS[3] = {
    CFG_U1_HALL_COORD, CFG_U2_HALL_COORD, CFG_U3_HALL_COORD
};

constexpr float CFG_JOYSTICK_PIVOT_COORD[3] = { 0.0f, 0.0f, 21.85f };

constexpr float CFG_HALL_TO_MAGNET_BOTTOM_Z = 5.85f; // mm

/****** xxx ******/