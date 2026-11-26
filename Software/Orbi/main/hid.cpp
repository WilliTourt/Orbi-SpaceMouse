#include "hid.h"

#include "esp_hid_gap.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"

#include <cmath>

extern "C" void ble_store_config_init(void);



// HID* HID::_inst = nullptr;
// esp_hid_raw_report_map_t HID::REPORT_MAPS;

HID::HID(const UserConfig *cfg) : _cfg(cfg) {
    _inst = this;
    REPORT_MAPS = { REPORT_MAP, sizeof(REPORT_MAP) };
}

bool HID::begin(TransType type) {
    _trans_type = type;
    switch (type) {
        case TransType::Ble: return _beginBle();
        case TransType::Usb: return _beginUsb();
        default: return false;
    }
}

bool HID::end(TransType type) {
    _trans_type = TransType::None;
    switch (type) {
        case TransType::Ble: return _endBle();
        case TransType::Usb: return _endUsb();
        default: return false;
    }
}

bool HID::_beginBle() {
    esp_err_t err = ESP_OK;
    err |= esp_hid_gap_init(HIDD_BLE_MODE); // init protoclo stack
    err |= esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GENERIC, DEVICE_NAME); // set advertising packet content
    if (err != ESP_OK) return false;

    esp_hid_device_config_t cfg = {};
    cfg.device_name = DEVICE_NAME;
    cfg.manufacturer_name = MANUFACTURER_NAME;
    cfg.product_id = PRODUCT_ID;
    cfg.report_maps = &REPORT_MAPS;
    cfg.report_maps_len = 1;
    cfg.serial_number = SERIAL_NUMBER;
    cfg.vendor_id = VENDOR_ID;
    cfg.version = VERSION;
    
    err = esp_hidd_dev_init(&cfg, ESP_HID_TRANSPORT_BLE, _eventCallback, &_dev); // write GATT database, register all services
    if (err != ESP_OK) return false;
    err = esp_hidd_dev_battery_set(_dev, _battery);
    if (err != ESP_OK) return false;

    ble_store_config_init();
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr; // manage pairing data

    nimble_port_freertos_init(_hostTask); // nimble_port_run()
    return true;
}

bool HID::_beginUsb() {return false;}

bool HID::_endBle() {
    if (!_dev) return false;

    _connected = false;

    esp_hidd_dev_deinit(_dev);
    _dev = nullptr;

    // nimble_port_run() only returns after this, so stop it before tearing
    // the controller down, otherwise those calls run on a live stack
    nimble_port_stop();
    esp_hid_gap_deinit();
    return true;
}

bool HID::_endUsb() {return false;}



void HID::setButton(uint8_t bit, bool triggered) {
    if (bit > 7) return;

    if (triggered) _report.buttons |= static_cast<uint8_t>(1u << bit);
    else           _report.buttons &= static_cast<uint8_t>(~(1u << bit));
}

void HID::setPose(const float pose[6]) {
    float deadzone;
    float full_scale;
    for (int i = 0; i < 6; i++) {
        deadzone = (i < 3) ? _cfg->joystick_deadzone_mm : _cfg->joystick_deadzone_rad;
        full_scale = (i < 3) ? _cfg->joystick_full_scale_mm : _cfg->joystick_full_scale_rad;

        _report.pose[i] = _pose2Report(pose[i], deadzone, full_scale);
        if (_cfg->joystick_invert_mask & (1 << i)) _report.pose[i] = -_report.pose[i];
    }
}

void HID::setBattery(uint8_t percent) {
    if (percent > 100) percent = 100;
    if (percent == _battery) return;

    _battery = percent;
    if (_dev && _connected) esp_hidd_dev_battery_set(_dev, _battery);
}

bool HID::send() {
    if (!_dev || !_connected) {
        _report.buttons = 0;
        return false;
    }

    for (int i = 0; i < 6; i++) { // minor endian
        _report_buf[i * 2] = static_cast<uint8_t>(_report.pose[i] & 0xFF);            // LSB
        _report_buf[i * 2 + 1] = static_cast<uint8_t>((_report.pose[i] >> 8) & 0xFF); // MSB
    }
    _report_buf[12] = _report.buttons;

    // for (int i = 0; i < 6; i++) _report.pose[i] = 0; // no need to clear pose
    _report.buttons = 0; // clear button events

    return esp_hidd_dev_input_set(_dev, 0, 1, _report_buf, sizeof(_report_buf)) == ESP_OK;
}



int16_t HID::_pose2Report(float in, float deadzone, float max) {
    const float absin = fabsf(in);
    if (absin < deadzone || max <= deadzone) return 0;

    float out = HID_LOGICAL_MAXIMUM * ((absin - deadzone) / (max - deadzone));

    out = (in > 0) ? out : -out;
    if (out > HID_LOGICAL_MAXIMUM) out = HID_LOGICAL_MAXIMUM;

    return static_cast<int16_t>(out);
}

void HID::_eventCallback(void*, esp_event_base_t, int32_t id, void*) {
    switch (id) {
        case ESP_HIDD_START_EVENT:
            esp_hid_ble_gap_adv_start();
            break;
        case ESP_HIDD_CONNECT_EVENT:
            _inst->_connected = true;
            break;
        case ESP_HIDD_DISCONNECT_EVENT:
            _inst->_connected = false;
            esp_hid_ble_gap_adv_start();
            break;
        default:
            break;
    }
}

void HID::_hostTask(void*) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}
