#include "hid.h"
#include "esp_hid_gap.h"

bool HID::begin(TransType type) {

}

void HID::end(TransType type) {}

bool HID::_beginBle() {
    esp_hid_gap_init(HIDD_BLE_MODE);
    esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GENERIC, DEVICE_NAME);
    esp_hidd_dev_init(&cfg, ESP_HID_TRANSPORT_BLE, cb, &_dev);
    esp_hidd_dev_battery_set(_dev, _battery);                     // 4. 电量
    ble_store_config_init();                                      // 5. 绑定密钥存 NVS
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    nimble_port_freertos_init(_hostTask);                         // 6. 启动主机任务
}

bool HID::_beginUsb() {}

bool HID::_endBle() {}

bool HID::_endUsb() {}



bool HID::isConnected() const {}

bool HID::send(const float axis[6]) {}

void HID::setButton(uint8_t bit, bool pressed) {}

void HID::setBattery(uint8_t percent) {}
