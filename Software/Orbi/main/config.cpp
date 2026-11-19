#include "config.h"

#include <cstring>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"



bool Config::begin() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "nvs partition needs erase (%s)", esp_err_to_name(err));
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
        _ready = false;
        return false;
    }

    _ready = true;
    if (!load()) {
        ESP_LOGW(TAG, "no stored config, using defaults");
        save();
    }
    return true;
}

bool Config::load() {
    if (!_ready) return false;

    nvs_handle_t nvs_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle) != ESP_OK) return false;

    size_t len = sizeof(_blob);
    esp_err_t err = nvs_get_blob(nvs_handle, NVS_KEY, &_blob, &len);
    nvs_close(nvs_handle);

    if (err != ESP_OK || len != sizeof(_blob) || _blob.magic != CFG_MAGIC) return false;

    return true;
}

bool Config::save() {
    if (!_ready) return false;

    nvs_handle_t nvs_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) != ESP_OK) return false;

    esp_err_t err = nvs_set_blob(nvs_handle, NVS_KEY, &_blob, sizeof(_blob));
    if (err == ESP_OK) err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "save failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}
