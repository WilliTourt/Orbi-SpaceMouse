#include "config.h"

#include "nvs.h"
#include "nvs_flash.h"



bool Config::begin() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) { // nvs partition needs erase
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) { // nvs init failed
        _ready = false;
        return false;
    }

    _ready = true;
    reset();
    if (!load()) return save(); // no stored config, using defaults

    return true;
}

bool Config::load() {
    if (!_ready) return false;

    nvs_handle_t nvs_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle) != ESP_OK) return false;

    Blob tmp = {}; 
    size_t len = sizeof(tmp);
    esp_err_t err = nvs_get_blob(nvs_handle, NVS_KEY, &tmp, &len);
    nvs_close(nvs_handle);

    if (err != ESP_OK || len != sizeof(tmp)) return false;
    if (tmp.magic != CFG_MAGIC || tmp.version != CFG_VERSION) return false;

    _blob = tmp; // actual load after verification
    return true;
}

bool Config::save() {
    if (!_ready) return false;

    nvs_handle_t nvs_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) != ESP_OK) return false;

    esp_err_t err = nvs_set_blob(nvs_handle, NVS_KEY, &_blob, sizeof(_blob));
    if (err == ESP_OK) err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (err != ESP_OK) return false; // save failed
    return true;
}
