#pragma once

#include "driver/i2c_master.h"
#include <stdbool.h>

#if defined(__cplusplus)
extern "C" {
#endif

bool i2c_buspool_init();
i2c_master_bus_handle_t i2c_buspool_acquire(uint8_t sda, uint8_t scl);
bool i2c_buspool_release(i2c_master_bus_handle_t bus);

#if defined(__cplusplus)
}
#endif
