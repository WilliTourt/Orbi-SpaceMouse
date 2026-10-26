#include "i2c_buspool.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define ESP32S3_I2C_BUS_TOTAL_NUM 2

typedef struct I2C_BusPool {
    uint8_t sda;
    uint8_t scl;
    i2c_master_bus_handle_t bus;
    uint8_t dev_num;
} I2C_BusPool_t;

static I2C_BusPool_t pool[ESP32S3_I2C_BUS_TOTAL_NUM] = {0};
static SemaphoreHandle_t pool_mutex = NULL;

#define POOL_ENTER_CRITICAL()    if (pool_mutex) xSemaphoreTake(pool_mutex, portMAX_DELAY)
#define POOL_EXIT_CRITICAL()     if (pool_mutex) xSemaphoreGive(pool_mutex)



bool i2c_buspool_init() {
    if (pool_mutex) return true;

    pool_mutex = xSemaphoreCreateMutex();
    return pool_mutex ? true : false;
}

i2c_master_bus_handle_t i2c_buspool_acquire(uint8_t sda, uint8_t scl) {
    if (!pool_mutex) return NULL;
    if (sda == 0 || scl == 0) return NULL;

    POOL_ENTER_CRITICAL();

    for (int i = 0; i < ESP32S3_I2C_BUS_TOTAL_NUM; i++) {
        if (pool[i].sda == sda && pool[i].scl == scl) {
            pool[i].dev_num++;
            POOL_EXIT_CRITICAL();
            return pool[i].bus;
        }

        if (pool[i].bus == NULL) {
            i2c_master_bus_config_t cfg = {
                .i2c_port = i,      // Port 0 for bus 0, port 1 too.
                .sda_io_num = sda,
                .scl_io_num = scl,
                .clk_source = I2C_CLK_SRC_DEFAULT,
                .glitch_ignore_cnt = 7,
                .intr_priority = 0,
                .trans_queue_depth = 0,
                .flags = {
                    .enable_internal_pullup = 1,
                    .allow_pd = 0   // For ESP32S3 there is no I2C sleep retention, must set this to 0
                }
            };

            if (i2c_new_master_bus(&cfg, &pool[i].bus) == ESP_OK) {
                pool[i].sda = sda;
                pool[i].scl = scl;
                pool[i].dev_num = 1;
                POOL_EXIT_CRITICAL();
                return pool[i].bus;
            }
            POOL_EXIT_CRITICAL();
            return NULL;
        }
    }

    POOL_EXIT_CRITICAL();
    return NULL;
}

bool i2c_buspool_release(i2c_master_bus_handle_t bus) {
    if (!pool_mutex) return false;
    if (bus == NULL) return false;

    POOL_ENTER_CRITICAL();

    for (int i = 0; i < ESP32S3_I2C_BUS_TOTAL_NUM; i++) {
        if (pool[i].bus == bus) {
            if (pool[i].dev_num > 0) pool[i].dev_num--;
            if (pool[i].dev_num == 0) {
                i2c_del_master_bus(bus);
                pool[i].sda = 0;
                pool[i].scl = 0;
                pool[i].bus = NULL;
            }
            POOL_EXIT_CRITICAL();
            return true;
        }
    }

    POOL_EXIT_CRITICAL();
    return false;
}
