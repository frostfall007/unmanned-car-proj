#ifndef HAL_BSP_AP3216C_H
#define HAL_BSP_AP3216C_H

#include <stdint.h>
#include "wifiiot_i2c.h"

/* AP3216C 的 7 位地址为 0x1E；Hi3861 WiFi-IoT I2C API 使用 8 位地址。 */
#define AP3216C_I2C_ADDR  0x3CU
#define AP3216C_I2C_IDX   WIFI_IOT_I2C_IDX_0
#define AP3216C_I2C_SPEED 400000U

uint32_t AP3216C_Init(void);
uint32_t AP3216C_ReadData(uint16_t *irData, uint16_t *alsData, uint16_t *psData);

#endif
