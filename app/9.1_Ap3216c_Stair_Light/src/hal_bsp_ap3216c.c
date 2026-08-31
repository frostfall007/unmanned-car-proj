#include "hal_bsp_ap3216c.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_i2c.h"
#include "wifiiot_i2c_ex.h"

#define AP3216C_SYSTEM_REG 0x00U
#define AP3216C_IR_L_REG   0x0AU
#define AP3216C_IR_H_REG   0x0BU
#define AP3216C_ALS_L_REG  0x0CU
#define AP3216C_ALS_H_REG  0x0DU
#define AP3216C_PS_L_REG   0x0EU
#define AP3216C_PS_H_REG   0x0FU

static uint32_t AP3216C_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    WifiIotI2cData data = {0};

    data.sendBuf = buffer;
    data.sendLen = sizeof(buffer);
    return I2cWrite(AP3216C_I2C_IDX, AP3216C_I2C_ADDR, &data);
}

static uint32_t AP3216C_ReadRegister(uint8_t reg, uint8_t *value)
{
    WifiIotI2cData data = {0};
    uint32_t result;

    data.sendBuf = &reg;
    data.sendLen = 1U;
    result = I2cWrite(AP3216C_I2C_IDX, AP3216C_I2C_ADDR, &data);
    if (result != 0U) {
        return result;
    }

    data.receiveBuf = value;
    data.receiveLen = 1U;
    return I2cRead(AP3216C_I2C_IDX, AP3216C_I2C_ADDR, &data);
}

uint32_t AP3216C_Init(void)
{
    uint32_t result;

    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_9, WIFI_IOT_IO_FUNC_GPIO_9_I2C0_SCL);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_10, WIFI_IOT_IO_FUNC_GPIO_10_I2C0_SDA);

    result = I2cInit(AP3216C_I2C_IDX, AP3216C_I2C_SPEED);
    if (result != 0U) {
        return result;
    }
    result = I2cSetBaudrate(AP3216C_I2C_IDX, AP3216C_I2C_SPEED);
    if (result != 0U) {
        return result;
    }

    result = AP3216C_WriteRegister(AP3216C_SYSTEM_REG, 0x04U);
    if (result != 0U) {
        return result;
    }
    osDelay(1U);
    return AP3216C_WriteRegister(AP3216C_SYSTEM_REG, 0x03U);
}

uint32_t AP3216C_ReadData(uint16_t *irData, uint16_t *alsData, uint16_t *psData)
{
    uint8_t low;
    uint8_t high;
    uint32_t result;

    result = AP3216C_ReadRegister(AP3216C_IR_L_REG, &low);
    if (result != 0U) return result;
    result = AP3216C_ReadRegister(AP3216C_IR_H_REG, &high);
    if (result != 0U) return result;
    *irData = ((low & 0x80U) != 0U) ? 0U : (((uint16_t)high << 2) | (low & 0x03U));

    result = AP3216C_ReadRegister(AP3216C_ALS_L_REG, &low);
    if (result != 0U) return result;
    result = AP3216C_ReadRegister(AP3216C_ALS_H_REG, &high);
    if (result != 0U) return result;
    *alsData = ((uint16_t)high << 8) | low;

    result = AP3216C_ReadRegister(AP3216C_PS_L_REG, &low);
    if (result != 0U) return result;
    result = AP3216C_ReadRegister(AP3216C_PS_H_REG, &high);
    if (result != 0U) return result;
    *psData = ((low & 0x40U) != 0U) ? 0U : (((uint16_t)(high & 0x3FU) << 4) | (low & 0x0FU));

    return 0U;
}
