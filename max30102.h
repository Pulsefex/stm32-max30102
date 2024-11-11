/**
 *
 * This MAX30102 library was written by
 *
 * Leroy Musa (219198761)
 * Mame Mor Mbacke (218666206)
 * Justin Chiu (219589076)
 * Nicole Xiong (219574334)
 * Connor Chan (218993089)
 *
 * https://github.com/pulsefex/max30102
 */

#ifndef __MAX30102_H__
#define __MAX30102_H__


#if defined(STM32WB)
#include "stm32wbxx_hal.h"
#include "stm32wbxx_hal_i2c.h"
#else
#endif

#ifndef MAX30102_I2C_ADDR
#define MAX30102_I2C_ADDRESS   (0x57 << 1)  // 7-bit I2C address for MAX30102 sensor. 
                                            // Left shift by 1 to set up for read/write bit as per I2C protocol:
                                            // 0xAE for write (0 as LSB), 0xAF for read (1 as LSB).
#endif

#define MAX30102_REG_INTR_STATUS_1   0x00
#define MAX30102_REG_INTR_STATUS_2   0x01
#define MAX30102_REG_INTR_ENABLE_1   0x02
#define MAX30102_REG_INTR_ENABLE_2   0x03
#define MAX30102_REG_FIFO_WR_PTR     0x04
#define MAX30102_REG_OVF_COUNTER     0x05
#define MAX30102_REG_FIFO_RD_PTR     0x06
#define MAX30102_REG_FIFO_DATA       0x07
#define MAX30102_REG_MODE_CONFIG     0x09
#define MAX30102_REG_SPO2_CONFIG     0x0A
#define MAX30102_REG_LED1_PA         0x0C  // Red LED
#define MAX30102_REG_LED2_PA         0x0D  // IR LED
#define MAX30102_ADDR_WRITE          0xAE
#define MAX30102_ADDR_READ           0xAF

typedef enum {
    MAX30102_OK = 0,
    MAX30102_ERROR = 1
} MAX30102_Status_t;

MAX30102_Status_t MAX30102_Init(I2C_HandleTypeDef *hi2c);
MAX30102_Status_t MAX30102_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value);
MAX30102_Status_t MAX30102_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value);
MAX30102_Status_t MAX30102_ReadFIFO(I2C_HandleTypeDef *hi2c, uint32_t *red_data, uint32_t *ir_data);
MAX30102_Status_t GetHeartRate(I2C_HandleTypeDef *hi2c, float *heart_rate, int num_peaks);

#endif  // __MAX30102_H__