/*
 * max30102.h
 *
 *  Created on: Nov 13, 2024
 *      Author: Leroy, Mame Mor, Connor, Nicole, Justin
 */

#ifndef MAX30102_MAX30102_H_
#define MAX30102_MAX30102_H_

#include "stm32wbxx_hal.h"
#include "stm32wbxx_hal_i2c.h"
#include <stdint.h>

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
#define FILTER_LEVEL                 8
#define BUFFER_SIZE                  50

typedef enum {
    MAX30102_OK = 0,
    MAX30102_ERROR = 1
} MAX30102_Status_t;


typedef struct {
    uint32_t red;
    uint32_t iRed;
} SAMPLE;


// Function prototypes
MAX30102_Status_t MAX30102_Init(I2C_HandleTypeDef *hi2c);
MAX30102_Status_t MAX30102_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value);
MAX30102_Status_t MAX30102_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value);
MAX30102_Status_t MAX30102_ReadFIFO(I2C_HandleTypeDef *hi2c, SAMPLE *samples, uint8_t numSamples);
MAX30102_Status_t ON(I2C_HandleTypeDef *hi2c);
MAX30102_Status_t OFF(I2C_HandleTypeDef *hi2c);
MAX30102_Status_t MAX30102_FilterSample(SAMPLE *sample);
MAX30102_Status_t MAX30102_CalculateAcDc(uint16_t *redAC, uint32_t *redDC, uint16_t *irAC, uint32_t *irDC);
uint8_t MAX30102_GetUnreadSampleCount(I2C_HandleTypeDef *hi2c);
MAX30102_Status_t MAX30102_CalculateReadings(I2C_HandleTypeDef *hi2c, float *heartRate, uint8_t *spo2);MAX30102_Status_t MAX30102_GetStatus(I2C_HandleTypeDef *hi2c, uint8_t *status);
uint8_t MAX30102_GetHeartRate(void);
uint8_t MAX30102_GetSpO2(void);
int16_t MAX30102_GetDiff(void);

#endif /* MAX30102_MAX30102_H_ */