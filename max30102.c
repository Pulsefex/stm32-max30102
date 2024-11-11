//Leroy,MameMore,Connor,Nicole,Justin
#include <max30102.h>
#include <stdio.h>

MAX30102_Status_t MAX30102_Init(I2C_HandleTypeDef *hi2c)
{
    __uint8_t data;

    //set bit 6 to 1; this software resets the sensor
    data = 0x40; //0100000

    if (MAX30102_WriteRegister(hi2c,
                               MAX30102_REG_MODE_CONFIG,
                               data) != MAX30102_OK)
    {
        return MAX30102_ERROR;
    }

    do
    {
        if (MAX30102_ReadRegister(hi2c,
                                  MAX30102_REG_MODE_CONFIG,
                                  &data) != MAX30102_OK)
        {
            return MAX30102_ERROR;
        }
    } while (data & 0x40); // bitwise check

    data = 0x40; // enable FIFO almost full interrupt (bit 6)
    if (MAX30102_WriteRegister(hi2c,
                               MAX30102_REG_INTR_ENABLE_1,
                               data) != MAX30102_OK)
    {
        return MAX30102_ERROR;
    }

    /* 18-bit ADC resolution (SPO2_ADC_RGE = 3) */
    /* 50 samples per second (SPO2_SR = 2) */
    /* 411 μs LED pulse width (LED_PW = 3)  */
    /* Combined as 0x63 for the SPO2_CONFIG register */

    data = 0x63; //01100011
    if (MAX30102_WriteRegister(hi2c,
                               MAX30102_REG_SPO2_CONFIG,
                               data) != MAX30102_OK)
    {
        return MAX30102_ERROR;
    }

    // initial LED pulse amplitudes
    data = 0x24;
    if (MAX30102_WriteRegister(hi2c,
                               MAX30102_REG_LED1_PA,
                               data) != MAX30102_OK)
    {
        return MAX30102_ERROR;
    }
    if (MAX30102_WriteRegister(hi2c,
                               MAX30102_REG_LED2_PA,
                               data) != MAX30102_OK)
    {
        return MAX30102_ERROR;
    }

    return MAX30102_OK;
}

MAX30102_Status_t MAX30102_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
    //write a single byte to the specified register
    if (HAL_I2C_Mem_Write(hi2c,
                          MAX30102_I2C_ADDRESS,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          &value,
                          1,
                          HAL_MAX_DELAY) != HAL_OK)
    {
        return MAX30102_ERROR;
    }

    return MAX30102_OK;
}

MAX30102_Status_t MAX30102_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value)
{
    //read a single byte from the specified register
    if (HAL_I2C_Mem_Read(hi2c,
                         MAX30102_I2C_ADDRESS,
                         reg,
                         I2C_MEMADD_SIZE_8BIT,
                         value,
                         1,
                         HAL_MAX_DELAY) != HAL_OK)
    {
        return MAX30102_ERROR;
    }

    return MAX30102_OK;
}

MAX30102_Status_t MAX30102_ReadFIFO(I2C_HandleTypeDef *hi2c, uint32_t *red_data, uint32_t *ir_data) {
    uint8_t fifo_data[6]; 

    // Read 6 bytes from the FIFO data register
    if (HAL_I2C_Mem_Read(hi2c,
                         MAX30102_I2C_ADDRESS,
                         MAX30102_REG_FIFO_DATA,
                         I2C_MEMADD_SIZE_8BIT,
                         fifo_data,
                         6,
                         HAL_MAX_DELAY) != HAL_OK)
    {
        return MAX30102_ERROR;
    }
    
    // Intensity of red LED reflected back into MAX30102 photodetector
    *red_data = ((uint32_t)fifo_data[0] << 16) | ((uint32_t)fifo_data[1] << 8) | fifo_data[2];
    // Intensity of infrared LED reflected back into MAX30102 photodetector
    *ir_data = ((uint32_t)fifo_data[3] << 16) | ((uint32_t)fifo_data[4] << 8) | fifo_data[5]; 

    return MAX30102_OK;
}

MAX30102_Status_t ON(I2C_HandleTypeDef *hi2c) {
    uint8_t data;

    // Read current value of the SHDN bit
    if (MAX30102_ReadRegister(hi2c, MAX30102_REG_MODE_CONFIG, &data) != MAX30102_OK) {
        return MAX30102_ERROR;
    }

    data &= ~0x80; // Wake up sensor by clearing SHDN bit
    if (MAX30102_WriteRegister(hi2c, MAX30102_REG_MODE_CONFIG, data) != MAX30102_OK) {
        return MAX30102_ERROR;
    }



    return MAX30102_OK;
}

MAX30102_Status_t OFF(I2C_HandleTypeDef *hi2c) {
    uint8_t data;

    // Read current value of the SHDN bit
    if (MAX30102_ReadRegister(hi2c, MAX30102_REG_MODE_CONFIG, &data) != MAX30102_OK) {
        return MAX30102_ERROR;
    }

    data |= 0x80; // Put sensor to sleep using SHDN bit
    if (MAX30102_WriteRegister(hi2c, MAX30102_REG_MODE_CONFIG, data) != MAX30102_OK) {
        return MAX30102_ERROR;
    }



    return MAX30102_OK;
}
/*HOW DOES HEART RATE SENSOR WORK

Red and infrared data signals are read by the MAX30102 sensor. These values are a result of the reflected light intensity after the red and infrared LEDs hit a blood vessel
The reflection intensity will be low during a heart beat since blood volume is higher in the arteries. Higher blood volume leads to slightly darker skin around the finger thus more light
is absorbed. Since more light is absorbed, less will be reflected. Thus in graphing terms, a trough is reached when the heart beats.
The opposite is true between heart beats, blood volume is lower leading to a lighter color which absorbs less light and reflects more. In turn a peak is reached.
Calculating the time between peaks can allow an estimated heart rate to be calculated.

Infrared data is primarily used for heart rate, data sheet states red data is 'critical' for SPo2 which is necessary for temperature sensing
*/

MAX30102_Status_t GetHeartRate(I2C_HandleTypeDef *hi2c, float *heart_rate, int num_peaks) { // Number of peaks before heart rate calculation necessary on function call
    
    uint32_t red_data, ir_data; // red data and infrared data represent the light intensity values of light reflected back into the MAX30102 sensor from these LEDs hitting a persons skin
                                // the red and infrared data values changed based on pulse

    static uint32_t prev_ir_data = 0;
    static uint32_t peak_data_signal = 0;

    static uint32_t peak_interval_sum = 0;
    static uint32_t prev_peak_time = 0;
    static int peaks_reached = 0;
    
    uint32_t curr_time = 0;
    



        // Ensure ReadFIFO is functioning properly by reading red and infrared data from it.
        if (MAX30102_ReadFIFO(hi2c, &red_data, &ir_data) != MAX30102_OK) {
            return MAX30102_ERROR;
        }

        // Get current time in milliseconds, function exists in all STM32 boards
        curr_time = HAL_GetTick();

        // Check and update peak value for the max infrared value, only update when an upward trend in infrared data value occurs.
        if (ir_data > prev_ir_data && ir_data > peak_data_signal) { // Double checks to ensure peak value not updated on random highs
            peak_data_signal = ir_data;  // Update peak if current value is higher
        } 

        // Peak value is found, calculates time interval between new peak value and previous peak value
        else if (ir_data < prev_ir_data && peak_data_signal > 0) {
            if (peaks_reached > 0) {
                peak_interval_sum += curr_time - prev_peak_time;
            }

            prev_peak_time = curr_time;  
            peak_data_signal = 0;
            peaks_reached++; 
        }

        prev_ir_data = ir_data; // Assign current ir data value the previous one. 

        // Calculate heart rate in BPM after every 15 peaks. More peaks = more accurate measurements since the average heart rate is calculated with more interval values
        if (peaks_reached >= num_peaks) {
            *heart_rate = (60 * 1000) / (peak_interval_sum / peaks_reached); // Convert average interval time from milliseconds to minutes 
            peak_interval_sum = 0; 
            peaks_reached = 0;         
        }
    return MAX30102_OK; // Return if loop is working properly
}