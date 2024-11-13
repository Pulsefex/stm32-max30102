/*
 * max30102.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Leroy, Mame Mor, Connor, Nicole, Justin
 */

#include <max30102.h>
#include <stdio.h>

//Global variables for MAX30102 calculations
SAMPLE sampleBuff[BUFFER_SIZE];    //Buffer to store samples for processing
uint8_t heartRate = 0;             //Calculated heart rate value
uint8_t spo2 = 0;                  //Calculated spo2 value
uint16_t redAC = 0;                //AC component of red LED signal
uint32_t redDC = 0;                //DC component of red LED signal
uint16_t irAC = 0;                 //AC component of IR LED signal
uint32_t irDC = 0;                 //DC component of IR LED signal
int16_t eachSampleDiff = 0;        //Difference between current and previous sample


MAX30102_Status_t MAX30102_Init(I2C_HandleTypeDef *hi2c)
{
	uint8_t data;

    // Step 1: Software reset the device
    // Setting bit 6 (RESET) in the MODE_CONFIG register triggers a reset. The sensor will reset all configuration,
    // threshold, and data registers to their power-on-state. The RESET bit will clear automatically once the reset completes.
    data = 0x40; // 0b01000000
    if (MAX30102_WriteRegister(hi2c, MAX30102_REG_MODE_CONFIG, data) != MAX30102_OK)
    {
        return MAX30102_ERROR;
    }

    // Wait for reset to complete
    // The datasheet advises polling the RESET bit in the MODE_CONFIG register to check when it clears (indicating reset completion).
    do
    {
        if (MAX30102_ReadRegister(hi2c, MAX30102_REG_MODE_CONFIG, &data) != MAX30102_OK)
        {
            return MAX30102_ERROR;
        }
    } while (data & 0x40);  // Keep checking until RESET bit clears

    // Step 2: Enable new data interrupt
    // Setting bit 6 in the INTERRUPT_ENABLE_1 register enables the "new FIFO data ready" interrupt.
    // This interrupt is triggered when new data is available in the FIFO, allowing the main program to know
    // when to read from the FIFO.
    data = 0x40; // Enable new data ready interrupt (bit 6)
    if (MAX30102_WriteRegister(hi2c, MAX30102_REG_INTR_ENABLE_1, data) != MAX30102_OK)
    {
        return MAX30102_ERROR;
    }

    // Step 3: Configure the SPO2 settings
    // This step configures ADC resolution, sample rate, and LED pulse width in the SPO2_CONFIG register.
    // - Bits [5:4] (SPO2_ADC_RGE) set the ADC range. Here, we use a range of 16384, providing maximum sensitivity.
    // - Bits [3:2] (SPO2_SR) set the sample rate to 50 samples per second, per the application requirements.
    // - Bits [1:0] (LED_PW) set LED pulse width to 411 µs, maximizing the signal strength.
    data = 0x63; // 0b01100011 for 16384 ADC range, 50 samples per second, 411 µs pulse width
    if (MAX30102_WriteRegister(hi2c, MAX30102_REG_SPO2_CONFIG, data) != MAX30102_OK)
    {
        return MAX30102_ERROR;
    }

    // Step 4: Set initial LED pulse amplitude
    // The LED pulse amplitude controls the brightness of the Red and IR LEDs, affecting the strength of the signals reflected back.
    // Here, we set an initial amplitude of 0x47 for both LED channels to ensure a strong signal.
    data = 0x47;  // Initial brightness level for Red and IR LEDs
    if (MAX30102_WriteRegister(hi2c, MAX30102_REG_LED1_PA, data) != MAX30102_OK)
    {
        return MAX30102_ERROR;
    }
    if (MAX30102_WriteRegister(hi2c, MAX30102_REG_LED2_PA, data) != MAX30102_OK)
    {
        return MAX30102_ERROR;
    }

    // Step 5: Clear FIFO pointers
    // The datasheet advises clearing the FIFO pointers before entering SpO2 or Heart Rate mode. This is done
    // by setting the FIFO_WR_PTR, OVF_COUNTER, and FIFO_RD_PTR registers to 0x00 to ensure a known state.
    data = 0;
    if (MAX30102_WriteRegister(hi2c, MAX30102_REG_FIFO_WR_PTR, data) != MAX30102_OK)
    {
        return MAX30102_ERROR;
    }
    if (MAX30102_WriteRegister(hi2c, MAX30102_REG_OVF_COUNTER, data) != MAX30102_OK)
    {
        return MAX30102_ERROR;
    }
    if (MAX30102_WriteRegister(hi2c, MAX30102_REG_FIFO_RD_PTR, data) != MAX30102_OK)
    {
        return MAX30102_ERROR;
    }

    // Step 6: Set SPO2 mode
    // To enter SpO2 mode, configure the MODE_CONFIG register with 0x03. This mode allows the sensor to collect data
    // for Red and IR LEDs, which are used for SpO2 and heart rate calculations.
    data = 0x03;  // 0b00000011 to set SpO2 mode
    if (MAX30102_WriteRegister(hi2c, MAX30102_REG_MODE_CONFIG, data) != MAX30102_OK)
    {
        return MAX30102_ERROR;
    }

    return MAX30102_OK;
}

MAX30102_Status_t MAX30102_WriteRegister(I2C_HandleTypeDef *i2cHandle, uint8_t registerAddress, uint8_t data)
{
    //This function writes a single byte (data) to a specified register in the MAX30102 sensor.
    //1. `i2cHandle`: The I2C handle for communication
    //2. `registerAddress`: The register address in the MAX30102 to which we want to write
    //3. `data`: The byte of data we want to write to the register

    //We use HAL_I2C_Mem_Write to handle the I2C memory write operation.
    //Parameters:
    // - `i2cHandle`: I2C handle
    // - `MAX30102_I2C_ADDRESS`: I2C address of the MAX30102 device
    // - `registerAddress`: Target register in the MAX30102 for the write
    // - `I2C_MEMADD_SIZE_8BIT`: Memory address size (8-bit register address)
    // - `&data`: Pointer to the byte of data to be written
    // - `1`: Number of bytes to write
    // - `HAL_MAX_DELAY`: Maximum wait time for the I2C operation

    if (HAL_I2C_Mem_Write(i2cHandle,
                          MAX30102_I2C_ADDRESS,
                          registerAddress,
                          I2C_MEMADD_SIZE_8BIT,
                          &data,
                          1,
                          HAL_MAX_DELAY) != HAL_OK)
    {
        return MAX30102_ERROR; //Return error status if the write operation fails
    }

    return MAX30102_OK; //Return OK status if the write operation is successful
}

MAX30102_Status_t MAX30102_ReadRegister(I2C_HandleTypeDef *i2cHandle, uint8_t registerAddress, uint8_t *data)
{
    //This function reads a single byte from a specified register in the MAX30102 sensor.
    //1. `i2cHandle`: The I2C handle for communication
    //2. `registerAddress`: The register address in the MAX30102 to read from
    //3. `data`: Pointer to store the byte of data read from the register

    //We use HAL_I2C_Mem_Read to handle the I2C memory read operation.
    //Parameters:
    // - `i2cHandle`: I2C handle
    // - `MAX30102_I2C_ADDRESS`: I2C address of the MAX30102 device
    // - `registerAddress`: Target register in the MAX30102 to read from
    // - `I2C_MEMADD_SIZE_8BIT`: Memory address size (8-bit register address)
    // - `data`: Pointer to store the byte read from the register
    // - `1`: Number of bytes to read
    // - `HAL_MAX_DELAY`: Maximum wait time for the I2C operation

    if (HAL_I2C_Mem_Read(i2cHandle,
                         MAX30102_I2C_ADDRESS,
                         registerAddress,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         1,
                         HAL_MAX_DELAY) != HAL_OK)
    {
        return MAX30102_ERROR; //Return error status if the read operation fails
    }

    return MAX30102_OK; //Return OK status if the read operation is successful
}


MAX30102_Status_t MAX30102_ReadFIFO(I2C_HandleTypeDef *i2cHandle, SAMPLE *samples, uint8_t numSamples)
{
    uint8_t rawData[6 * numSamples]; //Array to hold data for each sample; each sample contains 6 bytes (3 for Red, 3 for IR)

    //Set FIFO_DATA register (0x07) as the starting register for reading
    //According to the datasheet, the FIFO_DATA register does not auto-increment its address during a burst read
    //This means that reading multiple bytes from this address will provide continuous data from the FIFO
    uint8_t fifoDataRegAddr = MAX30102_REG_FIFO_DATA;
    if (HAL_I2C_Master_Transmit(i2cHandle, MAX30102_I2C_ADDRESS, &fifoDataRegAddr, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        return MAX30102_ERROR; //Return error if the register address transmission fails
    }

    //Burst-read all data from the FIFO_DATA register
    //Each sample contains data for two channels (Red and IR), each occupying 3 bytes (total 6 bytes per sample)
    //The FIFO read pointer auto-increments, so we can read all needed data in a single burst transaction
    if (HAL_I2C_Master_Receive(i2cHandle, MAX30102_I2C_ADDRESS, rawData, sizeof(rawData), HAL_MAX_DELAY) != HAL_OK)
    {
        return MAX30102_ERROR; //Return error if the read fails
    }

    //Now we process each sample, converting the raw data bytes into meaningful values for each channel
    //FIFO data is left-justified, meaning the significant bits are on the left side of the byte sequence
    //We only need the top 18 bits for each channel; the remaining bits are padding and should be ignored
    for (uint8_t idx = 0; idx < numSamples; idx++)
    {
        //To extract the Red channel data:
        //Each channel’s data (Red or IR) is stored in 3 consecutive bytes, with the most significant bits first
        //We combine these 3 bytes into a single 24-bit integer by shifting and OR-ing the bytes together

        //Extract the first byte of the Red channel data and shift it 16 bits to the left
        //This moves it to the top 8 bits of the 24-bit space (bits 16-23)
        uint32_t redChannelData = rawData[idx * 6] << 16;

        //Extract the second byte and shift it 8 bits to the left
        //This places it in the middle of the 24-bit space (bits 8-15)
        redChannelData |= rawData[idx * 6 + 1] << 8;

        //Extract the third byte as it is, placing it in the lowest 8 bits (bits 0-7)
        redChannelData |= rawData[idx * 6 + 2];

        //Mask the result to retain only the top 18 bits (as specified in the datasheet), discarding extra padding bits
        redChannelData &= 0x3FFFF;

        //Store the 18-bit Red channel data in the sample struct
        samples[idx].red = redChannelData;

        //Repeat the process for the IR channel data, which occupies the next 3 bytes in the FIFO data
        uint32_t irChannelData = rawData[idx * 6 + 3] << 16; 	//Shift the first IR byte 16 bits to the left
        irChannelData |= rawData[idx * 6 + 4] << 8;           	//Shift the second IR byte 8 bits to the left
        irChannelData |= rawData[idx * 6 + 5];                	//Use the third IR byte as is

        //Mask to keep only the top 18 bits for the IR channel, discarding any padding
        irChannelData &= 0x3FFFF;

        //Store the 18-bit IR channel data in the sample struct
        samples[idx].iRed = irChannelData;
    }

    return MAX30102_OK; //Return OK status if the function completes successfully
}

MAX30102_Status_t ON(I2C_HandleTypeDef *hi2c) {
    uint8_t data;

    // Read the current value of the MODE_CONFIG register to get the SHDN bit
    if (MAX30102_ReadRegister(hi2c, MAX30102_REG_MODE_CONFIG, &data) != MAX30102_OK) {
        return MAX30102_ERROR;
    }

    // Clear the SHDN bit to wake up the sensor
    data &= ~0x80; // Clear bit 7 (SHDN)
    if (MAX30102_WriteRegister(hi2c, MAX30102_REG_MODE_CONFIG, data) != MAX30102_OK) {
        return MAX30102_ERROR;
    }

    HAL_Delay(10);
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

/*
 * HOW DOES HEART RATE SENSOR WORK
 * Red and infrared data signals are read by the MAX30102 sensor.
 * These values are a result of the reflected light intensity after
 * the red and infrared LEDs hit a blood vessel
 * The reflection intensity will be low during a heart beat since
 * blood volume is higher in the arteries. Higher blood volume leads
 * to slightly darker skin around the finger thus more light
 * is absorbed. Since more light is absorbed, less will be reflected.
 * Thus in graphing terms, a trough is reached when the heart beats.
 * The opposite is true between heart beats, blood volume is lower
 * leading to a lighter color which absorbs less light and reflects
 * more. In turn a peak is reached.
 * Calculating the time between peaks can allow an estimated
 * heart rate to be calculated.
 * Infrared data is primarily used for heart rate, data sheet
 * states red data is 'critical' for SPo2 which is necessary
 * for temperature sensing
 */


//Function to filter the sample by applying a moving average filter
MAX30102_Status_t MAX30102_FilterSample(SAMPLE *currentSample) {
    uint32_t redSum = 0;
    uint32_t irSum = 0;

    // Moving average over the last FILTER_LEVEL samples
    for (uint8_t i = 0; i < FILTER_LEVEL - 1; i++) {
        redSum += sampleBuff[i].red;
        irSum += sampleBuff[i].iRed;
    }

    // Update the current sample with the averaged value
    currentSample->red = (redSum + currentSample->red) / FILTER_LEVEL;
    currentSample->iRed = (irSum + currentSample->iRed) / FILTER_LEVEL;
    return MAX30102_OK;
}

// Function to insert a new sample into the buffer and shift older samples back
MAX30102_Status_t MAX30102_InsertSample(SAMPLE newSample) {
    // Shift samples back to make room for the new sample
    for (uint8_t i = BUFFER_SIZE - 1; i > 0; i--) {
        sampleBuff[i].red = sampleBuff[i - 1].red;
        sampleBuff[i].iRed = sampleBuff[i - 1].iRed;
    }

    // Insert the new sample at the beginning of the buffer
    sampleBuff[0] = newSample;
    return MAX30102_OK;
}

// Function to calculate AC and DC components of the red and IR channels
MAX30102_Status_t MAX30102_CalculateAcDc(uint16_t *redAC, uint32_t *redDC, uint16_t *irAC, uint32_t *irDC) {
    uint32_t redMax = sampleBuff[0].red;
    uint32_t redMin = sampleBuff[0].red;
    uint32_t irMax = sampleBuff[0].iRed;
    uint32_t irMin = sampleBuff[0].iRed;

    // Iterate through the buffer to find min and max values for AC and DC calculations
    for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
        if (sampleBuff[i].red > redMax) redMax = sampleBuff[i].red;
        if (sampleBuff[i].red < redMin) redMin = sampleBuff[i].red;
        if (sampleBuff[i].iRed > irMax) irMax = sampleBuff[i].iRed;
        if (sampleBuff[i].iRed < irMin) irMin = sampleBuff[i].iRed;
    }

    // AC component is the range between max and min; DC component is the average of max and min
    *redAC = redMax - redMin;
    *redDC = (redMax + redMin) / 2;
    *irAC = irMax - irMin;
    *irDC = (irMax + irMin) / 2;

    return MAX30102_OK;
}

uint8_t MAX30102_GetUnreadSampleCount(I2C_HandleTypeDef *hi2c) {
    uint8_t fifoWritePtr = 0;
    uint8_t fifoReadPtr = 0;

    // Read FIFO write pointer
    if (MAX30102_ReadRegister(hi2c, MAX30102_REG_FIFO_WR_PTR, &fifoWritePtr) != MAX30102_OK) {
        return 0;
    }

    // Read FIFO read pointer
    if (MAX30102_ReadRegister(hi2c, MAX30102_REG_FIFO_RD_PTR, &fifoReadPtr) != MAX30102_OK) {
        return 0;
    }

    // Calculate the number of unread samples (modulo 32 to handle wraparound)
    return (fifoWritePtr - fifoReadPtr) & 0x1F;
}



MAX30102_Status_t MAX30102_CalculateReadings(I2C_HandleTypeDef *hi2c, float *heartRate, uint8_t *spo2) {
	uint8_t unreadSamples = MAX30102_GetUnreadSampleCount(hi2c);
    SAMPLE tempBuffer[5];
    MAX30102_ReadFIFO(hi2c, tempBuffer, unreadSamples);

    static uint8_t beatSampleCount = 0;
    static uint8_t lastTenBeatCounts[10];
    static uint32_t lastIRValue = 0;

    for (uint8_t i = 0; i < unreadSamples; i++) {
        if (tempBuffer[i].iRed < 40000) {
            *heartRate = 0;
            *spo2 = 0;
            eachSampleDiff = 0;
            continue;
        }

        MAX30102_InsertSample(tempBuffer[i]);
        MAX30102_CalculateAcDc(&redAC, &redDC, &irAC, &irDC);
        MAX30102_FilterSample(&tempBuffer[i]);

        float ratio = ((float)redAC / redDC) / ((float)irAC / irDC);
        if (ratio >= 0.36 && ratio < 0.66) {
            *spo2 = (uint8_t)(107 - 20 * ratio);  // Use *spo2 to dereference
        } else if (ratio >= 0.66 && ratio < 1) {
            *spo2 = (uint8_t)(129.64 - 54 * ratio);  // Use *spo2 to dereference
        }

        eachSampleDiff = lastIRValue - tempBuffer[i].iRed;
        if (eachSampleDiff > 50 && beatSampleCount > 12) {
            for (uint8_t j = 9; j > 0; j--) {
                lastTenBeatCounts[j] = lastTenBeatCounts[j - 1];
            }
            lastTenBeatCounts[0] = beatSampleCount;

            uint32_t totalTime = 0;
            for (uint8_t j = 0; j < 10; j++) {
                totalTime += lastTenBeatCounts[j];
            }

            *heartRate = (uint8_t)(60.0 * 10 / 0.02 / totalTime);  // Use *heartRate to dereference
            beatSampleCount = 0;
        }

        lastIRValue = tempBuffer[i].iRed;
        beatSampleCount++;
    }

    return MAX30102_OK;
}


// Returns the calculated heart rate
uint8_t MAX30102_GetHeartRate(void) {
    return heartRate;
}

// Returns the calculated SpO₂ level
uint8_t MAX30102_GetSpO2(void) {
    return spo2;
}

// Returns the difference between the current and last IR sample
int16_t MAX30102_GetDiff(void) {
    return eachSampleDiff;
}