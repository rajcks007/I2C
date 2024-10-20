https://github.com/ryankurte/stm32-base/blob/master/drivers/BSP/Components/Common/accelero.h
https://github.com/STMicroelectronics/stm32-lsm6dsl/blob/main/lsm6dsl.c
https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lsm6dsl_STdC/examples
https://github.com/STMicroelectronics/stm32-lsm6dsl/tree/main




#include "stm32f4xx_hal.h"  // Adjust based on your STM32 series (e.g., stm32f4xx_hal.h, stm32l4xx_hal.h)
#include "lsm6dsl.h"

#define LSM6DSL_I2C_ADDRESS  0x6B << 1  // 7-bit address shifted for HAL I2C

extern I2C_HandleTypeDef hi2c1;  // Assumes I2C1 is configured in STM32CubeMX

// LSM6DSL Register Addresses
#define LSM6DSL_WHO_AM_I_REG  0x0F
#define LSM6DSL_CTRL1_XL      0x10
#define LSM6DSL_CTRL2_G       0x11
#define LSM6DSL_OUTX_L_G      0x22
#define LSM6DSL_OUTX_H_G      0x23
#define LSM6DSL_OUTX_L_XL     0x28
#define LSM6DSL_OUTX_H_XL     0x29

// Function Prototypes
void LSM6DSL_Init(void);
uint8_t LSM6DSL_ReadID(void);
void LSM6DSL_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz);
void LSM6DSL_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az);
void LSM6DSL_ReadRegisters(uint8_t reg_addr, uint8_t *data, uint16_t size);

// Main function
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    // Initialize LSM6DSL
    LSM6DSL_Init();

    while (1)
    {
        int16_t gyro_x, gyro_y, gyro_z;
        int16_t accel_x, accel_y, accel_z;

        // Read gyroscope data
        LSM6DSL_ReadGyro(&gyro_x, &gyro_y, &gyro_z);

        // Read accelerometer data
        LSM6DSL_ReadAccel(&accel_x, &accel_y, &accel_z);

        // Add delay (e.g., 500 ms)
        HAL_Delay(500);
    }
}

// Initialize the LSM6DSL sensor
void LSM6DSL_Init(void)
{
    uint8_t data;

    // Check if LSM6DSL is connected by reading the WHO_AM_I register
    if (LSM6DSL_ReadID() == 0x6A)  // WHO_AM_I value should be 0x6A for LSM6DSL
    {
        // Set accelerometer to 1.66 kHz, 2g range
        data = 0x60;  // 1.66 kHz ODR, ±2g
        HAL_I2C_Master_Transmit(&hi2c1, LSM6DSL_I2C_ADDRESS, &data, 1, HAL_MAX_DELAY);

        // Set gyroscope to 1.66 kHz, 250 dps range
        data = 0x60;  // 1.66 kHz ODR, 250 dps
        HAL_I2C_Master_Transmit(&hi2c1, LSM6DSL_I2C_ADDRESS, &data, 1, HAL_MAX_DELAY);
    }
    else
    {
        // Handle sensor connection error
    }
}

// Read the LSM6DSL WHO_AM_I register to verify the sensor
uint8_t LSM6DSL_ReadID(void)
{
    uint8_t who_am_i = 0;
    LSM6DSL_ReadRegisters(LSM6DSL_WHO_AM_I_REG, &who_am_i, 1);
    return who_am_i;
}

// Read gyroscope data (X, Y, Z axes)
void LSM6DSL_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t gyro_data[6];

    // Read 6 bytes of gyro data (OUTX_L_G to OUTZ_H_G)
    LSM6DSL_ReadRegisters(LSM6DSL_OUTX_L_G, gyro_data, 6);

    // Combine low and high bytes to get 16-bit gyroscope values
    *gx = (int16_t)(gyro_data[1] << 8 | gyro_data[0]);
    *gy = (int16_t)(gyro_data[3] << 8 | gyro_data[2]);
    *gz = (int16_t)(gyro_data[5] << 8 | gyro_data[4]);
}

// Read accelerometer data (X, Y, Z axes)
void LSM6DSL_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az)
{
    uint8_t accel_data[6];

    // Read 6 bytes of accelerometer data (OUTX_L_XL to OUTZ_H_XL)
    LSM6DSL_ReadRegisters(LSM6DSL_OUTX_L_XL, accel_data, 6);

    // Combine low and high bytes to get 16-bit accelerometer values
    *ax = (int16_t)(accel_data[1] << 8 | accel_data[0]);
    *ay = (int16_t)(accel_data[3] << 8 | accel_data[2]);
    *az = (int16_t)(accel_data[5] << 8 | accel_data[4]);
}

// Function to read registers using I2C master transmit and receive
void LSM6DSL_ReadRegisters(uint8_t reg_addr, uint8_t *data, uint16_t size)
{
    // Transmit register address
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DSL_I2C_ADDRESS, &reg_addr, 1, HAL_MAX_DELAY);

    // Receive data from the sensor
    HAL_I2C_Master_Receive(&hi2c1, LSM6DSL_I2C_ADDRESS, data, size, HAL_MAX_DELAY);
}

// Error Handler (can be customized based on your needs)
void Error_Handler(void)
{
    while(1)
    {
        // Stay in this loop in case of error
    }
}
