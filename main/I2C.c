#include <stdio.h>
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define I2C_MASTER_SCL_IO    22    // GPIO number for I2C clock
#define I2C_MASTER_SDA_IO    21    // GPIO number for I2C data
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000  // Frequency of the I2C bus
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define MPU6050_ADDR         0x68  // I2C address of the MPU-6050
#define MPU6050_PWR_MGMT_1   0x6B  // Register to wake up the MPU-6050
#define MPU6050_ACCEL_XOUT_H 0x3B  // Register for accelerometer X-axis high byte
// #define MPU6050_GYRO_XOUT_H  0x43  // Register for gyroscope X-axis high byte 

#define UART_NUM             UART_NUM_0  // UART port number
#define UART_TX_PIN          1          // GPIO number for UART TX      ----- for UART 1 -- 17
#define UART_RX_PIN          3          // GPIO number for UART RX      ----- for UART 1 -- 16
#define UART_BAUD_RATE       115200      // Baud rate for UART communication
#define UART_BUFFER_SIZE     1024               // Buffer size for incoming data

static const char *TAG = "MPU6050";

esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);  // Repeated start
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, UART_BUFFER_SIZE, 0, 0, NULL, 0);
}

void send_data_via_uart(const char *data) {
    size_t len = strlen(data);  // Calculate the length of the string
    uart_write_bytes(UART_NUM, data, len);  // Send the string via UART
}

void app_main(void) {

    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return;
    }

    // Initialize UART
    uart_init();

    // Wake up MPU6050 by writing 0x00 to the power management register
    ret = mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return;
    }

    while (1) {
        // Read accelerometer and gyroscope data
        uint8_t data[14];  // 6 bytes for accel, 6 bytes for gyro, 2 bytes for temp
        ret = mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, data, 14);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read sensor data");
            return;
        }

        // int temp;

        // Convert the raw data to signed double integers
        int16_t accel_x = (data[0] << 8) | data[1];
        int16_t accel_y = (data[2] << 8) | data[3];
        int16_t accel_z = (data[4] << 8) | data[5];
        int16_t temp = (data[6] << 8) | data[7];
        int16_t gyro_x = (data[8] << 8) | data[9];
        int16_t gyro_y = (data[10] << 8) | data[11];
        int16_t gyro_z = (data[12] << 8) | data[13];

        /*
        // Print accelerometer and gyroscope data in decimal format
        ESP_LOGI(TAG, "Accel X: %d, Y: %d, Z: %d", accel_x, accel_y, accel_z);
        ESP_LOGI(TAG, "temp : %d", temp);
        ESP_LOGI(TAG, "Gyro X: %d, Y: %d, Z: %d", gyro_x, gyro_y, gyro_z);
        */

        double accel_x1 = (accel_x / 819 ) * 0.05 ;           // to convert data
        double accel_y1 = (accel_y / 819 ) * 0.05 ;           // to convert data
        double accel_z1 = (accel_z / 819 ) * 0.05 ;           // to convert data
        double temp1 = (temp + 12420 ) / 340.0 ;           // to convert in celcius
        double gyro_y1 = (gyro_y / 655 ) * 5 ;           // to convert data into degree per sec.
        double gyro_z1 = (gyro_z / 655 ) * 5 ;           // to convert data into degree per sec.
        double gyro_x1 = (gyro_x / 655 ) * 5 ;           // to convert data into degree per sec.

        // Send the data string over UART0
        // Prepare data string
        char data_str[16];
        snprintf(data_str, sizeof(data_str), "[%.2f]", temp1);

        // // Send the data string over UART0
        send_data_via_uart(data_str);

        // Delay to control the reading frequency
        vTaskDelay(pdMS_TO_TICKS(250));  // Delay in micro-second
    }
}
