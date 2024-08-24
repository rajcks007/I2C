#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"

#define I2C_MASTER_SCL_IO    22    // GPIO22 for I2C clock
#define I2C_MASTER_SDA_IO    21    // GPIO21 for I2C data
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000  // 100kHz
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define I2C_SLAVE_ADDR       0x68  // Replace with your slave device address
#define READ_DELAY_MS        500  // Delay between reads in milliseconds

static const char *TAG = "I2C_EXAMPLE";

void app_main(void)
{
    esp_err_t ret;

    // Configure I2C master
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(ret));
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return;
    }

    // Write data to I2C slave
    uint8_t write_data = 0x00;  // Data to write
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, &write_data, sizeof(write_data), true);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(READ_DELAY_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Data 0x00 written successfully");
    }

     while (1) {
        // Read data from I2C slave
        uint8_t read_data[3] = {0};  // Buffer to store the read data
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, read_data, sizeof(read_data), I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(READ_DELAY_MS));
        i2c_cmd_link_delete(cmd);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
        } else {
            // Print received data
            ESP_LOGI(TAG, "accelerometer_x: 0x%02X " , read_data[0]);
            ESP_LOGI(TAG, "accelerometer_y: 0x%02X " , read_data[1]);
            ESP_LOGI(TAG, "accelerometer_z: 0x%02X " , read_data[2]);
            ESP_LOGI(TAG, "temperature: 0x%02X " , read_data[3]);
        }

        // Wait for a while before the next read
        vTaskDelay(pdMS_TO_TICKS(READ_DELAY_MS));
    }
}
