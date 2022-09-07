/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <unistd.h>

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0                        /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000               /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define MPU9250_SENSOR_ADDR 0x68       /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR 0x75 /*!< Register addresses of the "who am I" register */

#define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT 7

#define SCD30_SENSOR_ADDR 0x61
#define DATA_LEN 1

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1

static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SCD30_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    vTaskDelay(150 / portTICK_PERIOD_MS);
    i2c_master_write_byte(cmd, 0x02, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x02, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "ESP_Write_NO");
        return ret;
    }
    vTaskDelay(150 / portTICK_PERIOD_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SCD30_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_h + sizeof(uint8_t), ACK_VAL);
    ret = i2c_master_read_byte(cmd, data_h + 2 * sizeof(uint8_t), NACK_VAL);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "ESP_Read_write_NO");
    }
    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "ESP_Read_stop_NO");
    }
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "ESP_Read_NO");
    }
    return ret;
}

static esp_err_t i2c_master_sensor_CM(i2c_port_t i2c_num, uint8_t *data_h) // continuous measurement
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SCD30_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x10, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x81, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_sensor_CMS(i2c_port_t i2c_num, uint8_t *data_h) // continuous measurement stop
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SCD30_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    vTaskDelay(150 / portTICK_PERIOD_MS);
    i2c_master_write_byte(cmd, 0x01, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x04, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
static esp_err_t i2c_master_sensor_SR(i2c_port_t i2c_num, uint8_t *data_h) // Soft Reset
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SCD30_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xD3, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x04, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_sensor_readM(i2c_port_t i2c_num, uint8_t *data_h)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SCD30_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    vTaskDelay(150 / portTICK_PERIOD_MS);
    i2c_master_write_byte(cmd, 0x03, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        return ret;
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SCD30_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    for (int i = 0; i < 17; i++)
    {
        i2c_master_read_byte(cmd, data_h, ACK_VAL);
        data_h++;
    }
    i2c_master_read_byte(cmd, data_h, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void)
{
    while (1)
    {
        uint8_t sensor_data_1[18] = {0};
        //uint8_t sensor_data_2[3] = {0};

        ESP_ERROR_CHECK(i2c_master_init());
        ESP_LOGI(TAG, "I2C initialized successfully");

        // // 動作状態取得
        // i2c_master_sensor_test(I2C_MASTER_NUM, &sensor_data_1);
        // ESP_LOGI(TAG, "DATA READY STATUS = %X %X", sensor_data_1[0],sensor_data_1[1]);

        //測定値読み取り
        //i2c_master_sensor_test(I2C_MASTER_NUM, &sensor_data_2);
        i2c_master_sensor_readM(I2C_MASTER_NUM, &sensor_data_1);
        //ESP_LOGI(TAG, "DATA READY STATUS = %X %X", sensor_data_2[0], sensor_data_2[1]);
        ESP_LOGI(TAG, "READ DATA");
        for (int i = 0; i < 18; i += 6)
        {
            ESP_LOGI(TAG, "%X %X %X %X %X %X", sensor_data_1[i], sensor_data_1[i + 1], sensor_data_1[i + 2], sensor_data_1[i + 3], sensor_data_1[i + 4], sensor_data_1[i + 5]);
        }
        float co2;
        unsigned int tempU32;
        tempU32 = (unsigned int)((((unsigned int)sensor_data_1[0]) << 24) |
                                 (((unsigned int)sensor_data_1[1]) << 16) |
                                 (((unsigned int)sensor_data_1[3]) << 8) |
                                 ((unsigned int)sensor_data_1[4]));

        co2 = *(float *)&tempU32;
        ESP_LOGI(TAG, "co2 %f", co2);

        // //continuous measurement start
        // i2c_master_sensor_CM(I2C_MASTER_NUM, &sensor_data_1);
        // ESP_LOGI(TAG, "continuous measurement start!");

        // i2c_master_sensor_CMS(I2C_MASTER_NUM, &sensor_data_1);
        // ESP_LOGI(TAG, "continuous measurement stop!");

        // ESP_ERROR_CHECK(i2c_master_sensor_SR(I2C_MASTER_NUM, &sensor_data_1));
        // ESP_LOGI(TAG, "soft reset complete!");

        ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
        ESP_LOGI(TAG, "I2C unitialized successfully");
        sleep(2);
    }
}
