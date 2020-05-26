// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <stdio.h>
#include "driver/i2c.h"
#include "iot_i2c_bus.h"
#include "esp_log.h"
#include "iot_hdc2010.h"

#define WRITE_BIT                       I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT                        I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN                    0x1               /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                   0x0               /*!< I2C master will not check ack from slave */
#define ACK_VAL                         0x0               /*!< I2C ack value */
#define NACK_VAL                        0x1               /*!< I2C nack value */

typedef struct {
    i2c_bus_handle_t bus;
    uint16_t dev_addr;
} hdc2010_dev_t;

static esp_err_t iot_hdc2010_init(hdc2010_handle_t sensor);

hdc2010_handle_t iot_hdc2010_create(i2c_bus_handle_t bus, uint16_t dev_addr)
{
    hdc2010_dev_t* sensor = (hdc2010_dev_t*) calloc(1, sizeof(hdc2010_dev_t));
    sensor->bus = bus;
    sensor->dev_addr = dev_addr;
    iot_hdc2010_init(sensor);
    return (hdc2010_handle_t) sensor;
}

esp_err_t iot_hdc2010_delete(hdc2010_handle_t sensor, bool del_bus)
{
    hdc2010_dev_t* device = (hdc2010_dev_t*) sensor;
    if (del_bus) {
        iot_i2c_bus_delete(device->bus);
        device->bus = NULL;
    }
    free(device);
    return ESP_OK;
}

static esp_err_t iot_hdc2010_write_byte(hdc2010_handle_t sensor, uint8_t address, uint8_t write_data)
{
    esp_err_t ret;
    hdc2010_dev_t* device = (hdc2010_dev_t*) sensor;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, write_data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = iot_i2c_bus_cmd_begin(device->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static uint8_t iot_hdc2010_read_byte(hdc2010_handle_t sensor, uint8_t address)
{
    uint8_t data_rd = 0;
    int data_res = HDC2010_ERR_VAL;
    hdc2010_dev_t* device = (hdc2010_dev_t*) sensor;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->dev_addr << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_rd, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = iot_i2c_bus_cmd_begin(device->bus, cmd, 1000 / portTICK_RATE_MS);
    if (ret != ESP_OK) {
        ESP_LOGE("hdc2010:", "errno id :%d\n", ret);
    } else {
        data_res = data_rd;
    }
    i2c_cmd_link_delete(cmd);
    return data_res;
}

static bool _not_valid_return_val(int val)
{
    return (val == HDC2010_ERR_VAL);
}

float iot_hdc2010_get_temperature(hdc2010_handle_t sensor)
{
    int temperature_high = 0;
    int temperature_low = 0;
    float real_temp = 0;
    temperature_high = iot_hdc2010_read_byte(sensor, HDC2010_TEMP_HIGH);
    temperature_low = iot_hdc2010_read_byte(sensor, HDC2010_TEMP_LOW);
    if (_not_valid_return_val(temperature_high) || _not_valid_return_val(temperature_low)) {
        return (float)HDC2010_ERR_VAL;
    }
    real_temp = ((float) ((temperature_high << 8) | temperature_low) / (1 << 16) * 165 - 40);
    return real_temp;
}

float iot_hdc2010_get_humidity(hdc2010_handle_t sensor)
{
    int humidity_high = 0;
    int humidity_low = 0;
    float real_humi = 0;
    humidity_high = iot_hdc2010_read_byte(sensor, HDC2010_HUM_HIGH);
    humidity_low = iot_hdc2010_read_byte(sensor, HDC2010_HUM_LOW);
    if (_not_valid_return_val(humidity_high) || _not_valid_return_val(humidity_low)) {
        return (float)HDC2010_ERR_VAL;
    }
    real_humi = ((float) ((humidity_high << 8) | humidity_low) * 100) / (1 << 16);
    return real_humi;
}

esp_err_t iot_hdc2010_get_interrupt_info(hdc2010_handle_t sensor, hdc2010_interrupt_info_t * info)
{
    int config_data = 0;
    config_data = iot_hdc2010_read_byte(sensor, HDC2010_INTERRUPT);
    if (_not_valid_return_val(config_data)) {
        return ESP_FAIL;
    }
    info->drdy_status = (config_data & 0x80) && 1;
    info->th_status = (config_data & 0x40) && 1;
    info->tl_status = (config_data & 0x20) && 1;
    info->hh_status = (config_data & 0x10) && 1;
    info->hl_status = (config_data & 0x08) && 1;
    return ESP_OK;
}

esp_err_t iot_hdc2010_set_interrupt_info(hdc2010_handle_t sensor, hdc2010_interrupt_info_t * info)
{
    uint8_t config_data = 0;
    config_data |= info->drdy_status << 7;
    config_data |= info->th_status << 6;
    config_data |= info->tl_status << 5;
    config_data |= info->hh_status << 4;
    config_data |= info->hl_status << 3;
    return iot_hdc2010_write_byte(sensor, HDC2010_INTERRUPT, config_data);
}


esp_err_t iot_hdc2010_set_interrupt_config(hdc2010_handle_t sensor, hdc2010_interrupt_config_t * config)
{
    uint8_t config_data = 0;
    config_data |= config->drdy_mask << 7;
    config_data |= config->th_mask << 6;
    config_data |= config->tl_mask << 5;
    config_data |= config->hh_mask << 4;
    config_data |= config->hl_mask << 3;
    return iot_hdc2010_write_byte(sensor, HDC2010_INT_MASK, config_data);
}

esp_err_t iot_hdc2010_get_interrupt_config(hdc2010_handle_t sensor, hdc2010_interrupt_config_t * config)
{
    int config_data = 0;
    config_data = iot_hdc2010_read_byte(sensor, HDC2010_INT_MASK);
    if (_not_valid_return_val(config_data)) {
        return ESP_FAIL;
    }
    config->drdy_mask = (config_data & 0x80) && 1;
    config->th_mask = (config_data & 0x40) && 1;
    config->tl_mask = (config_data & 0x20) && 1;
    config->hh_mask = (config_data & 0x10) && 1;
    config->hl_mask = (config_data & 0x08) && 1;
    return ESP_OK;
}

float iot_hdc2010_get_max_temperature(hdc2010_handle_t sensor)
{
    int temperature_data = 0;
    float real_max_temp = 0;
    temperature_data = iot_hdc2010_read_byte(sensor, HDC2010_TEMPE_MAX);
    if (_not_valid_return_val(temperature_data)) {
        return (float)HDC2010_ERR_VAL;
    }
    real_max_temp = (((float) temperature_data / (1 << 8) * 165) - 40);
    return real_max_temp;
}

float iot_hdc2010_get_max_humidity(hdc2010_handle_t sensor)
{
    int humidity_data = 0;
    float real_max_humi = 0;
    humidity_data = iot_hdc2010_read_byte(sensor, HDC2010_HUM_MAX);
    if (_not_valid_return_val(humidity_data)) {
        return (float)HDC2010_ERR_VAL;
    }
    real_max_humi = ((float) (humidity_data * 100) / (1 << 8));
    return real_max_humi;
}

esp_err_t iot_hdc2010_set_temperature_Offset(hdc2010_handle_t sensor, uint8_t Offset_data)
{
    return iot_hdc2010_write_byte(sensor, HDC2010_TEMP_OFFSET, Offset_data);
}

esp_err_t iot_hdc2010_set_humidity_Offset(hdc2010_handle_t sensor, uint8_t Offset_data)
{
    return iot_hdc2010_write_byte(sensor, HDC2010_HUM_OFFSET, Offset_data);
}

esp_err_t iot_hdc2010_set_temperature_threshold(hdc2010_handle_t sensor, float temperature_data)
{
    esp_err_t ret;
    uint16_t temperature;
    temperature = (uint16_t) (temperature_data + 40) / 165 * (1 << 16);
    ret = iot_hdc2010_write_byte(sensor, HDC2010_TEMP_THR_H, (temperature >> 8) & 0xFF);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }
    ret = iot_hdc2010_write_byte(sensor, HDC2010_TEMP_THR_L, temperature & 0xFF);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}


esp_err_t iot_hdc2010_set_temperature_high_threshold(hdc2010_handle_t sensor, float temperature_data)
{
    esp_err_t ret;
	uint8_t temp_thresh_high;
	// Verify user is not trying to set value outside bounds
	if (temperature_data < -40)
	{
		temperature_data = -40;
	}
	else if (temperature_data > 125)
	{
		temperature_data = 125;
	}
	// Calculate value to load into register
	temp_thresh_high= (uint8_t)(256 * (temperature_data + 40)/165);
    ret = iot_hdc2010_write_byte(sensor, HDC2010_TEMP_THR_H, temp_thresh_high );
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }
	return ret;
}

esp_err_t iot_hdc2010_set_temperature_low_threshold(hdc2010_handle_t sensor, float temperature_data)
{
    esp_err_t ret;
 	uint8_t temp_thresh_low;
	// Verify user is not trying to set value outside bounds
	if (temperature_data < -40)
	{
		temperature_data = -40;
	}
	else if (temperature_data > 125)
	{
		temperature_data = 125;
	}
	// Calculate value to load into register
	temp_thresh_low= (uint8_t)(256 * (temperature_data + 40)/165);
    ret = iot_hdc2010_write_byte(sensor, HDC2010_TEMP_THR_L, temp_thresh_low );
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }
	return ret;
}


float iot_hdc2010_get_temperature_high_threshold(hdc2010_handle_t sensor)
{
    float temperature_data = 0;
	float temperature_high_threshold = 0;
    temperature_data = iot_hdc2010_read_byte(sensor, HDC2010_TEMP_THR_H);
    if (_not_valid_return_val(temperature_data)) {
        return (float)HDC2010_ERR_VAL;
    }
    temperature_high_threshold  = (((float) temperature_data * 165/256) - 40);
    return temperature_high_threshold ;
}
   
float iot_hdc2010_get_temperature_low_threshold(hdc2010_handle_t sensor)
{
    float temperature_data = 0;
	float temperature_low_threshold = 0;
    temperature_data = iot_hdc2010_read_byte(sensor, HDC2010_TEMP_THR_L);
    if (_not_valid_return_val(temperature_data)) {
        return (float)HDC2010_ERR_VAL;
    }
    temperature_low_threshold  = (((float) temperature_data * 165/256) - 40);
    return temperature_low_threshold ;
}
   

esp_err_t iot_hdc2010_set_humidity_threshold(hdc2010_handle_t sensor, float humidity_data)
{
    esp_err_t ret;
    uint16_t humidity;
    humidity = (uint16_t) humidity_data / 100 * (1 << 16);
    ret = iot_hdc2010_write_byte(sensor, HDC2010_HUM_THR_H, (humidity >> 8) & 0xFF);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }
    ret = iot_hdc2010_write_byte(sensor, HDC2010_HUM_THR_L, humidity & 0xFF);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t iot_hdc2010_set_humidity_high_threshold(hdc2010_handle_t sensor, float humid_high)
{
    esp_err_t ret;
	
	uint8_t humid_thresh;
	
	// Verify user is not trying to set value outside bounds
	if (humid_high < 0)
	{
		humid_high = 0;
	}
	else if (humid_high > 100)
	{
		humid_high = 100;
	}
	
	// Calculate value to load into register
	humid_thresh = (uint8_t)(256 * (humid_high)/100);
    ret = iot_hdc2010_write_byte(sensor, HDC2010_HUM_THR_H, humid_thresh);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t iot_hdc2010_set_humidity_low_threshold(hdc2010_handle_t sensor, float humid_low)
{
    esp_err_t ret;
	
	uint8_t humid_thresh;
	
	// Verify user is not trying to set value outside bounds
	if (humid_low < 0)
	{
		humid_low = 0;
	}
	else if (humid_low > 100)
	{
		humid_low = 100;
	}
	
	// Calculate value to load into register
	humid_thresh = (uint8_t)(256 * (humid_low)/100);
    ret = iot_hdc2010_write_byte(sensor, HDC2010_HUM_THR_L, humid_thresh);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

float iot_hdc2010_get_humidity_high_threshold(hdc2010_handle_t sensor)
{
    uint8_t humidity_data = 0;
	float humidity_high_threshold = 0;
    humidity_data = iot_hdc2010_read_byte(sensor, HDC2010_HUM_THR_H);
    if (_not_valid_return_val(humidity_data)) {
        return (float)HDC2010_ERR_VAL;
    }
    humidity_high_threshold  = ((float) humidity_data * 100/256);
    return humidity_high_threshold ;
}
   
float iot_hdc2010_get_humidity_low_threshold(hdc2010_handle_t sensor)
{
    uint8_t humidity_data = 0;
	float humidity_low_threshold = 0;
    humidity_data = iot_hdc2010_read_byte(sensor, HDC2010_HUM_THR_L);
    if (_not_valid_return_val(humidity_data)) {
        return (float)HDC2010_ERR_VAL;
    }
    humidity_low_threshold  = ((float) humidity_data * 100/256);
    return humidity_low_threshold ;
}
esp_err_t iot_hdc2010_set_reset_and_drdy(hdc2010_handle_t sensor, hdc2010_reset_and_drdy_t * config)
{
    uint8_t config_data = 0;
    config_data |= config->soft_res << 7;
	//config_data |= config->heat_en << 7;
    config_data |= config->output_rate << 4;
    config_data |= config->heat_en << 3;
    config_data |= config->int_en << 2;
    config_data |= config->int_pol << 1;
    config_data |= config->int_mode << 0;
    return iot_hdc2010_write_byte(sensor, HDC2010_RESET_INT_CONF, config_data);
}

esp_err_t iot_hdc2010_set_measurement_config(hdc2010_handle_t sensor, hdc2010_measurement_config_t * config)
{
    uint8_t config_data = 0;
    config_data |= config->tres << 6;
    config_data |= config->hres << 4;
    config_data |= config->meas_conf << 1;
    config_data |= config->meas_trig << 0;
    return iot_hdc2010_write_byte(sensor, HDC2010_MEASURE_CONF, config_data);
}

int iot_hdc2010_get_manufacturer_id(hdc2010_handle_t sensor)
{
    int manufacturer_id_high = 0;
    int manufacturer_id_low = 0;
    manufacturer_id_high = iot_hdc2010_read_byte(sensor, HDC2010_MANUFACTURER_ID_H);
    manufacturer_id_low = iot_hdc2010_read_byte(sensor, HDC2010_MANUFACTURER_ID_L);
    if (_not_valid_return_val(manufacturer_id_high) || _not_valid_return_val(manufacturer_id_low)) {
        return HDC2010_ERR_VAL;
    }
    return (manufacturer_id_high << 8) + manufacturer_id_low;
}

int iot_hdc2010_get_device_id(hdc2010_handle_t sensor)
{
    int device_id_high = 0;
    int device_id_low = 0;
    device_id_high = iot_hdc2010_read_byte(sensor, HDC2010_DEVICE_ID_H);
    device_id_low = iot_hdc2010_read_byte(sensor, HDC2010_DEVICE_ID_L);
    if (_not_valid_return_val(device_id_high) || _not_valid_return_val(device_id_low)) {
        return HDC2010_ERR_VAL;
    }
    return (device_id_high << 8) + device_id_low;
}

static esp_err_t iot_hdc2010_init(hdc2010_handle_t sensor)
{
    esp_err_t ret;
    hdc2010_reset_and_drdy_t reset_config, normal_config;
    hdc2010_measurement_config_t measurement_config;
	hdc2010_interrupt_config_t interrupt_config;
    hdc2010_interrupt_info_t interrupt_info;

	static const char *TAG1 = "hdc2010";
	reset_config.soft_res = HDC2010_SOFT_RESET;
    reset_config.output_rate = HDC2010_ODR_000;
    reset_config.heat_en = HDC2010_HEATER_OFF;
	reset_config.int_en =  HDC2010_DRDY_INT_EN_HIGH_Z;
    reset_config.int_pol = HDC2010_INT_POL_ACTIVE_LOW;
    reset_config.int_mode = HDC2010_INT_LEVEL_SENSITIVE;
	
	normal_config.soft_res = HDC2010_NORMAL_OPERATION;
    normal_config.output_rate = HDC2010_ODR_100;
    normal_config.heat_en = HDC2010_HEATER_OFF;
	normal_config.int_en =   HDC2010_DRDY_INT_EN_ENABLE;
    normal_config.int_pol = HDC2010_INT_POL_ACTIVE_HIGH;
    normal_config.int_mode = HDC2010_INT_LEVEL_SENSITIVE;
	
    measurement_config.tres = HDC2010_TRES_BIT_14;
    measurement_config.hres = HDC2010_HRES_BIT_14;
    measurement_config.meas_conf = HDC2010_MEAS_CONF_HUM_AND_TEMP;
    measurement_config.meas_trig = HDC2010_MEAS_START;
	
    interrupt_config.drdy_mask = HDC2010_DRDY_MASK_INT_DISABLE;
	interrupt_config.hh_mask = HDC2010_HH_MASK_INT_DISABLE;
	interrupt_config.hl_mask = HDC2010_HL_MASK_INT_DISABLE;
	interrupt_config.th_mask = HDC2010_TH_MASK_INT_DISABLE;
	interrupt_config.tl_mask = HDC2010_TL_MASK_INT_DISABLE;
	
    ret = iot_hdc2010_set_reset_and_drdy(sensor, &reset_config);
	ESP_LOGI(TAG1, "step2");
    if (ret != ESP_OK) {
		ESP_LOGI(TAG1, "step3");
        return ESP_FAIL;
    }
	
	
	ESP_LOGI(TAG1, "hello2");
	ret = iot_hdc2010_set_reset_and_drdy(sensor, &normal_config);
	ESP_LOGI(TAG1, "step15");
    if (ret != ESP_OK) {
		ESP_LOGI(TAG1, "step16");
        return ESP_FAIL;
    }
 
	ESP_LOGI(TAG1, "hello");
	ret = iot_hdc2010_set_interrupt_config(sensor, &interrupt_config);
	ESP_LOGI(TAG1, "step10");
    if (ret != ESP_OK) {
		ESP_LOGI(TAG1, "step11");
        return ESP_FAIL;
    }

    ret = iot_hdc2010_set_measurement_config(sensor, &measurement_config);
	ESP_LOGI(TAG1, "step 6");
    if (ret != ESP_OK) {
		ESP_LOGI(TAG1, "step 7");
        return ESP_FAIL;
    }
	
	ret =iot_hdc2010_get_interrupt_info(sensor, &interrupt_info);
		ESP_LOGI(TAG1, "step 8");
	if (ret != ESP_OK) {
		ESP_LOGI(TAG1, "step 9");
        return ESP_FAIL;
    }
   ESP_LOGI(TAG1, "interrupt intialization info: drdy_status_initial %d, th_status %d, tl_status %d, hh_status %d,hl_status %d",  interrupt_info.drdy_status, interrupt_info.th_status, interrupt_info.tl_status, interrupt_info.hh_status, interrupt_info.hl_status);

    return ESP_OK;
}

