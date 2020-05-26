/* Example

   For other examples please check:
   https://github.com/espressif/esp-iot-solution/tree/master/examples

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>
#include "example1.h"
#include "example2.h"
*/
/******************************************************************************
 * FunctionName : app_main
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
/*void app_main()
{
    printf("in app code...\n");
    example1();
    example2();
}
*/
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
#include "unity.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "iot_i2c_bus.h"
#include "esp_system.h"
#include "iot_hdc2010.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"

#define HDC2010_I2C_MASTER_SCL_IO           (gpio_num_t)23         /*!< gpio number for I2C master clock */
#define HDC2010_I2C_MASTER_SDA_IO           (gpio_num_t)25         /*!< gpio number for I2C master data  */
#define HDC2010_I2C_MASTER_NUM              I2C_NUM_1           /*!< I2C port number for master dev */
#define HDC2010_I2C_MASTER_TX_BUF_DISABLE   0                      /*!< I2C master do not need buffer */
#define HDC2010_I2C_MASTER_RX_BUF_DISABLE   0                      /*!< I2C master do not need buffer */
#define HDC2010_I2C_MASTER_FREQ_HZ          100000                 /*!< I2C master clock frequency */
#define _I2C_OP_ADDR(_addr, _op) ((_addr << 1) | _op)
#define I2C_WRITE(_addr) _I2C_OP_ADDR(_addr, I2C_MASTER_WRITE)
#define I2C_READ(_addr) _I2C_OP_ADDR(_addr, I2C_MASTER_READ)
#define GPIO_INPUT_IO 27
#define GPIO_INPUT_PIN_SEL 1ULL<<GPIO_INPUT_IO 
#define ESP_INTR_FLAG_DEFAULT 0

static i2c_bus_handle_t i2c_bus = NULL;
static hdc2010_handle_t hdc2010 = NULL;
static const char *TAG = "hdc2010";
/**
 * @brief i2c master initialization
 */
static void i2c_master_init()
{
    int i2c_master_port = HDC2010_I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = HDC2010_I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = HDC2010_I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = HDC2010_I2C_MASTER_FREQ_HZ;
    i2c_bus = iot_i2c_bus_create(i2c_master_port, &conf);

    i2c_cmd_handle_t i2c_hdl1 = i2c_cmd_link_create();
    configASSERT(i2c_master_start(i2c_hdl1) == ESP_OK);
    configASSERT(i2c_master_write_byte(i2c_hdl1, I2C_WRITE(0x71), I2C_MASTER_ACK) == ESP_OK); 
    configASSERT(i2c_master_write_byte(i2c_hdl1, 0xFF, I2C_MASTER_ACK) == ESP_OK); // Turn on all the ports for I2C switch
    configASSERT(i2c_master_stop(i2c_hdl1) == ESP_OK);
    uint8_t id_reg_val1 = 0;    
    esp_err_t begin_res = i2c_master_cmd_begin(i2c_master_port, i2c_hdl1, 1000 / portTICK_RATE_MS); 
    i2c_cmd_link_delete(i2c_hdl1);
    printf("i2c_master_cmd_begin() result is: %d and id_reg_val1=0x%02X\n", begin_res, id_reg_val1);

	i2c_cmd_handle_t i2c_hdl_exp1 = i2c_cmd_link_create();
    configASSERT(i2c_master_start(i2c_hdl_exp1) == ESP_OK);
    configASSERT(i2c_master_write_byte(i2c_hdl_exp1, I2C_WRITE(0x22), I2C_MASTER_ACK) == ESP_OK);
    configASSERT(i2c_master_write_byte(i2c_hdl_exp1, 0x8C, I2C_MASTER_ACK) == ESP_OK);
	configASSERT(i2c_master_write_byte(i2c_hdl_exp1, 0x2B, I2C_MASTER_ACK) == ESP_OK);
	configASSERT(i2c_master_write_byte(i2c_hdl_exp1, 0xFF, I2C_MASTER_ACK) == ESP_OK);
	configASSERT(i2c_master_write_byte(i2c_hdl_exp1, 0x3F, I2C_MASTER_ACK) == ESP_OK);
    configASSERT(i2c_master_stop(i2c_hdl_exp1) == ESP_OK);
    esp_err_t exp_res1 = i2c_master_cmd_begin(i2c_master_port, i2c_hdl_exp1, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_hdl_exp1);
    printf("i2c_master_cmd_begin() for expander result is: %d  \n", exp_res1);
}


void hdc2010_init()
{
    i2c_master_init();
    hdc2010 = iot_hdc2010_create(i2c_bus, HDC2010_ADDR_PIN_SELECT_GND);
	
}

void hdc2010_test()
{
    hdc2010_init();
    int cnt = 10;
	float temperature_low_get_data=0, temperature_high_get_data = 0;
	float humidity_low_data = 0, humidity_high_data = 0;
	float humidity_low_set = 20;
	float humidity_high_set = 80;
	float temperature_low_set = 40;
	float temperature_high_set = 2;
	hdc2010_interrupt_info_t interrupt_info;
	hdc2010_interrupt_config_t interrupt_config;
	
	interrupt_config.drdy_mask = HDC2010_DRDY_MASK_INT_ENABLE;
	interrupt_config.hh_mask = HDC2010_HH_MASK_INT_DISABLE;
	interrupt_config.hl_mask = HDC2010_HL_MASK_INT_DISABLE;
	interrupt_config.th_mask = HDC2010_TH_MASK_INT_ENABLE;
	interrupt_config.tl_mask = HDC2010_TL_MASK_INT_ENABLE;
	configASSERT(iot_hdc2010_set_interrupt_config(hdc2010, &interrupt_config) == ESP_OK);  
	
	iot_hdc2010_set_temperature_low_threshold(hdc2010, temperature_low_set);
	iot_hdc2010_set_temperature_high_threshold(hdc2010, temperature_high_set);
	iot_hdc2010_set_humidity_low_threshold(hdc2010, humidity_low_set);
	iot_hdc2010_set_humidity_high_threshold(hdc2010, humidity_high_set);
	
    iot_hdc2010_get_interrupt_info(hdc2010, &interrupt_info);
	ESP_LOGI(TAG, "interrupt info first pass: drdy_status_initial %d, th_status %d, tl_status %d, hh_status %d,hl_status %d",  interrupt_info.drdy_status, interrupt_info.th_status, interrupt_info.tl_status, interrupt_info.hh_status, interrupt_info.hl_status);
	iot_hdc2010_get_interrupt_config(hdc2010, &interrupt_config);
	ESP_LOGI(TAG, "config  info first pass: drdy_mask_initial %d, th_mask %d, tl_mask %d, hh_mask %d,hl_mask %d",  interrupt_config.drdy_mask, interrupt_config.th_mask, interrupt_config.tl_mask, interrupt_config.hh_mask, interrupt_config.hl_mask);
	
	temperature_low_get_data = iot_hdc2010_get_temperature_low_threshold(hdc2010);
    ESP_LOGI(TAG, " temperature_low threshold:  %f",  temperature_low_get_data);
    temperature_high_get_data = iot_hdc2010_get_temperature_high_threshold(hdc2010);
    ESP_LOGI(TAG, " temperature_high threshold:  %f",  temperature_high_get_data);
	
	humidity_low_data = iot_hdc2010_get_humidity_low_threshold(hdc2010);
    ESP_LOGI(TAG, " humidity_low threshold:  %f",  humidity_low_data);
	humidity_high_data = iot_hdc2010_get_humidity_high_threshold(hdc2010);
    ESP_LOGI(TAG, " humidity_high threshold:  %f",  humidity_high_data);
	
	ESP_LOGI(TAG, "temperature %f", iot_hdc2010_get_temperature(hdc2010));
	ESP_LOGI(TAG, "humidity:%f", iot_hdc2010_get_humidity(hdc2010));
	ESP_LOGI(TAG, "max temperature:%f", iot_hdc2010_get_max_temperature(hdc2010));
	ESP_LOGI(TAG, "max humidity:%f", iot_hdc2010_get_max_humidity(hdc2010));
	iot_hdc2010_get_interrupt_info(hdc2010, &interrupt_info);
	ESP_LOGI(TAG, "interrupt info after read: drdy_status %d, th_status %d, tl_status %d, hh_status %d,hl_status %d ",  interrupt_info.drdy_status, interrupt_info.th_status, interrupt_info.tl_status, interrupt_info.hh_status, interrupt_info.hl_status);
	iot_hdc2010_get_interrupt_config(hdc2010, &interrupt_config);
	ESP_LOGI(TAG, "config info after read: drdy_mask %d, th_mask %d, tl_mask %d, hh_mask %d,hl_mask %d",  interrupt_config.drdy_mask, interrupt_config.th_mask, interrupt_config.tl_mask, interrupt_config.hh_mask, interrupt_config.hl_mask);
    ESP_LOGI(TAG,"**************************\n");
	vTaskDelay(10000 / portTICK_RATE_MS);
	ESP_LOGI(TAG, "temperature %f", iot_hdc2010_get_temperature(hdc2010));
	ESP_LOGI(TAG, "humidity:%f", iot_hdc2010_get_humidity(hdc2010));
	ESP_LOGI(TAG, "max temperature:%f", iot_hdc2010_get_max_temperature(hdc2010));
	ESP_LOGI(TAG, "max humidity:%f", iot_hdc2010_get_max_humidity(hdc2010));
	iot_hdc2010_get_interrupt_info(hdc2010, &interrupt_info);
	ESP_LOGI(TAG, "interrupt info after read: drdy_status %d, th_status %d, tl_status %d, hh_status %d,hl_status %d ",  interrupt_info.drdy_status, interrupt_info.th_status, interrupt_info.tl_status, interrupt_info.hh_status, interrupt_info.hl_status);
	iot_hdc2010_get_interrupt_config(hdc2010, &interrupt_config);
	ESP_LOGI(TAG, "config info after read: drdy_mask %d, th_mask %d, tl_mask %d, hh_mask %d,hl_mask %d",  interrupt_config.drdy_mask, interrupt_config.th_mask, interrupt_config.tl_mask, interrupt_config.hh_mask, interrupt_config.hl_mask);
    ESP_LOGI(TAG,"**************************\n");
	vTaskDelay(10000 / portTICK_RATE_MS);
	while (cnt--) {
		ESP_LOGI(TAG,"**************************");
		iot_hdc2010_get_interrupt_info(hdc2010, &interrupt_info);
	    ESP_LOGI(TAG, "interrupt infor before read : drdy_status_initial %d, th_status %d, tl_status %d, hh_status %d,hl_status %d",  interrupt_info.drdy_status, interrupt_info.th_status, interrupt_info.tl_status, interrupt_info.hh_status, interrupt_info.hl_status);
		iot_hdc2010_get_interrupt_config(hdc2010, &interrupt_config);
		ESP_LOGI(TAG, "config  info before read: drdy_mask_initial %d, th_mask %d, tl_mask %d, hh_mask %d,hl_mask %d",  interrupt_config.drdy_mask, interrupt_config.th_mask, interrupt_config.tl_mask, interrupt_config.hh_mask, interrupt_config.hl_mask);

        if(interrupt_info.tl_status)
		{
			ESP_LOGI(TAG, "temperature %f", iot_hdc2010_get_temperature(hdc2010));
			ESP_LOGI(TAG, "humidity:%f", iot_hdc2010_get_humidity(hdc2010));
			ESP_LOGI(TAG, "max temperature:%f", iot_hdc2010_get_max_temperature(hdc2010));
			ESP_LOGI(TAG, "max humidity:%f", iot_hdc2010_get_max_humidity(hdc2010));
			iot_hdc2010_get_interrupt_info(hdc2010, &interrupt_info);
			ESP_LOGI(TAG, "interrupt info after read: drdy_status %d, th_status %d, tl_status %d, hh_status %d,hl_status %d ",  interrupt_info.drdy_status, interrupt_info.th_status, interrupt_info.tl_status, interrupt_info.hh_status, interrupt_info.hl_status);
			iot_hdc2010_get_interrupt_config(hdc2010, &interrupt_config);
			ESP_LOGI(TAG, "config info after read: drdy_mask %d, th_mask %d, tl_mask %d, hh_mask %d,hl_mask %d",  interrupt_config.drdy_mask, interrupt_config.th_mask, interrupt_config.tl_mask, interrupt_config.hh_mask, interrupt_config.hl_mask);
            ESP_LOGI(TAG,"**************************\n");
			//vTaskDelay(6000 / portTICK_RATE_MS);
		}
		vTaskDelay(10000 / portTICK_RATE_MS);
    }
}

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void button(void *arg)
{
	uint32_t io_num;
	for(;;){
		if(xQueueReceive(gpio_evt_queue,&io_num,portMAX_DELAY)){
			printf("GPIO: %d,  interrupt , val: %d\n", io_num, gpio_get_level(io_num));
			}
		}
}


void app_main()
{
	
	gpio_config_t io_conf;
	//interrupt of rising edge
	io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
	//bit mask of pins, use GPIO 0
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull up mode
	io_conf.pull_up_en = 0;
	io_conf.pull_down_en = 0;
	gpio_config(&io_conf);
	
	//set gpio interrupt type for one pin_bit_mask
	gpio_set_intr_type(GPIO_INPUT_IO, GPIO_INTR_ANYEDGE);
	
	//create queue to handle gpio event from isr
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	
	//start gpio task
	xTaskCreate(button, "button", 2048, NULL, 10, NULL);
	
	//install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_INPUT_IO, gpio_isr_handler,(void*)GPIO_INPUT_IO);
	
	
    hdc2010_test();
	
	//remove isr for gpio number
	gpio_isr_handler_remove(GPIO_INPUT_IO);



}

