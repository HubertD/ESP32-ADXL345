#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_config.h"
#include "LSM6DS3.h"

#define PIN_NUM_CLK 15
#define PIN_NUM_MOSI 13
#define PIN_NUM_MISO 26
#define PIN_NUM_CS 27
#define PIN_NUM_INT GPIO_NUM_14

extern "C" {
void app_main()
{
    printf("LSM6DS3 app starting!\n");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 64;

    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 0);
    ESP_ERROR_CHECK(ret);

    LSM6DS3 sensor(HSPI_HOST, PIN_NUM_CS, PIN_NUM_INT);
    if (sensor.Init())
    {
    	printf("Sensor init success.\n");
    }
    else
    {
    	printf("Sensor init failed!\n");
    	return;
    }

    unsigned i = 0;
	while (true)
	{
		sensor.Update();

		if ( (++i & 0xFF) == 0)
		{
			printf("pitch: %f roll:%f yaw:%f temp:%f\n", sensor.GetPitch(), sensor.GetRoll(), sensor.GetYaw(), sensor.GetTemperature());
		}
	}

	printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
}
