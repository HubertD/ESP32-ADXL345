#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "ADXL345.h"

extern "C" {
void app_main()
{
    printf("ADXL app starting!\n");

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num =  GPIO_NUM_27;
    conf.scl_io_num = GPIO_NUM_26;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;

    i2c_port_t i2c_port = I2C_NUM_0;
    i2c_param_config(i2c_port, &conf);
    if (i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0) == ESP_OK)
	{
		printf("I2C driver installed.\n");
	}
	else
	{
		printf("error initializing I2C.\n");
	}

	ADXL345 adxl(i2c_port, false);
	printf(adxl.Check() ? "ADXL check ok.\n" : "ADXL check not ok.\n");
	adxl.SetPowerControl(false, false, true, false);

	ADXL345::SensorData acc;
	while (true)
	{
		if (adxl.ReadSensorData(acc))
		{
			printf("%d,%d,%d\n", acc.GetRawX(), acc.GetRawY(), acc.GetRawZ());
		}
		else
		{		
			printf("ADXL sensor read failed.\n");
			vTaskDelay(100);
		}
	}

	printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
}
