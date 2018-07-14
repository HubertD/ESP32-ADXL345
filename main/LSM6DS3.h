#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"

class LSM6DS3
{
	public:
		LSM6DS3(spi_host_device_t device, int cs_iopin, gpio_num_t pin_int1);
		bool Init();
		bool Check();
		bool Update();
		double GetPitch() { return _pitch; }
		double GetRoll() { return _roll; }
		double GetYaw() { return _yaw; }
		double GetTemperature() { return _temperature; }

	private:
		enum class Register
		{
			FUNC_CFG_ACCESS = 0x01,
			SENSOR_SYNC_TIME_FRAME = 0x04,
			FIFO_CTRL1 = 0x06,
			FIFO_CTRL2 = 0x07,
			FIFO_CTRL3 = 0x08,
			FIFO_CTRL4 = 0x09,
			FIFO_CTRL5 = 0x0A,
			ORIENT_CFG_G = 0x0B,
			INT1_CTRL = 0x0D,
			INT2_CTRL = 0x0E,
			WHO_AM_I = 0x0F,
			CTRL1_XL = 0x10,
			CTRL2_G = 0x11,
			CTRL3_C = 0x12,
			CTRL4_C = 0x13,
			CTRL5_C = 0x14,
			CTRL6_C = 0x15,
			CTRL7_G = 0x16,
			CTRL8_XL = 0x17,
			CTRL9_XL = 0x18,
			CTRL10_C = 0x19,
			MASTER_CONFIG = 0x1A,
			WAKEUP_SRC = 0x1B,
			TAP_SRC = 0x1C,
			D6D_SRC = 0x1D,
			STATUS_REG = 0x1E,
			OUT_TEMP_L = 0x20,
			OUT_TEMP = 0x21,
			OUTX_L_G = 0x22,
			OUTX_H_G = 0x23,
			OUTY_L_G = 0x24,
			OUTY_H_G = 0x25,
			OUTZ_L_G = 0x26,
			OUTZ_H_G = 0x27,
			OUTX_L_XL = 0x28,
			OUTX_H_XL = 0x29,
			OUTY_L_XL = 0x2A,
			OUTY_H_XL = 0x2B,
			OUTZ_L_XL = 0x2C,
			OUTZ_H_XL = 0x2D,
			SENSORHUB1_REG = 0x2E,
			SENSORHUB2_REG = 0x2F,
			SENSORHUB3_REG = 0x30,
			SENSORHUB4_REG = 0x31,
			SENSORHUB5_REG = 0x32,
			SENSORHUB6_REG = 0x33,
			SENSORHUB7_REG = 0x34,
			SENSORHUB8_REG = 0x35,
			SENSORHUB9_REG = 0x36,
			SENSORHUB10_REG = 0x37,
			SENSORHUB11_REG = 0x38,
			SENSORHUB12_REG = 0x39,
			FIFO_STATUS1 = 0x3A,
			FIFO_STATUS2 = 0x3B,
			FIFO_STATUS3 = 0x3C,
			FIFO_STATUS4 = 0x3D,
			FIFO_DATA_OUT_L = 0x3E,
			FIFO_DATA_OUT_H = 0x3F,
			TIMESTAMP0_REG = 0x40,
			TIMESTAMP1_REG = 0x41,
			TIMESTAMP2_REG = 0x42,
			STEP_TIMESTAMP_L = 0x49,
			STEP_TIMESTAMP_H = 0x4A,
			STEP_COUNTER_L = 0x4B,
			STEP_COUNTER_H = 0x4C,
			SENSORHUB13_REG = 0x4D,
			SENSORHUB14_REG = 0x4E,
			SENSORHUB15_REG = 0x4F,
			SENSORHUB16_REG = 0x50,
			SENSORHUB17_REG = 0x51,
			SENSORHUB18_REG = 0x52,
			FUNC_SRC = 0x53,
			TAP_CFG = 0x58,
			TAP_THS_6D = 0x59,
			INT_DUR2 = 0x5A,
			WAKE_UP_THS = 0x5B,
			WALE_UP_DUR = 0x5C,
			FREE_FALL = 0x5D,
			MD1_CFG = 0x5E,
			MD2_CFG = 0x5F,
			OUT_MAG_RAW_X_L = 0x66,
			OUT_MAG_RAW_X_H = 0x67,
			OUT_MAG_RAW_Y_L = 0x68,
			OUT_MAG_RAW_Y_H = 0x69,
			OUT_MAG_RAW_Z_L = 0x6A,
			OUT_MAG_RAW_Z_H = 0x6B,
		};

		static constexpr const uint8_t MODE_WRITE = 0x00;
		static constexpr const uint8_t MODE_READ = 0x80;
		static constexpr const uint8_t WHO_AM_I_RESULT = 0x69;

		static constexpr const double LSB_STEP_GYRO = 1000.0/32768; // deg/s
		static constexpr const double LSB_STEP_ACCEL = 8.0/32768; // g
		static constexpr const double LSB_STEP_TEMP = 1.0/16; // °C
		static constexpr const double OFFSET_TEMP = 25.0; // °C

		spi_host_device_t _device;
		spi_device_handle_t _spi = nullptr;
		int _cs_iopin;
		gpio_num_t _pin_int1;
		double _pitch = 0.0;
		double _roll = 0.0;
		double _yaw = 0.0;
		double _temperature = 25.0;

		bool ReadRegister(Register reg, uint8_t& value);
		bool WriteRegister(Register reg, uint8_t value);

		bool Read16Bit(Register reg, uint16_t& value);
		bool Read16Bit(Register reg, int16_t& value);

		bool ReadData(Register reg, void* data, size_t num_bytes);

		bool EnableConfigAccess();
		bool DisableConfigAccess();
		bool ResetFifo();

};
