#pragma once
#include <stdint.h>
#include "driver/i2c.h"

class ADXL345
{
	public:
		enum class Range
		{
			Range_2g = 0x00,
			Range_4g = 0x01,
			Range_8g = 0x02,
			Range_16g = 0x03
		};

		class SensorData
		{
			public:
				int16_t GetRawX() { return (static_cast<int16_t>(datax1)<<8) | datax0; }
				int16_t GetRawY() { return (static_cast<int16_t>(datay1)<<8) | datay0; }
				int16_t GetRawZ() { return (static_cast<int16_t>(dataz1)<<8) | dataz0; }

			private:
				uint8_t data_format;
				uint8_t datax0, datax1;
				uint8_t datay0, datay1;
				uint8_t dataz0, dataz1;

		} __attribute__((packed));
 
		ADXL345(i2c_port_t i2c, bool alternateAddress);
		bool Check();
		bool SetPowerControl(bool link, bool autoSleep, bool measure, bool sleep);
		bool SetDataFormat(bool selfTest, Range range);
		bool GetDeviceId(uint8_t& deviceId);
		bool ReadSensorData(SensorData& data);

	private:
		static constexpr const uint8_t DEVICE_ID = 0b11100101;

		enum class Register 
		{
			DEVID = 0x00,
			POWER_CTL = 0x2D,
			DATA_FORMAT = 0x31,
			DATAX0 = 0x32,
			DATAX1 = 0x33,
			DATAY0 = 0x34,
			DATAY1 = 0x35,
			DATAZ0 = 0x36,
			DATAZ1 = 0x37,
		};

		i2c_port_t _i2c;
		uint8_t _addr_7bit;

		bool Read(Register reg, void *data, size_t size);
		bool Write(Register reg, const void *data, size_t size);
};


