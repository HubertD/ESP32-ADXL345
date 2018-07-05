#include "ADXL345.h"

ADXL345::ADXL345(i2c_port_t i2c, bool alternateAddress)
	: _i2c(i2c)
    , _addr_7bit(alternateAddress ? 0x1D : 0x53)
{	
}

bool ADXL345::Check()
{
	uint8_t id = 0;
	if (!GetDeviceId(id))
	{
		printf("ADXL345 GetDeviceId() failed\n");
		return false;
	}
	return id == DEVICE_ID;
}

bool ADXL345::GetDeviceId(uint8_t& deviceId)
{
	return Read(Register::DEVID, &deviceId, 1);
}

bool ADXL345::SetPowerControl(bool link, bool autoSleep, bool measure, bool sleep)
{
	uint8_t regval = 0;
	if (link) { regval |= 0x20; }
	if (autoSleep) { regval |= 0x10; }
	if (measure) { regval |= 0x08; }
	if (sleep) { regval |= 0x04; }
	return Write(Register::POWER_CTL, &regval, 1);
}

bool ADXL345::ReadSensorData(ADXL345::SensorData& data)
{
	return Read(Register::DATA_FORMAT, &data, sizeof(data));
}

bool ADXL345::Read(Register reg, void *data, size_t size)
{
    auto cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr_7bit << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, static_cast<uint8_t>(reg), true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr_7bit << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, static_cast<uint8_t*>(data), size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    auto ret = i2c_master_cmd_begin(_i2c, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

	return ret == ESP_OK;
}

bool ADXL345::Write(Register reg, const void *data, size_t size)
{
    auto cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr_7bit << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, static_cast<uint8_t>(reg), true);
    i2c_master_write(cmd, const_cast<uint8_t*>(static_cast<const uint8_t*>(data)), size, true);
    i2c_master_stop(cmd);

    auto ret = i2c_master_cmd_begin(_i2c, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

	return ret == ESP_OK;
}

