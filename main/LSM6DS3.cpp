#include "LSM6DS3.h"
#include <algorithm>
#include <array>
#include <string.h>
#include "freertos/task.h"

LSM6DS3::LSM6DS3(spi_host_device_t device, int cs_iopin, gpio_num_t pin_int1)
	: _device(device)
	, _cs_iopin(cs_iopin)
	, _pin_int1(pin_int1)
{
}

bool LSM6DS3::Init()
{
	gpio_set_direction(_pin_int1, GPIO_MODE_INPUT);

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.flags = SPI_DEVICE_HALFDUPLEX;
    devcfg.clock_speed_hz = 4*1000*1000; // 4 MHz
    devcfg.mode = 0;
    devcfg.command_bits = 0;
    devcfg.address_bits = 8;
    devcfg.dummy_bits = 0;
    devcfg.spics_io_num = _cs_iopin;
    devcfg.queue_size = 7;

    auto ret = spi_bus_add_device(_device, &devcfg, &_spi);
    ESP_ERROR_CHECK(ret);

    if (!Check()) {
    	printf("LSM6DS3 check failed!\n");
    	return false;
    }

    // software reset
    if (!WriteRegister(Register::CTRL3_C, 0x01)) { return false; }
    vTaskDelay(100);

    // set accelerometer ODR to 1660Hz, +/- 8g, filter bandwidth 400Hz
    if (!WriteRegister(Register::CTRL1_XL, 0b10001100)) { return false; }

    // set gyro ODR to 1660Hz, full scale at 1000dps
    if (!WriteRegister(Register::CTRL2_G, 0b10001000)) { return false; }

    // set block data update mode
    if (!WriteRegister(Register::CTRL3_C, 0b01000100)) { return false; }

    // enable storage of temperature values to FIFO
    if (!WriteRegister(Register::CTRL4_C, 0b0010000)) { return false; }

    // set acceleration x, y, z axis enabled
    if (!WriteRegister(Register::CTRL9_XL, 0x38)) { return false; }

    // set gyro x, y, z axis enabled
    if (!WriteRegister(Register::CTRL10_C, 0x38)) { return false; }

    // set INT1 pin when FIFO is not empty
    if (!WriteRegister(Register::INT1_CTRL,  0b00001000)) { return false; }

    // set FIFO threshold level to 1
    if (!WriteRegister(Register::FIFO_CTRL1, 0x01)) { return false; }

    // disable pedometer to allow for storing temperature in FIFO
    if (!WriteRegister(Register::FIFO_CTRL2, 0x00)) { return false; }

    // set gyro and acceleration FIFO decimation to None
    if (!WriteRegister(Register::FIFO_CTRL3, 0b00001001)) { return false; }

    // set temperature FIFO decimation to None
    if (!WriteRegister(Register::FIFO_CTRL4, 0b00001000)) { return false; }

    if (!ResetFifo()) { return false; }

    return true;
}

bool LSM6DS3::Check()
{
	uint8_t value = 0;
	return ReadRegister(Register::WHO_AM_I, value) && (value == WHO_AM_I_RESULT);
}

bool LSM6DS3::Update()
{
	while (gpio_get_level(_pin_int1) == 1) // FIFO non-empty -> read data
	{
		std::array<int16_t, 9> data;
		if (!ReadData(Register::FIFO_DATA_OUT_L, data.data(), data.size()*2)) { return false; }
		double dx = static_cast<double>(data[0]) * LSB_STEP_GYRO * (1.0/1660.0);
		double dy = static_cast<double>(data[1]) * LSB_STEP_GYRO * (1.0/1660.0);
		double dz = static_cast<double>(data[2]) * LSB_STEP_GYRO * (1.0/1660.0);
		_temperature = OFFSET_TEMP + static_cast<double>(data[7]) * LSB_STEP_TEMP;
		_pitch += dx;
		_roll += dy;
		_yaw += dz;
	}
	return true;

	/* alternate way using the fifo status register:
	uint16_t fifo_status;
	if (!Read16Bit(Register::FIFO_STATUS1, fifo_status)) { return false; }

	if (fifo_status & 0x1000)
	{
		// FIFO is empty
		return true;
	}

	if ((fifo_status & 0x6000)!=0)
	{
		printf("FIFO_STATUS=0x%04x\n", fifo_status);
		ResetFifo();
		return false;
	}

	size_t diff_fifo = fifo_status & 0xFFF;
	//printf("%d elements in FIFO\n", diff_fifo);
	for (size_t i=0; i<diff_fifo/6; i++)
	{
		int16_t rxdata[6];
		if (!ReadData(Register::FIFO_DATA_OUT_L, rxdata, 6*2)) { return false; }
		double dx = static_cast<double>(rxdata[0]) * LSB_STEP_GYRO * (1.0/1660.0);
		double dy = static_cast<double>(rxdata[1]) * LSB_STEP_GYRO * (1.0/1660.0);
		double dz = static_cast<double>(rxdata[2]) * LSB_STEP_GYRO * (1.0/1660.0);
		_pitch += dx;
		_roll += dy;
		_yaw += dz;
	}

	return true;
	 *
	 */
}

bool LSM6DS3::ReadRegister(Register reg, uint8_t& value)
{
	return ReadData(reg, &value, 1);
}

bool LSM6DS3::Read16Bit(Register reg, uint16_t& value)
{
	spi_transaction_t t;
	uint32_t rxd = 0;
	memset(&t, 0, sizeof(t));
	t.addr = MODE_READ | static_cast<uint8_t>(reg);
	t.length = 2*8;
	t.rxlength = 2*8;
	t.tx_buffer = nullptr;
	t.rx_buffer = &rxd;

	auto ret = spi_device_transmit(_spi, &t);

	ESP_ERROR_CHECK(ret);
    if (ret != ESP_OK)
    {
    	return false;
    }

    value = rxd;
	return true;
}

bool LSM6DS3::Read16Bit(Register reg, int16_t& value)
{
	return Read16Bit(reg, reinterpret_cast<uint16_t&>(value));
}

inline bool LSM6DS3::ReadData(Register reg, void* data, size_t num_bytes)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.addr = MODE_READ | static_cast<uint8_t>(reg);
	t.tx_buffer = nullptr;
	t.length = 8*num_bytes;
	t.rx_buffer = data;
	t.rxlength = t.length;
	auto ret = spi_device_transmit(_spi, &t);
    ESP_ERROR_CHECK(ret);
    return (ret == ESP_OK);
}

bool LSM6DS3::WriteRegister(Register reg, uint8_t value)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.addr = MODE_WRITE | static_cast<uint8_t>(reg);
	t.tx_buffer = &value;
	t.length = 8;
	t.rx_buffer = nullptr;
	t.rxlength = 0;
	auto ret = spi_device_transmit(_spi, &t);
    ESP_ERROR_CHECK(ret);
    return (ret == ESP_OK);
}

bool LSM6DS3::EnableConfigAccess()
{
	return WriteRegister(Register::FUNC_CFG_ACCESS, 0x80);
}

bool LSM6DS3::DisableConfigAccess()
{
	return WriteRegister(Register::FUNC_CFG_ACCESS, 0x00);
}

bool LSM6DS3::ResetFifo()
{
    // set disable fifo
    if (!WriteRegister(Register::FIFO_CTRL5, 0x00)) { return false; }

    // set FIFO ODR to 1660Hz, enable FIFO continuous mode
    if (!WriteRegister(Register::FIFO_CTRL5, 0b01000110)) { return false; }

    return true;
}
