// I2C

#include <pthread.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>

#include "I2C.h"

#define DEV_I2C	"/dev/i2c-1"

// Constructor
I2C::I2C(char device_addr)
{
	_device_addr = device_addr;
}

void I2C::Write(char reg_addr, char data)
{
	int fd;

	if ((fd = open(DEV_I2C, O_RDWR) >= 0)
	{
		int res;
		if ((res = ioctl(fd, I2C_SLAVE, _device_addr) >= 0)
		{
			char buffer[2] = {reg_addr, data};
			write(fd, buffer, 2);
		}
		close(fd);
	}
}

char I2C::Read(char reg_addr)
{
	int fd;
	char data[1];

	if ((fd = open(DEV_I2C, O_RDWR) >= 0)
	{
		int res;
		if ((res = ioctl(fd, I2C_SLAVE, _device_addr) >= 0)
		{
			char buffer[1] = { reg_addr };
			write(fd, buffer, 1);
			read(fd, data, 1);
		}
		close(fd);
	}
	return data[0];
}

int  I2C::ReadWord(char reg_addr)
{
	int fd;
	char data[2];

	if ((fd = open(DEV_I2C, O_RDWR) >= 0)
	{
		int res;
		if ((res = ioctl(fd, I2C_SLAVE, _device_addr) >= 0)
		{
			char buffer[1] = { reg_addr };
			write(fd, buffer, 1);
			read(fd, data, 2);
		}
		close(fd);
	}
	return (int)data[0] * 256 + data[1];
}

///////////////////////////////////////////////
// MPU-6050
MPU_6050::MPU_6050()
{
}

char MPU_6050::WhoAmI()
{
	return Read(0x75); // WHO_AM_I
}

void MPU_6050::Init()
{
	Write(0x26, 0x06); // I2C_SLV0_REG 
	Write(0x6B, 0x00); // PWR_MGMT_1
}

static double toDouble(int value)
{
	if (value < 0x8000)
		return (double)value;
	else
		return (double)((int)(0x10000 - value) * -1);
}

// Read 
double AccelX()
{
	int value = ReadWord(0x3B);
	return toDouble(value);
}

double AccelY()
{
	int value = ReadWord(0x3D);
	return toDouble(value);
}

double AccelZ()
{
	int value = ReadWord(0x3F);
	return toDouble(value);
}

int DyroX()
{
	return ReadWord(0x43);
}

int DyroY()
{
	return ReadWord(0x45);
}

int DyroZ()
{
	return ReadWord(0x47);
}