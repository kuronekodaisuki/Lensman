// I2C

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <time.h>
#include "I2C.h"

#define DEV_I2C	"/dev/i2c-1"
const double RAD_TO_DEG = (180 / 3.14159265359);

// Constructor
I2C::I2C(char device_addr)
{
	_device_addr = device_addr;
}

void I2C::Write(char reg_addr, char data)
{
	int fd;

	if ((fd = open(DEV_I2C, O_RDWR)) >= 0)
	{
		int res;
		if ((res = ioctl(fd, I2C_SLAVE, _device_addr)) >= 0)
		{
			char buffer[2] = {reg_addr, data};
			write(fd, buffer, 2);
		}
		close(fd);
	}
}

void I2C::Write(char reg_addr, char* data, int size)
{
	int fd;

	if ((fd = open(DEV_I2C, O_RDWR)) >= 0)
	{
		int res;
		if ((res = ioctl(fd, I2C_SLAVE, _device_addr)) >= 0)
		{
			char buffer[1] = { reg_addr };
			write(fd, buffer, 1);
			write(fd, data, size);
		}
		close(fd);
	}
}

char I2C::Read(char reg_addr)
{
	int fd;
	char data[1];

	if ((fd = open(DEV_I2C, O_RDWR)) >= 0)
	{
		int res;
		if ((res = ioctl(fd, I2C_SLAVE, _device_addr)) >= 0)
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

	if ((fd = open(DEV_I2C, O_RDWR)) >= 0)
	{
		int res;
		if ((res = ioctl(fd, I2C_SLAVE, _device_addr)) >= 0)
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
MPU_6050::MPU_6050() : I2C(0x68)
{
}

char MPU_6050::WhoAmI()
{
	return Read(0x75); // WHO_AM_I
}

// Calc Roll, Pitch and Yaw
void MPU_6050::CalcRPY()
{
	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -ÉŒ to ÉŒ (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	roll = atan2(accY, accZ) * RAD_TO_DEG;
	pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

bool MPU_6050::Init()
{
	char initial[4] = {
		7, // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
		0, // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
		0, // Set Gyro Full Scale Range to Å}250deg/s
		0  // Set Accelerometer Full Scale Range to Å}2g
	};
	Write(0x19, initial, 4);
	Write(0x26, 0x06); // I2C_SLV0_REG 
	Write(0x6B, 0x01); // PLL with X axis gyroscope reference and disable sleep mode

	if (WhoAmI() != 0x68)
		return false;
	else
	{
		usleep(100); // Wait for sensor to stabilize
		accX = AccelX();
		accY = AccelY();
		accZ = AccelZ();
		gyroX = GyroX();
		gyroY = GyroY();
		gyroZ = GyroZ();
		clock_gettime(CLOCK_REALTIME, &timer);

		CalcRPY();

		kalmanX.setAngle(roll); // Set starting angle
		kalmanY.setAngle(pitch);

		return true;
	}
}

void MPU_6050::Next()
{
	accX = AccelX();
	accY = AccelY();
	accZ = AccelZ();
	gyroX = GyroX();
	gyroY = GyroY();
	gyroZ = GyroZ();

	struct timespec time;
	clock_gettime(CLOCK_REALTIME, &time);
	
	double start = timer.tv_sec + (double)timer.tv_nsec / 1000000000;
	double now = time.tv_sec + (double)time.tv_nsec / 1000000000;  
	// Calculate delta time
	double dt = now - start;
	clock_gettime(CLOCK_REALTIME, &timer);

	CalcRPY();

	double gyroXrate = gyroX / 131.0; // Convert to deg/s
	double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
		kalmanX.setAngle(roll);
	}
	else
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalAngleX) > 90)
		gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
	kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
		kalmanY.setAngle(pitch);
	}
	else
		kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalAngleY) > 90)
		gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
	kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif
}

static double toDouble(int value)
{
	if (value < 0x8000)
		return (double)value;
	else
		return (double)((int)(0x10000 - value) * -1);
}

// Read 
double MPU_6050::AccelX()
{
	int value = ReadWord(0x3B);
	return toDouble(value);
}

double MPU_6050::AccelY()
{
	int value = ReadWord(0x3D);
	return toDouble(value);
}

double MPU_6050::AccelZ()
{
	int value = ReadWord(0x3F);
	return toDouble(value);
}

int MPU_6050::GyroX()
{
	return ReadWord(0x43);
}

int MPU_6050::GyroY()
{
	return ReadWord(0x45);
}

int MPU_6050::GyroZ()
{
	return ReadWord(0x47);
}

///////////////////////////////////////////////
// AXDL345
AXDL345::AXDL345() : I2C(0x53)
{
}

bool AXDL345::Init()
{
	Write(0x2D, 0x08); // POWER_CTL 
	return true;
}

// Read 
double AXDL345::AccelX()
{
	return (ReadWord(0x32) * 3.9) / 1000.0;
}

double AXDL345::AccelY()
{
	return (ReadWord(0x34) * 3.9) / 1000.0;
}

double AXDL345::AccelZ()
{
	return (ReadWord(0x36) * 3.9) / 1000.0;
}
