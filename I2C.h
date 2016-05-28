// I2C,h
#include <stdint.h>
#include <time.h>
#include "KalmanFilter/Kalman.h"

class I2C
{
protected:
	char _device_addr;

public:
	I2C(char device_addr);

	void Write(char reg_addr, char data);
	void Write(char reg_addr, char* data, int size);
	char Read(char reg_addr);
	int  ReadWord(char reg_addr);
};

class MPU_6050 : public I2C
{
public:
	double roll, pitch, yaw;
	double kalAngleX, kalAngleY, kalAngleZ;

protected:
	double accX, accY, accZ;
	double gyroX, gyroY, gyroZ;
	struct timespec timer;
	Kalman kalmanX;
	Kalman kalmanY;
	Kalman kalmanZ;
	void CalcRPY();

public:
	MPU_6050();

	bool Init();
	char WhoAmI();

	// Read 
	double AccelX();
	double AccelY();
	double AccelZ();

	int GyroX();
	int GyroY();
	int GyroZ();

	void Next();
};

class AXDL345 : I2C
{
public:
	AXDL345();

	bool Init();

	// Read 
	double AccelX();
	double AccelY();
	double AccelZ();
};
