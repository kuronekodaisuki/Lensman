// I2C,h

public class I2C
{
protected:
	char _device_addr;

public:
	I2C(char device_addr);

	void Write(char reg_addr, char data);
	char Read(char reg_addr);
	int  ReadWord(char reg_addr);
};

public class MPU_6050 : I2C
{
public:
	MPU_6050() : (0x68);

	void Init();
	char WhoAmI();

	// Read 
	double AccelX();
	double AccelY();
	double AccelZ();

	int DyroX();
	int DyroY();
	int DyroZ();
};