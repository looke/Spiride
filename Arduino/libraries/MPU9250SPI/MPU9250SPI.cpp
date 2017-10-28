// Do not remove the include below
#include "MPU9250SPI.h"

MPU9250SPI::MPU9250SPI()
{
	magSelectPin = 10;

	//ACC Original Data
	acc_X_H = 0;
	acc_X_L = 0;
	acc_X = 0;
	acc_Y_H = 0;
	acc_Y_L = 0;
	acc_Y = 0;
	acc_Z_H = 0;
	acc_Z_L = 0;
	acc_Z = 0;
	//Mag Original Data
	mag_X_H = 0;
	mag_X_L = 0;
	mag_X = 0;
	mag_Y_H = 0;
	mag_Y_L = 0;
	mag_Y = 0;
	mag_Z_H = 0;
	mag_Z_L = 0;
	mag_Z = 0;
	//Gyro Original Data
	gyro_X_H = 0;
	gyro_X_L = 0;
	gyro_X = 0;
	gyro_Y_H = 0;
	gyro_Y_L = 0;
	gyro_Y = 0;
	gyro_Z_H = 0;
	gyro_Z_L = 0;
	gyro_Z = 0;

	acc_X_filtered = 0.0;
	acc_Y_filtered = 0.0;
	acc_Z_filtered = 0.0;

	gyro_X_filtered = 0.0;
	gyro_Y_filtered = 0.0;
	gyro_Z_filtered = 0.0;

	mag_X_filtered = 0.0;
	mag_Y_filtered = 0.0;
	mag_Z_filtered = 0.0;
};

void MPU9250SPI::init(byte input_magSelectPin)
{
	magSelectPin = input_magSelectPin;
	//SPI∂¡»°≥ı ºªØ
	pinMode(magSelectPin, OUTPUT);
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV16);
	SPI.setDataMode(SPI_MODE0);
};

void MPU9250SPI::readRegisterAndPrint(byte regAdd, String regName)
{
	digitalWrite(magSelectPin, LOW);
	SPI.transfer(regAdd|MPU9250_SPI_READ_MASK);
	byte regValue = SPI.transfer(0xFF);
	digitalWrite(magSelectPin, HIGH);
	Serial.print(regName);
	Serial.print(":");
	Serial.println(regValue,BIN);
}

byte MPU9250SPI::readRegister(byte regAdd)
{
	digitalWrite(magSelectPin, LOW);
	SPI.transfer(regAdd|MPU9250_SPI_READ_MASK);
	byte regValue = SPI.transfer(0xFF);
	digitalWrite(magSelectPin, HIGH);
	return regValue;
};

byte MPU9250SPI::read_WHO_AM_I_MPU9250()
{
	return readRegister(MPU9250_REG_WHO_AM_I);
};


void MPU9250SPI::writeRegister(byte regAdd, byte value)
{
	digitalWrite(magSelectPin, LOW);
	SPI.transfer(regAdd);
	SPI.transfer(value);
	digitalWrite(magSelectPin, HIGH);
}
/*
 * Setup MPU9250 to make Mag data automatically read form AK8963
 */
void MPU9250SPI::setupMPU9250_MagRead()
{
	testConnectionToMPU9250();

	//Clock source auto select
	writeRegister(MPU9250_REG_PWR_MGMT_1, 0x01);
	readRegisterAndPrint(MPU9250_REG_PWR_MGMT_1, "MPU9250_REG_PWR_MGMT_1");

	//Enable Acc&Gyro
	writeRegister(MPU9250_REG_PWR_MGMT_2, 0x00);
	readRegisterAndPrint(MPU9250_REG_PWR_MGMT_2, "MPU9250_REG_PWR_MGMT_2");

	//Bandwidth 41Hz/Sample Rate;1kHz
	writeRegister(MPU9250_REG_CONFIG, 0x03);
	readRegisterAndPrint(MPU9250_REG_CONFIG, "MPU9250_REG_CONFIG");

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeRegister(MPU9250_REG_SMPLRT_DIV, 0x04);
	readRegisterAndPrint(MPU9250_REG_SMPLRT_DIV, "MPU9250_REG_SMPLRT_DIV");

	//Gyro Full Scale:250dps{0x00}/F_choiceb:0x00
	writeRegister(MPU9250_REG_GYRO_CONFIG, 0x00);
	readRegisterAndPrint(MPU9250_REG_GYRO_CONFIG, "MPU9250_REG_GYRO_CONFIG");

	//XYZ SelfTest OFF/ Full Scale:2G{0x00}
	writeRegister(MPU9250_REG_ACCEL_CONFIG, 0x00);
	readRegisterAndPrint(MPU9250_REG_ACCEL_CONFIG, "MPU9250_REG_ACCEL_CONFIG");

	//DLPF 41Hz
	writeRegister(MPU9250_REG_ACCEL_CONFIG2, 0x03);
	readRegisterAndPrint(MPU9250_REG_ACCEL_CONFIG2, "MPU9250_REG_ACCEL_CONFIG2");

	//check interrupt
	readRegisterAndPrint(MPU9250_REG_INT_PIN_CFG, "MPU9250_REG_INT_PIN_CFG");
	readRegisterAndPrint(MPU9250_REG_INT_ENABLE, "MPU9250_REG_INT_ENABLE");
	readRegisterAndPrint(MPU9250_REG_INT_STATUS, "MPU9250_REG_INT_STATUS");


	Serial.println("----------I2C Master Config---------------");
	//I2C Master:Enable
	writeRegister(MPU9250_REG_USER_CTRL, 0x20);
	readRegisterAndPrint(MPU9250_REG_USER_CTRL, "MPU9250_REG_USER_CTRL");
	//I2C Clock:400kHz 0xD/ I2C Master°Øs transition from one slave read to the next slave read. If 0, there is a restart between reads. If 1, there is a stop between reads.
	writeRegister(MPU9250_REG_I2C_MST_CTRL, 0x1D);
	readRegisterAndPrint(MPU9250_REG_I2C_MST_CTRL, "MPU9250_REG_I2C_MST_CTRL");
	readRegisterAndPrint(MPU9250_REG_I2C_MST_DELAY_CTRL, "MPU9250_REG_I2C_MST_DELAY_CTRL");

	Serial.println("----------I2C Slave0 ak8963 Config---------------");

	//First Reset AK8963
	writeRegister(MPU9250_REG_I2C_SLV0_ADDR, MPU9250_I2C_MAG_ADDRESS);
	//AK8963 Reg address to write MAG_REG_CNTL1
	writeRegister(MPU9250_REG_I2C_SLV0_REG, MPU9250_MAG_REG_CNTL2);
	//Data to write to AK8963-  SRST:RESET{0x01}
	writeRegister(MPU9250_REG_I2C_SLV0_DO,0x01);
	//I2C_Slave0 Start writing 1 byte
	writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x81);

	Serial.println("AK8963 reseting...");
	//wait for reset
	delay(3000);
	//check connection
	testConnectionToAK8963();

	//reset slave0 addr for write
	writeRegister(MPU9250_REG_I2C_SLV0_ADDR, MPU9250_I2C_MAG_ADDRESS);
	//AK8963 Reg address to write MAG_REG_CNTL1
	writeRegister(MPU9250_REG_I2C_SLV0_REG, MPU9250_MAG_REG_CNTL1);
	//Data to write to AK8963-  Output bit setting:16bit{0x1} Operating Mode: Continuous measurement mode 2{0x0110}
	writeRegister(MPU9250_REG_I2C_SLV0_DO,0x16);
	//I2C_Slave0 Start writing 1 byte
	writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x81);

	delay(100);

	//check CNTL1
	writeRegister(MPU9250_REG_I2C_SLV0_ADDR, MPU9250_I2C_MAG_ADDRESS|MPU9250_SPI_READ_MASK);
	//AK8963 Reg address to read MAG_REG_CNTL1
	writeRegister(MPU9250_REG_I2C_SLV0_REG, MPU9250_MAG_REG_CNTL1);
	//I2C_Slave0 Start reading 1 byte
	writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x81);
	delay(100);
	//writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x00);
	readRegisterAndPrint(MPU9250_REG_EXT_SENS_DATA_00, "MPU9250_MAG_REG_CNTL1");

	//Set Slove0 to read Mag data/ regAddress from ST1 to ST2 8 bytes in total
	writeRegister(MPU9250_REG_I2C_SLV0_REG, MPU9250_MAG_REG_ST1);
	//Start reading
	writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x88);

	Serial.println("AK8963 setting finish.");
	delay(10);

	readRegisterAndPrint(MPU9250_REG_EXT_SENS_DATA_00, "ST1");
	readRegisterAndPrint(MPU9250_REG_EXT_SENS_DATA_01, "XL");
	readRegisterAndPrint(MPU9250_REG_EXT_SENS_DATA_02, "XH");
	Serial.println("***********************************************************");
};

void MPU9250SPI::testConnectionToAK8963()
{
	//Set Slave0 8963 address in Read Mode
	writeRegister(MPU9250_REG_I2C_SLV0_ADDR, MPU9250_I2C_MAG_ADDRESS|MPU9250_SPI_READ_MASK);
	//AK8963 Reg address to read MAG_REG_WAI
	writeRegister(MPU9250_REG_I2C_SLV0_REG, MPU9250_MAG_REG_WAI);
	//I2C_Slave0 Start reading 1 byte
	writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x81);
	delay(10);
	//writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x00);
	byte regValue = readRegister(MPU9250_REG_EXT_SENS_DATA_00);
	Serial.println("AK8963 Who Am I:");
	Serial.println(regValue, BIN);
	if(regValue == MPU9250_MAG_WAI_VALUE)
	{
		Serial.println("AK8963 Online!");
	}
	else
	{
		Serial.println("AK8963 Offline!");
	}
}

void MPU9250SPI::testConnectionToMPU9250()
{
	byte whoami = read_WHO_AM_I_MPU9250();
	if(whoami == MPU9250_REG_WHO_AM_I_VALUE)
	{
		Serial.println("MPU9250 Online!");
	}
	else
	{
	    Serial.println("MPU9250 Offline!");
	}
};

void MPU9250SPI::read_Gyro_Data()
{
  digitalWrite(magSelectPin, LOW);
  SPI.transfer(MPU9250_REG_GYRO_XOUT_H|MPU9250_SPI_READ_MASK);
  gyro_X_H = SPI.transfer(0xFF);
  gyro_X_L = SPI.transfer(0xFF);
  gyro_Y_H = SPI.transfer(0xFF);
  gyro_Y_L = SPI.transfer(0xFF);
  gyro_Z_H = SPI.transfer(0xFF);
  gyro_Z_L = SPI.transfer(0xFF);
  digitalWrite(magSelectPin, HIGH);

  gyro_X = (gyro_X_H << 8) | gyro_X_L;
  gyro_Y = (gyro_Y_H << 8) | gyro_Y_L;
  gyro_Z = (gyro_Z_H << 8) | gyro_Z_L;
}

void MPU9250SPI::read_Gyro_Data_Filtered()
{
	read_Gyro_Data();
	gyro_X_filtered = gyro_x_LPF.apply(gyro_X);
	gyro_Y_filtered = gyro_y_LPF.apply(gyro_Y);
	gyro_Z_filtered = gyro_z_LPF.apply(gyro_Z);
};

void MPU9250SPI::read_ACC_Data()
{
  digitalWrite(magSelectPin, LOW);
  SPI.transfer(MPU9250_REG_ACCEL_XOUT_H|MPU9250_SPI_READ_MASK);
  acc_X_H = SPI.transfer(0xFF);
  acc_X_L = SPI.transfer(0xFF);

  acc_Y_H = SPI.transfer(0xFF);
  acc_Y_L = SPI.transfer(0xFF);

  acc_Z_H = SPI.transfer(0xFF);
  acc_Z_L = SPI.transfer(0xFF);
  digitalWrite(magSelectPin, HIGH);

  acc_X = (acc_X_H << 8) | acc_X_L;
  acc_Y = (acc_Y_H << 8) | acc_Y_L;
  acc_Z = (acc_Z_H << 8) | acc_Z_L;
}

void MPU9250SPI::read_ACC_Data_Filtered()
{
	read_ACC_Data();
	acc_X_filtered = acc_x_LPF.apply(acc_X);
	acc_Y_filtered = acc_y_LPF.apply(acc_Y);
	acc_Z_filtered = acc_z_LPF.apply(acc_Z);
};

void MPU9250SPI::read_Mag_Data()
{
  //Read Mag data from AK8930 by 9250 I2C Master
  digitalWrite(magSelectPin, LOW);
  SPI.transfer(MPU9250_REG_EXT_SENS_DATA_00|MPU9250_SPI_READ_MASK);
  byte ST1 = SPI.transfer(0xFF);
  mag_X_L = SPI.transfer(0xFF);
  mag_X_H = SPI.transfer(0xFF);
  mag_Y_L = SPI.transfer(0xFF);
  mag_Y_H = SPI.transfer(0xFF);
  mag_Z_L = SPI.transfer(0xFF);
  mag_Z_H = SPI.transfer(0xFF);
  byte ST2 = SPI.transfer(0xFF);
  digitalWrite(magSelectPin, HIGH);

  mag_X = (mag_X_H << 8) | mag_X_L;
  mag_Y = (mag_Y_H << 8) | mag_Y_L;
  mag_Z = (mag_Z_H << 8) | mag_Z_L;
}

void MPU9250SPI::read_Mag_Data_Filtered()
{
	read_Mag_Data();
	mag_X_filtered = mag_x_LPF.apply(mag_X);
	mag_Y_filtered = mag_y_LPF.apply(mag_Y);
	mag_Z_filtered = mag_z_LPF.apply(mag_Z);
};

void MPU9250SPI::reFreshLPF()
{
	mag_x_LPF.resetFilter();
	mag_y_LPF.resetFilter();
	mag_z_LPF.resetFilter();

	acc_x_LPF.resetFilter();
	acc_y_LPF.resetFilter();
	acc_z_LPF.resetFilter();

	gyro_x_LPF.resetFilter();
	gyro_y_LPF.resetFilter();
	gyro_z_LPF.resetFilter();
};
