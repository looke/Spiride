#include <math.h>
#include <SPI.h>
#include "MPU9250.h"
const int MagSelectPin = 10;

boolean isMagNormalized = false;
int Mag_X_Max = 270;
int Mag_X_Min = -296;
int Mag_X_Range = 566;
int Mag_X_ZP = 283;

int Mag_Y_Max = 396;
int Mag_Y_Min = -158;
int Mag_Y_Range = 554;
int Mag_Y_ZP = 277;

int Mag_Z_Max = 175;
int Mag_Z_Min = -389;
int Mag_Z_Range = 564;
int Mag_Z_ZP = 282;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //SPI读取初始化
  pinMode(MagSelectPin, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setDataMode(SPI_MODE0);

  delay(200);
  setupMag();
  delay(1000);

/*
  Serial.println("Mag Normalization X Start.");
  Mag_Normalization();
  showNormalizationResult();

  Serial.println("Mag Normalization Y Start.");
  Mag_Normalization();
  showNormalizationResult();

  Serial.println("Mag Normalization Z Start.");
  Mag_Normalization();
  showNormalizationResult();
*/
}

void loop() {
  // put your main code here, to run repeatedly:
  //read_WHO_AM_I();

  //Read Acc Data from 9250 by SPI
  //Mag_Z_Normalization();

  //read_Gyro_Data();
  read_Mag_Data();
  //Serial.println("***********************************************************");
  
  delay(10);
}

void setupMag()
{
  read_WHO_AM_I();

  writeRegister(MPU9250_REG_PWR_MGMT_1, 0x01); //Clock source auto select
  readRegister(MPU9250_REG_PWR_MGMT_1, "MPU9250_REG_PWR_MGMT_1");

  writeRegister(MPU9250_REG_PWR_MGMT_2, 0x00); //Enable Acc&Gyro
  readRegister(MPU9250_REG_PWR_MGMT_2, "MPU9250_REG_PWR_MGMT_2");

  byte DLPF_GYRO_41HZ = 0x03;
  writeRegister(MPU9250_REG_CONFIG, DLPF_GYRO_41HZ); //Bandwidth 41Hz/Sample Rate;1kHz
  readRegister(MPU9250_REG_CONFIG, "MPU9250_REG_CONFIG");
  
  byte sampleRateDivider = 0x04;
  writeRegister(MPU9250_REG_SMPLRT_DIV, sampleRateDivider); // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  readRegister(MPU9250_REG_SMPLRT_DIV, "MPU9250_REG_SMPLRT_DIV");
  
  writeRegister(MPU9250_REG_GYRO_CONFIG, 0x00); //Gyro Full Scale:250dps{0x00}/F_choiceb:0x00
  readRegister(MPU9250_REG_GYRO_CONFIG, "MPU9250_REG_GYRO_CONFIG");

  //XYZ SelfTest OFF/ Full Scale:2G{0x00}
  writeRegister(MPU9250_REG_ACCEL_CONFIG, 0x00);
  readRegister(MPU9250_REG_ACCEL_CONFIG, "MPU9250_REG_ACCEL_CONFIG");
  //DLPF 41Hz
  writeRegister(MPU9250_REG_ACCEL_CONFIG2, 0x03);
  readRegister(MPU9250_REG_ACCEL_CONFIG2, "MPU9250_REG_ACCEL_CONFIG2");

  readRegister(MPU9250_REG_INT_PIN_CFG, "MPU9250_REG_INT_PIN_CFG");
  readRegister(MPU9250_REG_INT_ENABLE, "MPU9250_REG_INT_ENABLE");
  readRegister(MPU9250_REG_INT_STATUS, "MPU9250_REG_INT_STATUS");
  
  Serial.println("----------I2C Master Config---------------");
  //I2C Master:Enable
  writeRegister(MPU9250_REG_USER_CTRL, 0x20);
  readRegister(MPU9250_REG_USER_CTRL, "MPU9250_REG_USER_CTRL");
  //I2C Clock:400kHz 0xD/ I2C Master’s transition from one slave read to the next slave read. If 0, there is a restart between reads. If 1, there is a stop between reads.
  writeRegister(MPU9250_REG_I2C_MST_CTRL, 0x1D);
  readRegister(MPU9250_REG_I2C_MST_CTRL, "MPU9250_REG_I2C_MST_CTRL");
  readRegister(MPU9250_REG_I2C_MST_DELAY_CTRL, "MPU9250_REG_I2C_MST_DELAY_CTRL");

  Serial.println("----------I2C Slave0 Config---------------");
  
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
  
  delay(100);
  //reset slave0 addr for write
  writeRegister(MPU9250_REG_I2C_SLV0_ADDR, MPU9250_I2C_MAG_ADDRESS);
  //AK8963 Reg address to write MAG_REG_CNTL1
  writeRegister(MPU9250_REG_I2C_SLV0_REG, MPU9250_MAG_REG_CNTL1);
  //Data to write to AK8963-  Output bit setting:16bit{0x1} Operating Mode: Continuous measurement mode 2{0x0110}
  writeRegister(MPU9250_REG_I2C_SLV0_DO,0x16);
  //I2C_Slave0 Start writing 1 byte
  writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x81);
  delay(100);
  //readRegister(MPU9250_REG_EXT_SENS_DATA_00, "MPU9250_REG_EXT_SENS_DATA_00");

  
  
  //check CNTL1
  writeRegister(MPU9250_REG_I2C_SLV0_ADDR, MPU9250_I2C_MAG_ADDRESS|MPU9250_SPI_READ_MASK);
  //AK8963 Reg address to read MAG_REG_CNTL1
  writeRegister(MPU9250_REG_I2C_SLV0_REG, MPU9250_MAG_REG_CNTL1);
  //I2C_Slave0 Start reading 1 byte
  writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x81);
  delay(100);
  //writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x00);
  readRegister(MPU9250_REG_EXT_SENS_DATA_00, "MPU9250_REG_EXT_SENS_DATA_00");
  
  //Set Slove0 to read Mag data/ regAddress from ST1 to ST2 8 bytes in total
  writeRegister(MPU9250_REG_I2C_SLV0_REG, MPU9250_MAG_REG_ST1);
  //Start reading
  writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x88);
  Serial.println("AK8963 setting finish.");
  delay(10);
  readRegister(MPU9250_REG_EXT_SENS_DATA_00, "ST1");
  readRegister(MPU9250_REG_EXT_SENS_DATA_01, "XL");
  readRegister(MPU9250_REG_EXT_SENS_DATA_02, "XH");
  Serial.println("***********************************************************");
  //readRegister(MPU9250_REG_I2C_SLV0_CTRL, "MPU9250_REG_I2C_SLV0_CTRL");

  //I2C_Slave0 Start reading
  //writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x81);
  //readRegister(MPU9250_REG_I2C_SLV0_CTRL, "MPU9250_REG_I2C_SLV0_CTRL");
  //delay(10);
  //I2C_Slave0 Stop reading
  //writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x00); 
  //readRegister(MPU9250_REG_I2C_SLV0_CTRL, "MPU9250_REG_I2C_SLV0_CTRL");


  
  //delay(10);
  //set slove0 addr to write
  //writeRegister(MPU9250_REG_I2C_SLV0_ADDR, MPU9250_I2C_MAG_ADDRESS);
  //writeRegister(MPU9250_REG_I2C_SLV0_DO,0x01);
  
  //delay(1000);
  
 // writeRegister(MPU9250_REG_I2C_SLV0_ADDR, MPU9250_I2C_MAG_ADDRESS|MPU9250_SPI_READ_MASK);
  
  //readRegister(MPU9250_REG_EXT_SENS_DATA_00, "MPU9250_REG_EXT_SENS_DATA_00");
  //readRegister(MPU9250_REG_EXT_SENS_DATA_01, "MPU9250_REG_EXT_SENS_DATA_01");
  /*
  byte sampleRateDivider = 0x04;
  writeRegister(MPU9250_REG_SMPLRT_DIV, sampleRateDivider);
  readRegister(MPU9250_REG_SMPLRT_DIV, "MPU9250_REG_SMPLRT_DIV");

  byte DLPF_GYRO_41HZ = 0x03;
  writeRegister(MPU9250_REG_CONFIG, DLPF_GYRO_41HZ);
  readRegister(MPU9250_REG_CONFIG, "MPU9250_REG_CONFIG");

  //XYZ SelfTest OFF/ Full Scale:2G
  writeRegister(MPU9250_REG_ACCEL_CONFIG, 0x00);
  //DLPF 41Hz
  writeRegister(MPU9250_REG_ACCEL_CONFIG2, 0x03);

  //I2C Master:Enable
  writeRegister(MPU9250_REG_USER_CTRL, 0x20);
  //I2C Clock:400kHz 0xD/ I2C Master’s transition from one slave read to the next slave read. If 0, there is a restart between reads. If 1, there is a stop between reads.
  writeRegister(MPU9250_REG_I2C_MST_CTRL, 0x1D);

  -----------------------Slave0 as AK8963 Config-----------------------------------------
  //I2C_SLV0_RNW:read{0x1} I2C Slave0 Addr:MPU9250_I2C_MAG_ADDRESS{0x0C} 
  writeRegister(MPU9250_REG_I2C_SLV0_ADDR, MPU9250_I2C_MAG_ADDRESS|MPU9250_SPI_READ_MASK);
  //I2C_SLV0_RNW:write{0x0} I2C Slave0 Addr:MPU9250_I2C_MAG_ADDRESS{0x0C} 
  writeRegister(MPU9250_REG_I2C_SLV0_ADDR, MPU9250_I2C_MAG_ADDRESS);
  
  //I2C_SLV0_Reg:Who Am I for AK8963{0x00}
  writeRegister(MPU9250_REG_I2C_SLV0_REG,0x00);

  //I2C_Slave0 Start reading 1Byte
  writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x81);

  //I2C_Slave0 Stop reading
  writeRegister(MPU9250_REG_I2C_SLV0_CTRL,0x00); 

  //I2C_Slave0 Data Out
  writeRegister(MPU9250_REG_I2C_SLV0_DO,0x01); 
  */
}

void readRegister(byte regAdd, String regName)
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(regAdd|MPU9250_SPI_READ_MASK);
  byte regValue = SPI.transfer(0xFF);
  digitalWrite(MagSelectPin, HIGH);
  Serial.print(regName);
  Serial.print(":");
  Serial.println(regValue,BIN);
}
byte readRegister(byte regAdd)
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(regAdd|MPU9250_SPI_READ_MASK);
  byte regValue = SPI.transfer(0xFF);
  digitalWrite(MagSelectPin, HIGH);
  return regValue;
}

void writeRegister(byte regAdd, byte value)
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(regAdd);
  SPI.transfer(value);
  digitalWrite(MagSelectPin, HIGH);
}

void read_WHO_AM_I()
{
  readRegister(MPU9250_REG_WHO_AM_I, "MPU9250_REG_WHO_AM_I");
}

void testConnectionToAK8963()
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

void read_ACC_Data()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(MPU9250_REG_ACCEL_XOUT_H|MPU9250_SPI_READ_MASK);
  byte xh = SPI.transfer(0xFF);
  byte xl = SPI.transfer(0xFF);
  byte yh = SPI.transfer(0xFF);
  byte yl = SPI.transfer(0xFF);
  byte zh = SPI.transfer(0xFF);
  byte zl = SPI.transfer(0xFF);
  digitalWrite(MagSelectPin, HIGH);
  
  Serial.print("ACC XH:");
  Serial.print(xh,BIN);
  Serial.print("\t");
  Serial.print("ACC XL:");
  Serial.println(xl,BIN);

  Serial.print("ACC YH:");
  Serial.print(yh,BIN);
  Serial.print("\t");
  Serial.print("ACC YL:");
  Serial.println(yl,BIN);

  Serial.print("ACC ZH:");
  Serial.print(zh,BIN);
  Serial.print("\t");
  Serial.print("ACC ZL:");
  Serial.println(zl,BIN);
}

void read_Gyro_Data()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(MPU9250_REG_GYRO_XOUT_H|MPU9250_SPI_READ_MASK);
  byte xh = SPI.transfer(0xFF);
  byte xl = SPI.transfer(0xFF);
  byte yh = SPI.transfer(0xFF);
  byte yl = SPI.transfer(0xFF);
  byte zh = SPI.transfer(0xFF);
  byte zl = SPI.transfer(0xFF);
  digitalWrite(MagSelectPin, HIGH);
  
  Serial.print("Gyro XH:");
  Serial.print(xh,BIN);
  Serial.print("\t");
  Serial.print("Gyro XL:");
  Serial.println(xl,BIN);

  Serial.print("Gyro YH:");
  Serial.print(yh,BIN);
  Serial.print("\t");
  Serial.print("Gyro YL:");
  Serial.println(yl,BIN);

  Serial.print("Gyro ZH:");
  Serial.print(zh,BIN);
  Serial.print("\t");
  Serial.print("Gyro ZL:");
  Serial.println(zl,BIN);
}

void read_Mag_Data()
{
  //Read Mag data from AK8930 by 9250 I2C Master
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(MPU9250_REG_EXT_SENS_DATA_00|MPU9250_SPI_READ_MASK);
  byte ST1 = SPI.transfer(0xFF);
  byte X_LSB = SPI.transfer(0xFF);
  byte X_MSB = SPI.transfer(0xFF);
  byte Y_LSB = SPI.transfer(0xFF);
  byte Y_MSB = SPI.transfer(0xFF);
  byte Z_LSB = SPI.transfer(0xFF);
  byte Z_MSB = SPI.transfer(0xFF);
  byte ST2 = SPI.transfer(0xFF);
  digitalWrite(MagSelectPin, HIGH);
  
  short int x = ((X_MSB << 8) | X_LSB);
  short int y = ((Y_MSB << 8) | Y_LSB);
  short int z = ((Z_MSB << 8) | Z_LSB);

  double x_gauss = (double)x*0.0015;
  double y_gauss = (double)y*0.0015;
  double z_gauss = (double)z*0.0015;

  float x_normal = x-Mag_X_Min;
  x_normal = x_normal/Mag_X_ZP -1;
  
  float y_normal = y-Mag_Y_Min;
  y_normal = y_normal/Mag_Y_ZP -1;
  
  float z_normal = z-Mag_Z_Min;
  z_normal = z_normal/Mag_Z_ZP -1;
  
  //Read Acc for reference
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(MPU9250_REG_ACCEL_XOUT_H|MPU9250_SPI_READ_MASK);
  byte acc_xh = SPI.transfer(0xFF);
  byte acc_xl = SPI.transfer(0xFF);
  byte acc_yh = SPI.transfer(0xFF);
  byte acc_yl = SPI.transfer(0xFF);
  byte acc_zh = SPI.transfer(0xFF);
  byte acc_zl = SPI.transfer(0xFF);
  digitalWrite(MagSelectPin, HIGH);
  
  short int acc_x = ((acc_xh << 8) | acc_xl);
  short int acc_y = ((acc_yh << 8) | acc_yl);
  short int acc_z = ((acc_zh << 8) | acc_zl);
  

  
  Serial.print("Mag East:");
  Serial.print(y_normal);
  Serial.print("\t");
  

  Serial.print("Mag North:");
  Serial.print(x_normal);
  Serial.print("\t");
  

  Serial.print("Mag Sky:");
  Serial.print(z_normal);
  Serial.print("\t");

  Serial.print("X acc:");
  Serial.print(acc_x);
  Serial.print("\t");

  Serial.print("Y acc:");
  Serial.print(acc_y);
  Serial.print("\t");

  Serial.print("Z acc:");
  Serial.println(acc_z);
  
}

void Mag_X_Normalization()
{
    int error_x_num = 0;
    //Read Mag data from AK8930 by 9250 I2C Master
    digitalWrite(MagSelectPin, LOW);
    SPI.transfer(MPU9250_REG_EXT_SENS_DATA_00|MPU9250_SPI_READ_MASK);
    byte ST1 = SPI.transfer(0xFF);
    byte X_LSB = SPI.transfer(0xFF);
    byte X_MSB = SPI.transfer(0xFF);
    byte Y_LSB = SPI.transfer(0xFF);
    byte Y_MSB = SPI.transfer(0xFF);
    byte Z_LSB = SPI.transfer(0xFF);
    byte Z_MSB = SPI.transfer(0xFF);
    byte ST2 = SPI.transfer(0xFF);
    digitalWrite(MagSelectPin, HIGH);

    short int x = ((X_MSB << 8) | X_LSB);
    short int y = ((Y_MSB << 8) | Y_LSB);
    short int z = ((Z_MSB << 8) | Z_LSB);
    if(x > 400)
    {
      x = 0;
      error_x_num++;
    }

    if(x < -400)
    {
      x = 0;
      error_x_num++;
    }
    
    if(x > Mag_X_Max)
    {
      Mag_X_Max = x;
    }
    if(x < Mag_X_Min)
    {
      Mag_X_Min = x;
    }
    
    //Read ACC data for Reference
    digitalWrite(MagSelectPin, LOW);
    SPI.transfer(MPU9250_REG_ACCEL_XOUT_H|MPU9250_SPI_READ_MASK);
    byte acc_xh = SPI.transfer(0xFF);
    byte acc_xl = SPI.transfer(0xFF);
    byte acc_yh = SPI.transfer(0xFF);
    byte acc_yl = SPI.transfer(0xFF);
    digitalWrite(MagSelectPin, HIGH);
    short int acc_x = ((acc_xh << 8) | acc_xl);
    short int acc_y = ((acc_yh << 8) | acc_yl);

    Serial.print("ACC_X:");
    Serial.print("\t");
    Serial.print(acc_x);
    Serial.print("\t");
    Serial.print("ACC_Y:");
    Serial.print("\t");
    Serial.print(acc_y);
    Serial.print("\t");
    Serial.print("X Max:");
    Serial.print(Mag_X_Max);
    Serial.print("\t");
    Serial.print("X Min:");
    Serial.print(Mag_X_Min);
    Serial.print("\t");
    Serial.print("X ERROR NUMBER:");
    Serial.println(error_x_num);
}

void Mag_Y_Normalization()
{
    int error_y_num = 0;
    //Read Mag data from AK8930 by 9250 I2C Master
    digitalWrite(MagSelectPin, LOW);
    SPI.transfer(MPU9250_REG_EXT_SENS_DATA_00|MPU9250_SPI_READ_MASK);
    byte ST1 = SPI.transfer(0xFF);
    byte X_LSB = SPI.transfer(0xFF);
    byte X_MSB = SPI.transfer(0xFF);
    byte Y_LSB = SPI.transfer(0xFF);
    byte Y_MSB = SPI.transfer(0xFF);
    byte Z_LSB = SPI.transfer(0xFF);
    byte Z_MSB = SPI.transfer(0xFF);
    byte ST2 = SPI.transfer(0xFF);
    digitalWrite(MagSelectPin, HIGH);

    short int x = ((X_MSB << 8) | X_LSB);
    short int y = ((Y_MSB << 8) | Y_LSB);
    short int z = ((Z_MSB << 8) | Z_LSB);
    if(y > 450)
    {
      y = 0;
      error_y_num++;
    }

    if(y < -400)
    {
      y = 0;
      error_y_num++;
    }
    
    if(y > Mag_Y_Max)
    {
      Mag_Y_Max = y;
    }
    if(y < Mag_Y_Min)
    {
      Mag_Y_Min = y;
    }
    
    //Read ACC data for Reference
    digitalWrite(MagSelectPin, LOW);
    SPI.transfer(MPU9250_REG_ACCEL_XOUT_H|MPU9250_SPI_READ_MASK);
    byte acc_xh = SPI.transfer(0xFF);
    byte acc_xl = SPI.transfer(0xFF);
    byte acc_yh = SPI.transfer(0xFF);
    byte acc_yl = SPI.transfer(0xFF);
    digitalWrite(MagSelectPin, HIGH);
    short int acc_x = ((acc_xh << 8) | acc_xl);
    short int acc_y = ((acc_yh << 8) | acc_yl);

    Serial.print("ACC_X:");
    Serial.print("\t");
    Serial.print(acc_x);
    Serial.print("\t");
    Serial.print("ACC_Y:");
    Serial.print("\t");
    Serial.print(acc_y);
    Serial.print("\t");
    Serial.print("Y Max:");
    Serial.print(Mag_Y_Max);
    Serial.print("\t");
    Serial.print("Y Min:");
    Serial.print(Mag_Y_Min);
    Serial.print("\t");
    Serial.print("Y ERROR NUMBER:");
    Serial.println(error_y_num);
}

void Mag_Z_Normalization()
{
    int error_z_num = 0;
    //Read Mag data from AK8930 by 9250 I2C Master
    digitalWrite(MagSelectPin, LOW);
    SPI.transfer(MPU9250_REG_EXT_SENS_DATA_00|MPU9250_SPI_READ_MASK);
    byte ST1 = SPI.transfer(0xFF);
    byte X_LSB = SPI.transfer(0xFF);
    byte X_MSB = SPI.transfer(0xFF);
    byte Y_LSB = SPI.transfer(0xFF);
    byte Y_MSB = SPI.transfer(0xFF);
    byte Z_LSB = SPI.transfer(0xFF);
    byte Z_MSB = SPI.transfer(0xFF);
    byte ST2 = SPI.transfer(0xFF);
    digitalWrite(MagSelectPin, HIGH);

    short int x = ((X_MSB << 8) | X_LSB);
    short int y = ((Y_MSB << 8) | Y_LSB);
    short int z = ((Z_MSB << 8) | Z_LSB);
    if(z > 400)
    {
      z = 0;
      error_z_num++;
    }

    if(z < -450)
    {
      z = 0;
      error_z_num++;
    }
    
    if(z > Mag_Z_Max)
    {
      Mag_Z_Max = z;
    }
    if(z < Mag_Z_Min)
    {
      Mag_Z_Min = z;
    }
    
    //Read ACC data for Reference
    digitalWrite(MagSelectPin, LOW);
    SPI.transfer(MPU9250_REG_ACCEL_XOUT_H|MPU9250_SPI_READ_MASK);
    byte acc_xh = SPI.transfer(0xFF);
    byte acc_xl = SPI.transfer(0xFF);
    byte acc_yh = SPI.transfer(0xFF);
    byte acc_yl = SPI.transfer(0xFF);
    digitalWrite(MagSelectPin, HIGH);
    short int acc_x = ((acc_xh << 8) | acc_xl);
    short int acc_y = ((acc_yh << 8) | acc_yl);

    Serial.print("ACC_X:");
    Serial.print("\t");
    Serial.print(acc_x);
    Serial.print("\t");
    Serial.print("ACC_Y:");
    Serial.print("\t");
    Serial.print(acc_y);
    Serial.print("\t");
    Serial.print("Z Max:");
    Serial.print(Mag_Z_Max);
    Serial.print("\t");
    Serial.print("Z Min:");
    Serial.print(Mag_Z_Min);
    Serial.print("\t");
    Serial.print("Z ERROR NUMBER:");
    Serial.println(error_z_num);
}

void Mag_Normalization()
{
    //Read Mag data from AK8930 by 9250 I2C Master
    digitalWrite(MagSelectPin, LOW);
    SPI.transfer(MPU9250_REG_EXT_SENS_DATA_00|MPU9250_SPI_READ_MASK);
    byte ST1 = SPI.transfer(0xFF);
    byte X_LSB = SPI.transfer(0xFF);
    byte X_MSB = SPI.transfer(0xFF);
    byte Y_LSB = SPI.transfer(0xFF);
    byte Y_MSB = SPI.transfer(0xFF);
    byte Z_LSB = SPI.transfer(0xFF);
    byte Z_MSB = SPI.transfer(0xFF);
    byte ST2 = SPI.transfer(0xFF);
    digitalWrite(MagSelectPin, HIGH);

    short int x = ((X_MSB << 8) | X_LSB);
    short int y = ((Y_MSB << 8) | Y_LSB);
    short int z = ((Z_MSB << 8) | Z_LSB);
    
    
    
    //Read ACC data for Reference
    digitalWrite(MagSelectPin, LOW);
    SPI.transfer(MPU9250_REG_ACCEL_XOUT_H|MPU9250_SPI_READ_MASK);
    byte acc_xh = SPI.transfer(0xFF);
    byte acc_xl = SPI.transfer(0xFF);
    byte acc_yh = SPI.transfer(0xFF);
    byte acc_yl = SPI.transfer(0xFF);
    digitalWrite(MagSelectPin, HIGH);
    short int acc_x = ((acc_xh << 8) | acc_xl);
    short int acc_y = ((acc_yh << 8) | acc_yl);

    Serial.print("ACC_X:");
    Serial.print("\t");
    Serial.print(acc_x);
    Serial.print("\t");
    Serial.print("ACC_Y:");
    Serial.print("\t");
    Serial.print(acc_y);
    Serial.print("\t");
    Serial.print("X Max:");
    Serial.print(Mag_X_Max);
    Serial.print("\t");
    Serial.print("X Min:");
    Serial.println(Mag_X_Min);
  
  
  unsigned long timer = micros();
  int printTime = 0;
  delay(10);
  
  while(1)
  {
    //Read Mag data from AK8930 by 9250 I2C Master
    digitalWrite(MagSelectPin, LOW);
    SPI.transfer(MPU9250_REG_EXT_SENS_DATA_00|MPU9250_SPI_READ_MASK);
    byte ST1 = SPI.transfer(0xFF);
    byte X_LSB = SPI.transfer(0xFF);
    byte X_MSB = SPI.transfer(0xFF);
    byte Y_LSB = SPI.transfer(0xFF);
    byte Y_MSB = SPI.transfer(0xFF);
    byte Z_LSB = SPI.transfer(0xFF);
    byte Z_MSB = SPI.transfer(0xFF);
    byte ST2 = SPI.transfer(0xFF);
    digitalWrite(MagSelectPin, HIGH);

    short int x = ((X_MSB << 8) | X_LSB);
    short int y = ((Y_MSB << 8) | Y_LSB);
    short int z = ((Z_MSB << 8) | Z_LSB);

    //Read ACC data for Reference
    digitalWrite(MagSelectPin, LOW);
    SPI.transfer(MPU9250_REG_ACCEL_XOUT_H|MPU9250_SPI_READ_MASK);
    byte acc_xh = SPI.transfer(0xFF);
    byte acc_xl = SPI.transfer(0xFF);
    byte acc_yh = SPI.transfer(0xFF);
    byte acc_yl = SPI.transfer(0xFF);
    digitalWrite(MagSelectPin, HIGH);
    short int acc_x = ((acc_xh << 8) | acc_xl);
    short int acc_y = ((acc_yh << 8) | acc_yl);
    
    if(x > Mag_X_Max)
    {
      Mag_X_Max = x;
    }
    if(x < Mag_X_Min)
    {
      Mag_X_Min = x;
    }

    if(y > Mag_Y_Max)
    {
      Mag_Y_Max = y;
    }
    if(y < Mag_Y_Min)
    {
      Mag_Y_Min = y;
    }

    if(z > Mag_Z_Max)
    {
      Mag_Z_Max = z;
    }
    if(z < Mag_Z_Min)
    {
      Mag_Z_Min = z;
    }
     unsigned long currentTime = micros();
     int eclipseTime = (currentTime - timer)/1000000;
     if(eclipseTime > 30)
     {
        break;
     }
     if(eclipseTime > printTime)
     {
        printTime++;
        Serial.println(printTime);
     }
     Serial.print("ACC_X:");
     Serial.print("\t");
     Serial.print(acc_x);
     Serial.print("\t");
     Serial.print("ACC_Y:");
     Serial.print("\t");
     Serial.print(acc_y);
     Serial.print("\t");
     Serial.print("X Max:");
     Serial.print(Mag_X_Max);
     Serial.print("\t");
     Serial.print("X Min:");
     Serial.println(Mag_X_Min);
     /*Serial.print("\t");
     Serial.print("Y Max:");
     Serial.print(Mag_Y_Max);
     Serial.print("\t");
     Serial.print("Y Min:");
     Serial.print(Mag_Y_Min);
     Serial.print("Z Max:");
     Serial.print(Mag_Z_Max);
     Serial.print("\t");
     Serial.print("Z Min:");
     Serial.println(Mag_Z_Min);
     */
     delay(10);
  }

  Mag_X_Range = Mag_X_Max- Mag_X_Min;
  Mag_X_ZP = Mag_X_Range/2;

  Mag_Y_Range = Mag_Y_Max- Mag_Y_Min;
  Mag_Y_ZP = Mag_Y_Range/2;

  Mag_Z_Range = Mag_Z_Max- Mag_Z_Min;
  Mag_Z_ZP = Mag_Z_Range/2;
}

void showNormalizationResult()
{
  Serial.print("X Max:");
  Serial.print(Mag_X_Max);
  Serial.print("\t");
  Serial.print("X Min:");
  Serial.print(Mag_X_Min);
  Serial.print("\t");
  Serial.print("X Range:");
  Serial.print(Mag_X_Range);
  Serial.print("\t");
  Serial.print("X Zero Point:");
  Serial.println(Mag_X_ZP);
  
  
  Serial.print("Y Max:");
  Serial.print(Mag_Y_Max);
  Serial.print("\t");
  Serial.print("Y Min:");
  Serial.print(Mag_Y_Min);
  Serial.print("\t");
  Serial.print("Y Range:");
  Serial.print(Mag_Y_Range);
  Serial.print("\t");
  Serial.print("Y Zero Point:");
  Serial.println(Mag_Y_ZP);
  
  Serial.print("Z Max:");
  Serial.print(Mag_Z_Max);
  Serial.print("\t");
  Serial.print("Z Min:");
  Serial.print(Mag_Z_Min);
  Serial.print("\t");
  Serial.print("Z Range:");
  Serial.print(Mag_Z_Range);
  Serial.print("\t");
  Serial.print("Z Zero Point:");
  Serial.println(Mag_Z_ZP);
  
}

