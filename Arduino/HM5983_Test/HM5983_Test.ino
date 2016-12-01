#include <math.h>
#include <SPI.h>
#include "HM5983.h"
const int MagSelectPin = 10;

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
}

void setupMag()
{
  //set configration A
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(WRITECommand_CONFIG_A);
  SPI.transfer(0x98); //1001 0000  Temprature:ON /Samples Per Measure:1 /Output Rate:75HZ /Measure Mode:Normal
  digitalWrite(MagSelectPin, HIGH);

  //set configration B
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(WRITECommand_CONFIG_B);
  SPI.transfer(0x00); //0000 0000  Gain:0.88GA
  digitalWrite(MagSelectPin, HIGH);

  //set MODE
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(WRTIECommand_MODE);
  SPI.transfer(0x00); // SPI:4 Wire / Operating Mode:Continuous
  digitalWrite(MagSelectPin, HIGH);

  delay(3000);
  readIDA();
  readConfigA();
  readConfigB();
  //readMode();
  //readStatus();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  readDATA_XYZ();
  
  delay(10);
}

void readDATA_XYZ()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_DATAALL);
  byte X_MSB = SPI.transfer(0x00);
  byte X_LSB = SPI.transfer(0x00);
  
  byte Z_MSB = SPI.transfer(0x00);
  byte Z_LSB = SPI.transfer(0x00);

  byte Y_MSB = SPI.transfer(0x00);
  byte Y_LSB = SPI.transfer(0x00);

  short int x = ((X_MSB << 8) | X_LSB);
  short int y = ((Y_MSB << 8) | Y_LSB);
  short int z = ((Z_MSB << 8) | Z_LSB);

  double x_gauss = (double)x/1370;
  double y_gauss = (double)y/1370;
  double z_gauss = (double)z/1370;
  digitalWrite(MagSelectPin, HIGH);

  
  /*
  Serial.print("X:");
  Serial.print(X_MSB,BIN);
  Serial.print("\t");
  Serial.println(X_LSB,BIN);

  Serial.print("Y:");
  Serial.print(Y_MSB,BIN);
  Serial.print("\t");
  Serial.println(Y_LSB,BIN);

  Serial.print("Z:");
  Serial.print(Z_MSB,BIN);
  Serial.print("\t");
  Serial.println(Z_LSB,BIN);
  */
  Serial.print("X:");
  Serial.print(x_gauss,4);
  //Serial.print(X_MSB,BIN);
  Serial.print("\t");
  //Serial.println(X_LSB,BIN);

  Serial.print("Y:");
  Serial.print(y_gauss,4);
  
  //Serial.print(Y_MSB,BIN);
  Serial.print("\t");
  //Serial.println(Y_LSB,BIN);

  Serial.print("Z:");
  Serial.println(z_gauss,4);
  
  //Serial.print(Z_MSB,BIN);
  //Serial.print("\t");
  //Serial.println(Z_LSB,BIN);
  
  Serial.println("--------------------");
}



void readStatus()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_STATUS);
  byte Status = SPI.transfer(0x00);
  digitalWrite(MagSelectPin, HIGH);
  Serial.print("Status:");
  Serial.println(Status,BIN);
}

void readMode()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_MODE);
  byte Mode = SPI.transfer(0x00);
  digitalWrite(MagSelectPin, HIGH);
  Serial.print("Mode:");
  Serial.println(Mode,BIN);
}

void readIDA()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_IDA);
  byte IDA = SPI.transfer(0x00);
  digitalWrite(MagSelectPin, HIGH);
  Serial.print("IDA:");
  Serial.println(IDA,BIN);
}

void readConfigA()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_CONFIG_A);
  byte ConfigA = SPI.transfer(0x00);
  digitalWrite(MagSelectPin, HIGH);
  Serial.print("Configuration A:");
  Serial.println(ConfigA,BIN);
}

void readConfigB()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_CONFIG_B);
  byte ConfigB = SPI.transfer(0x00);
  digitalWrite(MagSelectPin, HIGH);
  Serial.print("Configuration B:");
  Serial.println(ConfigB,BIN);
}
