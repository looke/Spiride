#include <math.h>
#include <SPI.h>
#include "ADXL345_Command.h"
const int AccSelectPin = 4;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  //SPI读取初始化
  SPI.begin(AccSelectPin);
  SPI.setBitOrder(AccSelectPin,MSBFIRST);
  SPI.setClockDivider(AccSelectPin,21);
  SPI.setDataMode(AccSelectPin,SPI_MODE3);
  
  //digitalWrite(AccSelectPin, HIGH); //disable the device
  //digitalWrite(GyroSelectPin, HIGH); //disable the device
  
  setupACC();
  delay(1000);
}

void loop() 
{
  // put your main code here, to run repeatedly:
  delay(100);
  readAccRegValue(READCommand_DATA_FORMAT);
  //readAccXValue();
}

void setupACC()
{
  //digitalWrite(AccSelectPin, LOW);
  SPI.transfer(AccSelectPin,WRITECommand_POWER_CTL,SPI_CONTINUE);
  SPI.transfer(AccSelectPin,0x08); //normal measure mode
  //digitalWrite(AccSelectPin, HIGH);
  delay(10);
  //digitalWrite(AccSelectPin, LOW);
  SPI.transfer(AccSelectPin,WRITECommand_DATA_FORMAT,SPI_CONTINUE);
  SPI.transfer(AccSelectPin,0x01); //4g range
  //digitalWrite(AccSelectPin, HIGH);
  //Serial.println(reg,BIN);
}

void readAccRegValue(byte command)
{
  //digitalWrite(AccSelectPin, LOW);
  byte reg = SPI.transfer(AccSelectPin,command);
  //byte reg = SPI.transfer(0x00);
  //digitalWrite(AccSelectPin, HIGH);
  Serial.println(reg,BIN);
}

void readAccXValue()
{
  //digitalWrite(AccSelectPin, LOW);
  //SPI.transfer(AccSelectPin,READCommand_DATA_X0,SPI_CONTINUE);
  //byte xLSB = SPI.transfer(AccSelectPin,0x00,SPI_CONTINUE);
  //byte xMSB = SPI.transfer(AccSelectPin,0x00);
  //digitalWrite(AccSelectPin, HIGH);

  SPI.transfer(AccSelectPin,READCommand_DATA_X0, SPI_CONTINUE);
  byte xLSB = SPI.transfer(AccSelectPin,0x00, SPI_CONTINUE);
  byte xMSB = SPI.transfer(AccSelectPin,0x00);
  //SPI.transfer(AccSelectPin,READCommand_DATA_X1, SPI_CONTINUE);
  //byte xMSB = SPI.transfer(AccSelectPin,0x00);
  //byte xLSB = SPI.transfer(AccSelectPin,0x00,SPI_CONTINUE);
  //byte xMSB = SPI.transfer(AccSelectPin,0x00);
  short int x = ((xMSB << 8) | xLSB);
  x=x+5;
  double degree = (double)x/161;
  degree = asin(degree)*180/PI;
  Serial.print("LSB:");
  Serial.print("\t");
  Serial.print(xLSB,BIN);
  Serial.print("\t");
  Serial.print("MSB:");
  Serial.print("\t");
  Serial.print(xMSB,BIN);
  Serial.print("\t");
  Serial.print("X:");
  Serial.print("\t");
  Serial.print(x,BIN);
  Serial.print("\t");
  Serial.print(x);
  Serial.print("\t");
  Serial.print("degree:");
  Serial.print("\t");
  Serial.println(degree);
}
