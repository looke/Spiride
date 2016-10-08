#include <math.h>
#include <SPI.h>
#include "ADXL345_Command.h"
//const int GyroSelectPin = 10;
const int AccSelectPin = 9;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  //SPI读取初始化
  pinMode(AccSelectPin, OUTPUT);
  //pinMode(GyroSelectPin, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setDataMode(SPI_MODE3);
  
  digitalWrite(AccSelectPin, HIGH); //disable the device
  //digitalWrite(GyroSelectPin, HIGH); //disable the device
  
  setupACC();
  delay(1000);
}

void loop() 
{
  // put your main code here, to run repeatedly:
  delay(100);
  //readAccRegValue(READCommand_DATA_FORMAT);
  readAccXValue();
}

void setupACC()
{
  digitalWrite(AccSelectPin, LOW);
  SPI.transfer(WRITECommand_POWER_CTL);
  SPI.transfer(0x08); //normal measure mode
  digitalWrite(AccSelectPin, HIGH);

  digitalWrite(AccSelectPin, LOW);
  SPI.transfer(WRITECommand_DATA_FORMAT);
  SPI.transfer(0x01); //4g range
  digitalWrite(AccSelectPin, HIGH);
  //Serial.println(reg,BIN);
}

void readAccRegValue(byte command)
{
  digitalWrite(AccSelectPin, LOW);
  SPI.transfer(command);
  byte reg = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);
  Serial.println(reg,BIN);
}

void readAccXValue()
{
  digitalWrite(AccSelectPin, LOW);
  SPI.transfer(READCommand_DATA_X0);
  byte xLSB = SPI.transfer(0x00);
  byte xMSB = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);
  int x = ((xMSB << 8) | xLSB);
  x=x+5;
  double degree = (double)x/161;
  degree = asin(degree)*180/PI;
  //Serial.println(xLSB,BIN);
  //Serial.println(xMSB,BIN);
  //Serial.println(x,BIN);
  Serial.print(x);
  Serial.print("\t");
  Serial.println(degree);
}
