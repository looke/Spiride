#include <math.h>
#include <SPI.h>
#include "L3G4200D_Command.h"

const int gyroPin = 4;

double dps=0;
void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  //SPI读取初始化
  
  SPI.begin(gyroPin);
  SPI.setBitOrder(gyroPin,MSBFIRST);
  SPI.setClockDivider(gyroPin,21);
  SPI.setDataMode(gyroPin,SPI_MODE3);
  
  setupGyro();
  delay(1000);
}

void setupGyro()
{
  //digitalWrite(chipSelectPin, LOW);
  SPI.transfer(gyroPin,WRITECommand_CTRL_REG4,SPI_CONTINUE);
  SPI.transfer(gyroPin,0b00010000);
  //digitalWrite(chipSelectPin, HIGH);
  delay(100);
  
  //digitalWrite(chipSelectPin, LOW);
  SPI.transfer(gyroPin,WRITECommand_CTRL_REG1,SPI_CONTINUE);
  SPI.transfer(gyroPin,0b00001111);
  //digitalWrite(chipSelectPin, HIGH);
}
void loop() 
{
  // put your main code here, to run repeatedly:
  delay(1000);
  getYGyroValue();
  //getRegValue(READCommand_WHO_AM_I);
}

void getYGyroValue()
{
  //digitalWrite(chipSelectPin, LOW);
  SPI.transfer(gyroPin,READCommand_OUT_Y_L,SPI_CONTINUE);
  byte yLSB = SPI.transfer(gyroPin,0x00,SPI_CONTINUE);
  //SPI.transfer(READCommand_OUT_Y_H);
  byte yMSB = SPI.transfer(gyroPin,0x00);
  //digitalWrite(chipSelectPin, HIGH);
  
  short int y = ((yMSB << 8) | yLSB);
  dps = y*0.0175;
  Serial.print("yLSB:");
  Serial.print(yLSB,BIN);
  Serial.print("\t");
  Serial.print("yMSB:");
  Serial.print(yMSB,BIN);
  Serial.print("\t");
  Serial.print("y:");
  Serial.print(y,BIN);
  Serial.print("\t");
  Serial.print("dps:");
  
  //Serial.println("\n");
  //Serial.println(yMSB,BIN);
  //Serial.println("\n");
  //Serial.println(y,BIN);
  //Serial.println("\n");
  //Serial.println(y);
  Serial.println(dps);
  //Serial.println("###################");
}

void getRegValue(byte command)
{
  //digitalWrite(chipSelectPin, LOW);
  SPI.transfer(gyroPin,command,SPI_CONTINUE);
  byte reg = SPI.transfer(gyroPin,0x00);
  //digitalWrite(chipSelectPin, HIGH);
  Serial.println(reg,BIN);
}

