#include <math.h>
#include <SPI.h>
#include "L3G4200D_Command.h"

const int chipSelectPin = 10;
long WHO_AM_I = 0;

double dps=0;

double max = 0;
double min = 0;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  //SPI读取初始化
  pinMode(chipSelectPin, OUTPUT);
  SPI.begin();
  //SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE3);
  digitalWrite(chipSelectPin, HIGH); //disable the device
  setupGyro();
  delay(1000);
}

void setupGyro()
{
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(WRITECommand_CTRL_REG4);
  SPI.transfer(0b00010000);
  digitalWrite(chipSelectPin, HIGH);
  delay(100);
  
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(WRITECommand_CTRL_REG1);
  SPI.transfer(0b00001111);
  digitalWrite(chipSelectPin, HIGH);
}
void loop() 
{
  // put your main code here, to run repeatedly:
  delay(100);
  getYGyroValue();
}

void getYGyroValue()
{
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(READCommand_OUT_Y_L);
  byte yLSB = SPI.transfer(0x00);
  //SPI.transfer(READCommand_OUT_Y_H);
  byte yMSB = SPI.transfer(0x00);
  digitalWrite(chipSelectPin, HIGH);
  
  int y = ((yMSB << 8) | yLSB);
  dps = y*0.0175;
  if(dps > max)
    max = dps;
  if(dps < min)
    min = dps;
  //Serial.println(yLSB,BIN);
  //Serial.println("\n");
  //Serial.println(yMSB,BIN);
  //Serial.println("\n");
  //Serial.println(y,BIN);
  //Serial.println("\n");
  //Serial.println(y);
  Serial.print("Max:");
  Serial.print(max);
  Serial.print("\t");
  Serial.print("Min:");
  Serial.print(min);
  Serial.print("\t");
  Serial.print("Current:");
  Serial.println(dps);
  //Serial.println("###################");
}

void getRegValue(byte command)
{
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(command);
  byte reg = SPI.transfer(0x00);
  digitalWrite(chipSelectPin, HIGH);
  Serial.println(reg,BIN);
}

