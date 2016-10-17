#include <math.h>
#include <SPI.h>
#include "SCA3100.h"
const int AccSelectPin = 8;

double FullScale = 2074.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //SPI读取初始化
  pinMode(AccSelectPin, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setDataMode(SPI_MODE0);

  delay(200);
  setupACC();
  delay(1000);
  
}

void setupACC()
{
  //Read DevID
  digitalWrite(AccSelectPin, LOW);
  byte head = SPI.transfer(READCommand_REVID);
  byte result = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);

  Serial.print("REVID:");
  Serial.print(head,BIN);
  Serial.print("\t");
  Serial.println(result,BIN);

  
  //read CTRL
  digitalWrite(AccSelectPin, LOW);
  head = SPI.transfer(WRITECommand_CTRL);
  result = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);
  
}


void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(AccSelectPin, LOW);
  byte head_M = SPI.transfer(READCommand_Z_MSB);
  byte MSB = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);

  digitalWrite(AccSelectPin, LOW);
  byte head_L = SPI.transfer(READCommand_Z_LSB);
  byte LSB = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);

  short int finalZ = ((MSB << 8) | LSB);
  finalZ = finalZ >> 1;
  
  digitalWrite(AccSelectPin, LOW);
  head_M = SPI.transfer(READCommand_X_MSB);
  MSB = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);

  digitalWrite(AccSelectPin, LOW);
  head_L = SPI.transfer(READCommand_X_LSB);
  LSB = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);
  
  short int finalX= ((MSB << 8) | LSB);
  finalX = finalX >> 1;
  
  digitalWrite(AccSelectPin, LOW);
  head_M = SPI.transfer(READCommand_Y_MSB);
  MSB = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);

  digitalWrite(AccSelectPin, LOW);
  head_L = SPI.transfer(READCommand_Y_LSB);
  LSB = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);
  
  short int finalY= ((MSB << 8) | LSB);
  finalY = finalY >> 1;

  double angel = atan2(finalY, finalZ)*180/PI;


  
  //double gRate = (double)finalX*10/18000;

  //if(gRate < -1)
  //{
  //  gRate = -1;
  //}
  /*
  double gRateForOtherSide = gRate * gRate;

  gRateForOtherSide = 1-gRateForOtherSide;

  gRateForOtherSide = sqrt(gRateForOtherSide);

  double tang = gRateForOtherSide/gRate;
  */
  //double angel = asin(gRate)*180/PI;
  
 
  //Serial.print("MSB_head:");
  //Serial.print(head_M,BIN);
  //Serial.print("\t");
  //Serial.print("MSB:");
  //Serial.print(MSB,BIN);
  //Serial.print("\t");
  //Serial.print("LSB_head:");
  //Serial.print(head_L,BIN);
  //Serial.print("\t");
  //Serial.print("LSB:");
  //Serial.print(LSB,BIN);
  //Serial.print("\t");
  //Serial.print("X:");
  //Serial.print(x,BIN);
  //Serial.print("\t");
  
  Serial.print("finalX:");
  //Serial.print(finalX,BIN);
  //Serial.print("\t");
  
  Serial.print(finalX);
  Serial.print("\t");

  Serial.print("finalY:");
  Serial.print(finalY);
  Serial.print("\t");

  Serial.print("finalZ:");
  Serial.print(finalZ);
  Serial.print("\t");

  Serial.print("Angel:");
  Serial.println(angel);
  
  //Serial.print("Other Side G:");
  //Serial.print(gRateForOtherSide);
  //Serial.print("\t");
  
  //Serial.print("Angel:");
  //Serial.println(angel);
  
  

  
  delay(10);
}
