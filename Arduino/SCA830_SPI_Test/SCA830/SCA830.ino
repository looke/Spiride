#include <math.h>
#include <SPI.h>
#include "SCA830.h"
const int AccSelectPin = 9;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //SPI读取初始化
  pinMode(AccSelectPin, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setDataMode(SPI_MODE0);
  
  //digitalWrite(AccSelectPin, HIGH); //disable the device
  delay(200);
  setupACC();
  delay(1000);  
}

void setupACC()
{
  //read CTRL
  digitalWrite(AccSelectPin, LOW);
  byte head = SPI.transfer(WRITECommand_CTRL);
  byte result = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);
  
}
void loop() {
  // put your main code here, to run repeatedly:
  //check register
  /*
  digitalWrite(AccSelectPin, LOW);
  byte head = SPI.transfer(READCommand_CTRL);
  byte ctrl = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);
  Serial.print(head,BIN);
  Serial.print("\t");
  Serial.println(ctrl,BIN);
  */
  digitalWrite(AccSelectPin, LOW);
  byte head_M = SPI.transfer(READCommand_Y_MSB);
  byte MSB = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);

  digitalWrite(AccSelectPin, LOW);
  byte head_L = SPI.transfer(READCommand_Y_LSB);
  byte LSB = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);

  short int x = ((MSB << 8) | LSB);
  double angel = (double)x/32000;
  angel = asin(angel)*180/PI;
  Serial.print("MSB_head:");
  Serial.print(head_M,BIN);
  Serial.print("\t");
  Serial.print("MSB:");
  Serial.print(MSB,BIN);
  Serial.print("\t");
  Serial.print("LSB_head:");
  Serial.print(head_L,BIN);
  Serial.print("\t");
  Serial.print("LSB:");
  Serial.print(LSB,BIN);
  Serial.print("\t");
  Serial.print("X:");
  //Serial.println(x,BIN);
  //Serial.print("\t");
  Serial.print(x);
  Serial.print("\t");
  Serial.print("Angel:");
  Serial.println(angel,4);
  delay(10);
}
