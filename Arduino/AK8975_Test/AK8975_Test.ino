#include <math.h>
#include <SPI.h>
#include "AK8975.h"
const int MagSelectPin = 10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //SPI读取初始化
  pinMode(MagSelectPin, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setDataMode(SPI_MODE3);
  digitalWrite(MagSelectPin, LOW);
  delay(200);
  digitalWrite(MagSelectPin, HIGH);
  setupMag();
  delay(1000);
}

void setupMag()
{

}

void loop() {
  // put your main code here, to run repeatedly:
  read_WHO_AM_I();
  delay(1000);
}


void read_WHO_AM_I()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_DEVICE_ID);
  byte ID = SPI.transfer(0x00);
  digitalWrite(MagSelectPin, HIGH);
  Serial.print("Device ID:");
  Serial.println(ID,BIN);
}
