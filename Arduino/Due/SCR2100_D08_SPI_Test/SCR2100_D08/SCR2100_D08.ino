#include <SPI.h>
const int GyroSelectPin = 10;

const byte READCommand_RATE_4 = 0x04;
const byte READCommand_RATE_3 = 0x00;
const byte READCommand_RATE_2 = 0x00;
const byte READCommand_RATE_1 = 0xF7;

const word READCommand_REVID_4 = 0x74;
const word READCommand_REVID_3 = 0x00;
const word READCommand_REVID_2 = 0x00;
const word READCommand_REVID_1 = 0xBF;

const byte READCommand_TEMP_4 = 0x1C;
const byte READCommand_TEMP_3 = 0x00;
const byte READCommand_TEMP_2 = 0x00;
const byte READCommand_TEMP_1 = 0xE3;

const byte READCommand_RATEStatus1_4 = 0x24;
const byte READCommand_RATEStatus1_3 = 0x00;
const byte READCommand_RATEStatus1_2 = 0x00;
const byte READCommand_RATEStatus1_1 = 0xc7;

const byte READCommand_RATEStatus2_4 = 0x28;
const byte READCommand_RATEStatus2_3 = 0x00;
const byte READCommand_RATEStatus2_2 = 0x00;
const byte READCommand_RATEStatus2_1 = 0xcd;

const byte READCommand_ACCStatus_4 = 0x3c;
const byte READCommand_ACCStatus_3 = 0x00;
const byte READCommand_ACCStatus_2 = 0x00;
const byte READCommand_ACCStatus_1 = 0xd3;

const byte READCommand_CommonStatus_4 = 0x6c;
const byte READCommand_CommonStatus_3 = 0x00;
const byte READCommand_CommonStatus_2 = 0x00;
const byte READCommand_CommonStatus_1 = 0xab;

const byte READCommand_StatusSumm_4 = 0x7c;
const byte READCommand_StatusSumm_3 = 0x00;
const byte READCommand_StatusSumm_2 = 0x00;
const byte READCommand_StatusSumm_1 = 0xb3;

const byte WRITECommand_FLT60Hz_4 = 0xfc;
const byte WRITECommand_FLT60Hz_3 = 0x20;
const byte WRITECommand_FLT60Hz_2 = 0x00;
const byte WRITECommand_FLT60Hz_1 = 0x06;

const byte WRITECommand_HARDRst_4 = 0xd8;
const byte WRITECommand_HARDRst_3 = 0x00;
const byte WRITECommand_HARDRst_2 = 0x04;
const byte WRITECommand_HARDRst_1 = 0x31;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //SPI读取初始化
  SPI.begin(GyroSelectPin);
  SPI.setBitOrder(GyroSelectPin,MSBFIRST);
  SPI.setClockDivider(GyroSelectPin,21);
  SPI.setDataMode(GyroSelectPin,SPI_MODE3);

  delay(1000);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  SPI.transfer(GyroSelectPin,READCommand_ACCStatus_4, SPI_CONTINUE);
  SPI.transfer(GyroSelectPin,READCommand_ACCStatus_3, SPI_CONTINUE);
  SPI.transfer(GyroSelectPin,READCommand_ACCStatus_2, SPI_CONTINUE);
  SPI.transfer(GyroSelectPin,READCommand_ACCStatus_1);

  byte head = SPI.transfer(GyroSelectPin,READCommand_ACCStatus_4, SPI_CONTINUE);
  byte data_MSB = SPI.transfer(GyroSelectPin,READCommand_ACCStatus_3, SPI_CONTINUE);
  byte data_LSB = SPI.transfer(GyroSelectPin,READCommand_ACCStatus_2, SPI_CONTINUE);
  byte CRC = SPI.transfer(GyroSelectPin,READCommand_ACCStatus_1);
  Serial.print("Head:");
  Serial.print(head,BIN);
  Serial.print("\t");
  Serial.print("data_MSB:");
  Serial.print(data_MSB,BIN);
  Serial.print("\t");
  Serial.print("data_LSB:");
  Serial.print(data_LSB,BIN);
  Serial.print("\t");
  Serial.print("CRC:");
  Serial.println(CRC,BIN);
  delay(1000);
}
