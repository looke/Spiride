#include <SPI.h>
#include "ADXRS453Z.h"

const int GyroSelectPin = 10;

int max = 0;
int min = 0;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(GyroSelectPin, OUTPUT);
  delay(100);
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);

  delay(1000);

  setupADXRS453();
}

void loop() 
{
  // put your main code here, to run repeatedly:
  readRATE();

  delay(10);
}

void setupADXRS453()
{
  digitalWrite(GyroSelectPin, LOW);
  byte Start_4 = SPI.transfer(Init_4);
  byte Start_3 = SPI.transfer(Init_3); 
  byte Start_2 = SPI.transfer(Init_2); 
  byte Start_1 = SPI.transfer(Init_1); 
  digitalWrite(GyroSelectPin, HIGH);
  delay(60);
  Serial.print(Start_4,BIN);
  Serial.print(" ");
  Serial.print(Start_3,BIN);
  Serial.print(" ");
  Serial.print(Start_2,BIN);
  Serial.print(" ");
  Serial.println(Start_1,BIN);

  
  digitalWrite(GyroSelectPin, LOW);
  SPI.transfer(ReadInit_4);
  SPI.transfer(ReadInit_3);
  SPI.transfer(ReadInit_2);
  SPI.transfer(ReadInit_1);
  digitalWrite(GyroSelectPin, HIGH);
  delay(60);

  //while(1)
  //{
    digitalWrite(GyroSelectPin, LOW);
    byte Fault_4 = SPI.transfer(ReadInit_4);
    byte Fault_3 = SPI.transfer(ReadInit_3);
    byte Fault_2 = SPI.transfer(ReadInit_2);
    byte Fault_1 = SPI.transfer(ReadInit_1);
    digitalWrite(GyroSelectPin, HIGH);
    Serial.print(Fault_4,BIN);
    Serial.print(" ");
    Serial.print(Fault_3,BIN);
    Serial.print(" ");
    Serial.print(Fault_2,BIN);
    Serial.print(" ");
    Serial.println(Fault_1,BIN);
    delay(100);
  //}
    digitalWrite(GyroSelectPin, LOW);
    Fault_4 = SPI.transfer(ReadInit_4);
    Fault_3 = SPI.transfer(ReadInit_3);
    Fault_2 = SPI.transfer(ReadInit_2);
    Fault_1 = SPI.transfer(ReadInit_1);
    digitalWrite(GyroSelectPin, HIGH);
    Serial.print(Fault_4,BIN);
    Serial.print(" ");
    Serial.print(Fault_3,BIN);
    Serial.print(" ");
    Serial.print(Fault_2,BIN);
    Serial.print(" ");
    Serial.println(Fault_1,BIN);
    delay(100);

    digitalWrite(GyroSelectPin, LOW);
    Fault_4 = SPI.transfer(ReadInit_4);
    Fault_3 = SPI.transfer(ReadInit_3);
    Fault_2 = SPI.transfer(ReadInit_2);
    Fault_1 = SPI.transfer(ReadInit_1);
    digitalWrite(GyroSelectPin, HIGH);
    Serial.print(Fault_4,BIN);
    Serial.print(" ");
    Serial.print(Fault_3,BIN);
    Serial.print(" ");
    Serial.print(Fault_2,BIN);
    Serial.print(" ");
    Serial.println(Fault_1,BIN);
    
}

void readSN_MSB()
{
  digitalWrite(GyroSelectPin, LOW);
  byte PID0_4 = SPI.transfer(READCommand_SN_MSB_4);
  byte PID0_3 = SPI.transfer(READCommand_SN_MSB_3);
  byte PID0_2 = SPI.transfer(READCommand_SN_MSB_2);
  byte PID0_1 = SPI.transfer(READCommand_SN_MSB_1);
  digitalWrite(GyroSelectPin, HIGH);
  delay(1);
  digitalWrite(GyroSelectPin, LOW);
  byte PID1_4 = SPI.transfer(READCommand_SN_MSB_4);
  byte PID1_3 = SPI.transfer(READCommand_SN_MSB_3);
  byte PID1_2 = SPI.transfer(READCommand_SN_MSB_2);
  byte PID1_1 = SPI.transfer(READCommand_SN_MSB_1);
  digitalWrite(GyroSelectPin, HIGH);
  
  int result = 0;
  PID1_3 = PID1_3 & 0x1F;     //取低5位
  result = result | PID1_3;
  
  result = result << 8;       //左移8位
  result = result | PID1_2;
  
  result = result << 3;       //左移3位
  
  PID1_1 = PID1_1 & 0xE0;     //取高3位;
  PID1_1 = PID1_1 >> 5;       //右移5位将高3位移至低3位
  result = result | PID1_1;
  
  Serial.print("result: ");
  Serial.println(result,BIN);
}

void readPID()
{
  digitalWrite(GyroSelectPin, LOW);
  byte PID0_4 = SPI.transfer(READCommand_PID_4);
  byte PID0_3 = SPI.transfer(READCommand_PID_3);
  byte PID0_2 = SPI.transfer(READCommand_PID_2);
  byte PID0_1 = SPI.transfer(READCommand_PID_1);
  digitalWrite(GyroSelectPin, HIGH);
  delay(1);
  
  digitalWrite(GyroSelectPin, LOW);
  byte PID1_4 = SPI.transfer(READCommand_PID_4);
  byte PID1_3 = SPI.transfer(READCommand_PID_3);
  byte PID1_2 = SPI.transfer(READCommand_PID_2);
  byte PID1_1 = SPI.transfer(READCommand_PID_1);
  digitalWrite(GyroSelectPin, HIGH);
  
  Serial.print("PID0: ");
  Serial.print(PID0_4,BIN);
  Serial.print(" ");
  Serial.print(PID0_3,BIN);
  Serial.print(" ");
  Serial.print(PID0_2,BIN);
  Serial.print(" ");
  Serial.println(PID0_1,BIN);
  
  Serial.print("PID1: ");
  Serial.print(PID1_4,BIN);
  Serial.print(" ");
  Serial.print(PID1_3,BIN);
  Serial.print(" ");
  Serial.print(PID1_2,BIN);
  Serial.print(" ");
  Serial.println(PID1_1,BIN);

  int result = 0;
  PID1_3 = PID1_3 & 0x1F;     //取低5位
  result = result | PID1_3;
  
  result = result << 8;       //左移8位
  result = result | PID1_2;
  
  result = result << 3;       //左移3位
  
  PID1_1 = PID1_1 & 0xE0;     //取高3位;
  PID1_1 = PID1_1 >> 5;       //右移5位将高3位移至低3位
  result = result | PID1_1;
  
  Serial.print("result: ");
  Serial.println(result,BIN);
}

void readTEMP()
{
  digitalWrite(GyroSelectPin, LOW);
  byte PID0_4 = SPI.transfer(READCommand_TEMP_4);
  byte PID0_3 = SPI.transfer(READCommand_TEMP_3);
  byte PID0_2 = SPI.transfer(READCommand_TEMP_2);
  byte PID0_1 = SPI.transfer(READCommand_TEMP_1);
  digitalWrite(GyroSelectPin, HIGH);
  delay(1);
  
  digitalWrite(GyroSelectPin, LOW);
  byte PID1_4 = SPI.transfer(0x00);
  byte PID1_3 = SPI.transfer(0x00);
  byte PID1_2 = SPI.transfer(0x00);
  byte PID1_1 = SPI.transfer(0x00);
  digitalWrite(GyroSelectPin, HIGH);
  
  Serial.print("PID0: ");
  Serial.print(PID0_4,BIN);
  Serial.print(" ");
  Serial.print(PID0_3,BIN);
  Serial.print(" ");
  Serial.print(PID0_2,BIN);
  Serial.print(" ");
  Serial.println(PID0_1,BIN);
  
  Serial.print("PID1: ");
  Serial.print(PID1_4,BIN);
  Serial.print(" ");
  Serial.print(PID1_3,BIN);
  Serial.print(" ");
  Serial.print(PID1_2,BIN);
  Serial.print(" ");
  Serial.println(PID1_1,BIN);

  short int result = 0;
  PID1_3 = PID1_3 & 0x1F;     //取低5位
  result = result | PID1_3;
  Serial.print("result: ");
  Serial.println(result,BIN);
  
  result = result << 8;       //左移8位
  result = result | PID1_2;
  Serial.print("result: ");
  Serial.println(result,BIN);
  
  //result = result | 0x0f;
  result = result >> 3;
  int sign = result & 0x0200;
  
  if(sign != 0)
  {
    result = result & 0xFDFF;
    result = result-512;
  }
  
  result = 45 + result/5;

  Serial.print("result: ");
  //Serial.println(result,BIN);
  Serial.println(result);
}

void readRATE()
{
  digitalWrite(GyroSelectPin, LOW);
  byte DATA0_4 = SPI.transfer(READCommand_RATE_4);
  byte DATA0_3 = SPI.transfer(READCommand_RATE_3);
  byte DATA0_2 = SPI.transfer(READCommand_RATE_2);
  byte DATA0_1 = SPI.transfer(READCommand_RATE_1);
  digitalWrite(GyroSelectPin, HIGH);
  delay(1);
  
  digitalWrite(GyroSelectPin, LOW);
  byte DATA1_4 = SPI.transfer(0x00);
  byte DATA1_3 = SPI.transfer(0x00);
  byte DATA1_2 = SPI.transfer(0x00);
  byte DATA1_1 = SPI.transfer(0x00);
  digitalWrite(GyroSelectPin, HIGH);
  /*
  Serial.print("DATA0: ");
  Serial.print(DATA0_4,BIN);
  Serial.print(" ");
  Serial.print(DATA0_3,BIN);
  Serial.print(" ");
  Serial.print(DATA0_2,BIN);
  Serial.print(" ");
  Serial.println(DATA0_1,BIN);
  
  Serial.print("DATA1: ");
  Serial.print(DATA1_4,BIN);
  Serial.print(" ");
  Serial.print(DATA1_3,BIN);
  Serial.print(" ");
  Serial.print(DATA1_2,BIN);
  Serial.print(" ");
  Serial.println(DATA1_1,BIN);
  */
  int result = 0;
  DATA1_3 = DATA1_3 & 0x1F;     //取低5位
  result = result | DATA1_3;
  
  result = result << 8;       //左移8位
  result = result | DATA1_2;
  
  result = result << 3;       //左移3位
  
  DATA1_1 = DATA1_1 & 0xE0;     //取高3位;
  DATA1_1 = DATA1_1 >> 5;       //右移5位将高3位移至低3位
  result = result | DATA1_1;

  double dps = (double)result/80;
  if(result > max)
    max = result;
  if(result < min)
    min = result;
  Serial.print("Max:");
  Serial.print(max);
  Serial.print("\t");
  Serial.print("Min:");
  Serial.print(min);
  Serial.print("\t");
  Serial.print("Current:");
  Serial.print(result);
  Serial.print("\t");
  Serial.print("DPS:");
  Serial.println(dps,4);
}

