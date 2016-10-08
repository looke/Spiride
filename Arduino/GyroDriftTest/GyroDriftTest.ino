#include <math.h>
#include <SPI.h>
//#include "L3G4200D_Command.h"
//#include "ADXL345_Command.h"
#include "ADXRS453Z.h"
const int GyroSelectPin_2 = 9;
const int GyroSelectPin_1 = 10;
double rate = 0.0;

int count_0_999 = 0;
int count_0_100 = 0;
int count_0_095 = 0;
int count_0_090 = 0;
int count_0_085 = 0;
int count_0_080 = 0;
int count_0_075 = 0;
int count_0_070 = 0;
int count_0_065 = 0;
int count_0_060 = 0;
int count_0_055 = 0;
int count_0_050 = 0;
int count_0_045 = 0;
int count_0_040 = 0;
int count_0_035 = 0;
int count_0_030 = 0;
int count_0_025 = 0;
int count_0_020 = 0;
int count_0_015 = 0;
int count_0_010 = 0;
int count_0_005 = 0;

//int count_0_0 = 0;

int count_005_0 = 0;
int count_010_0 = 0;
int count_015_0 = 0;
int count_020_0 = 0;
int count_025_0 = 0;
int count_030_0 = 0;
int count_035_0 = 0;
int count_040_0 = 0;
int count_045_0 = 0;
int count_050_0 = 0;
int count_055_0 = 0;
int count_060_0 = 0;
int count_065_0 = 0;
int count_070_0 = 0;
int count_075_0 = 0;
int count_080_0 = 0;
int count_085_0 = 0;
int count_090_0 = 0;
int count_095_0 = 0;
int count_100_0 = 0;
int count_999_0 = 0;


void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(GyroSelectPin_1, OUTPUT);
  pinMode(GyroSelectPin_2, OUTPUT);
  delay(100);
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);

  delay(1000);

  setupADXRS453(1);
  setupADXRS453(2);
}

void loop() 
{
  // put your main code here, to run repeatedly:
  double rate_1 = readRATE(1);
  double rate_2 = readRATE(2);
  double rate_abs_1 = 0.0;
  double rate_abs_2 = 0.0;
  double rate = 0.0;
  if(rate_1 < 0)
  {
    rate_abs_1 = 0-rate_1;
  }
  if(rate_2 < 0)
  {
    rate_abs_2 = 0-rate_2;
  }
  
  if(rate_abs_1 > rate_abs_2)
  {
    rate = rate_2;
  }
  else
  {
    rate = rate_1;
  }
  
  if(rate <= -1.0)
  {
    count_999_0++;
  }
  else if (-1.0 < rate && rate <= -0.95)
  {
    count_100_0++;
  }
  else if (-0.95 < rate && rate <= -0.9)
  {
    count_095_0++;
  }
  else if (-0.9 < rate && rate <= -0.85)
  {
    count_090_0++;
  }
  else if (-0.85 < rate && rate <= -0.8)
  {
    count_085_0++;
  }
  else if (-0.8 < rate && rate <= -0.75)
  {
    count_080_0++;
  }
  else if (-0.75 < rate && rate <= -0.7)
  {
    count_075_0++;
  }
  else if (-0.7 < rate && rate <= -0.65)
  {
    count_070_0++;
  }
  else if (-0.65 < rate && rate <= -0.6)
  {
    count_065_0++;
  }
  else if (-0.6 < rate && rate <= -0.55)
  {
    count_060_0++;
  }
  else if (-0.55 < rate && rate <= -0.5)
  {
    count_055_0++;
  }
  else if (-0.5 < rate && rate <= -0.45)
  {
    count_050_0++;
  }
  else if (-0.45 < rate && rate <= -0.4)
  {
    count_045_0++;
  }
  else if (-0.4 < rate && rate <= -0.35)
  {
    count_040_0++;
  }
  else if (-0.35 < rate && rate <= -0.3)
  {
    count_035_0++;
  }
  else if (-0.3 < rate && rate <= -0.25)
  {
    count_030_0++;
  }
  else if (-0.25 < rate && rate <= -0.2)
  {
    count_025_0++;
  }
  else if (-0.2 < rate && rate <= -0.15)
  {
    count_020_0++;
  }
  else if (-0.15 < rate && rate <= -0.1)
  {
    count_015_0++;
  }
  else if (-0.1 < rate && rate <= -0.05)
  {
    count_010_0++;
  }
  else if (-0.05 < rate && rate < 0.0)
  {
    count_005_0++;
  }




    
  else if (0 <= rate && rate < 0.05)
  {
    count_0_005++;
  }
  else if (0.05 <= rate && rate < 0.1)
  {
    count_0_010++;
  }
  else if (0.1 <= rate && rate < 0.15)
  {
    count_0_015++;
  }
  else if (0.15 <= rate && rate < 0.2)
  {
    count_0_020++;
  }
  else if (0.2 <= rate && rate < 0.25)
  {
    count_0_025++;
  }
  else if (0.25 <= rate && rate < 0.3)
  {
    count_0_030++;
  }
  else if (0.3 <= rate && rate < 0.35)
  {
    count_0_035++;
  }
  else if (0.35 <= rate && rate < 0.4)
  {
    count_0_040++;
  }
  else if (0.4 <= rate && rate < 0.45)
  {
    count_0_045++;
  }
  else if (0.45 <= rate && rate < 0.5)
  {
    count_0_050++;
  }
  else if (0.5 <= rate && rate < 0.55)
  {
    count_0_055++;
  }
  else if (0.55 <= rate && rate < 0.6)
  {
    count_0_060++;
  }
  else if (0.6 <= rate && rate < 0.65)
  {
    count_0_065++;
  }
  else if (0.65 <= rate && rate < 0.7)
  {
    count_0_070++;
  }
  else if (0.7 <= rate && rate < 0.75)
  {
    count_0_075++;
  }
  else if (0.75 <= rate && rate < 0.8)
  {
    count_0_080++;
  }
  else if (0.8 <= rate && rate < 0.85)
  {
    count_0_085++;
  }
  else if (0.85 <= rate && rate < 0.9)
  {
    count_0_090++;
  }
  else if (0.9 <= rate && rate < 0.95)
  {
    count_0_095++;
  }
  else if (0.95 <= rate && rate < 1.0)
  {
    count_0_100++;
  }
  else if (1.0 <= rate)
  {
    count_0_999++;
  }
  
  Serial.print("(-~,-1.0] : ");
  Serial.print(count_999_0);
  Serial.print("\t");
  Serial.print("(-1.0,-0.95] : ");
  Serial.print(count_100_0);
  Serial.print("\t");
  Serial.print("(-0.95,-0.9] : ");
  Serial.print(count_095_0);
  Serial.print("\t");
  Serial.print("(-0.9,-0.85] : ");
  Serial.print(count_090_0);
  Serial.print("\t");
  Serial.print("(-0.85,-0.8] : ");
  Serial.print(count_085_0);
  Serial.print("\t");
  Serial.print("(-0.8,-0.75] : ");
  Serial.print(count_080_0);
  Serial.print("\t");
  Serial.print("(-0.75,-0.7] : ");
  Serial.print(count_075_0);
  Serial.print("\t");
  Serial.print("(-0.7,-0.65] : ");
  Serial.print(count_070_0);
  Serial.print("\t");
  Serial.print("(-0.65,-0.6] : ");
  Serial.print(count_065_0);
  Serial.print("\t");
  Serial.print("(-0.6,-0.55] : ");
  Serial.print(count_060_0);
  Serial.print("\t");
  Serial.print("(-0.55,-0.5] : ");
  Serial.print(count_055_0);
  Serial.print("\t");
  Serial.print("(-0.5,-0.45] : ");
  Serial.print(count_050_0);
  Serial.print("\t");
  Serial.print("(-0.45,-0.4] : ");
  Serial.print(count_045_0);
  Serial.print("\t");
  Serial.print("(-0.4,-0.35] : ");
  Serial.print(count_040_0);
  Serial.print("\t");
  Serial.print("(-0.35,-0.3] : ");
  Serial.print(count_035_0);
  Serial.print("\t");
  Serial.print("(-0.3,-0.25] : ");
  Serial.print(count_030_0);
  Serial.print("\t");
  Serial.print("(-0.25,-0.2] : ");
  Serial.print(count_025_0);
  Serial.print("\t");
  Serial.print("(-0.2,-0.15] : ");
  Serial.print(count_020_0);
  Serial.print("\t");
  Serial.print("(-0.15,-0.1] : ");
  Serial.print(count_015_0);
  Serial.print("\t");
  Serial.print("(-0.1,-0.05) : ");
  Serial.println(count_010_0);
  Serial.print("\t");
  Serial.print("(-0.05,0.0) : ");
  Serial.println(count_005_0);
  Serial.print("\t");
  
  Serial.print("[0,0.05) : ");
  Serial.print(count_0_005);
  Serial.print("\t");
  Serial.print("[0.05,0.1) : ");
  Serial.print(count_0_010);
  Serial.print("\t");
  Serial.print("[0.1,0.15) : ");
  Serial.print(count_0_015);
  Serial.print("\t");
  Serial.print("[0.15,0.2) : ");
  Serial.print(count_0_020);
  Serial.print("\t");
  Serial.print("[0.2,0.25) : ");
  Serial.print(count_0_025);
  Serial.print("\t");
  Serial.print("[0.25,0.3) : ");
  Serial.print(count_0_030);
  Serial.print("\t");
  Serial.print("[0.3,0.35) : ");
  Serial.print(count_0_035);
  Serial.print("\t");
  Serial.print("[0.35,0.4) : ");
  Serial.print(count_0_040);
  Serial.print("\t");
  Serial.print("[0.4,0.45) : ");
  Serial.print(count_0_045);
  Serial.print("\t");
  Serial.print("[0.45,0.5) : ");
  Serial.print(count_0_050);
  Serial.print("\t");
  Serial.print("[0.5,0.55) : ");
  Serial.print(count_0_055);
  Serial.print("\t");
  Serial.print("[0.55,0.6) : ");
  Serial.print(count_0_060);
  Serial.print("\t");
  Serial.print("[0.6,0.65) : ");
  Serial.print(count_0_065);
  Serial.print("\t");
  Serial.print("[0.65,0.7) : ");
  Serial.print(count_0_070);
  Serial.print("\t");
  Serial.print("[0.7,0.75) : ");
  Serial.print(count_0_075);
  Serial.print("\t");
  Serial.print("[0.75,0.8) : ");
  Serial.print(count_0_080);
  Serial.print("\t");
  Serial.print("[0.8,0.85) : ");
  Serial.print(count_0_085);
  Serial.print("\t");
  Serial.print("[0.85,0.9) : ");
  Serial.print(count_0_090);
  Serial.print("\t");
  Serial.print("[0.9,0.95) : ");
  Serial.print(count_0_095);
  Serial.print("\t");
  Serial.print("[0.95,1.0) : ");
  Serial.print(count_0_100);
  Serial.print("\t");
  Serial.print("[1.0,~) : ");
  Serial.println(count_0_999);
  
  Serial.println("--------------------------");
  
  
}

void setupADXRS453(int gyroNo)
{
  int GyroSelectPin = 0;
  if(gyroNo == 1)
  {
    GyroSelectPin = GyroSelectPin_1;
  }
  if(gyroNo == 2)
  {
    GyroSelectPin = GyroSelectPin_2;
  }
  
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

double readRATE(int gyroNo)
{
  int GyroSelectPin = 0;
  if(gyroNo == 1)
  {
    GyroSelectPin = GyroSelectPin_1;
  }
  if(gyroNo == 2)
  {
    GyroSelectPin = GyroSelectPin_2;
  }
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
  return dps;
}
