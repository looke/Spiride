#include <math.h>
#include <SPI.h>
//#include "L3G4200D_Command.h"
//#include "ADXL345_Command.h"
#include "ADXRS453Z.h"
const int GyroSelectPin_1 = 10;
const int GyroSelectPin_2 = 9;

double rate = 0.0;
double oldRate = 0.0;
double threshold;
double threshold_neg;
double tempMultiResult;
int temp_big_drift_time = 0;
int total_loop_time = 0;

int total_overHold_time = 0;
int double_overHold_time = 0;
int triple_overHold_time = 0;
int quadruple_overHold_time = 0;
int more_overHold_time = 0;

double rate_1 = 0.0;
double rate_2 = 0.0;
double rate_abs_1 = 0.0;
double rate_abs_2 = 0.0;

double total_diff_value = 0.0;

void setup() {
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
  
  threshold = 0.5;
  threshold_neg = 0 - threshold;
  Serial.print("Threshold : ");
  Serial.println(threshold);
}

void loop() {
  // put your main code here, to run repeatedly:
  rate_1 = readRATE(1)+0.02;
  rate_2 = readRATE(2)+0.15;
  rate_abs_1 = rate_1;
  rate_abs_2 = rate_2;
  rate = 0.0;
  
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
  
  total_loop_time++;
  if( rate <= threshold_neg || threshold <= rate )
  {
    total_overHold_time++;
    temp_big_drift_time++;
    total_diff_value = total_diff_value + rate;

    if(temp_big_drift_time==2)
    {
      tempMultiResult = oldRate * rate;

      if(tempMultiResult > 0)
      {
        double_overHold_time ++;
      }
      else
      {
        temp_big_drift_time = 0;
      }
      
    }
    if(temp_big_drift_time==3)
    {
      tempMultiResult = oldRate * rate;
      if(tempMultiResult > 0)
      {
        triple_overHold_time++;
      }
      else
      {
        temp_big_drift_time = 0;
      }
    }
    
    if(temp_big_drift_time==4)
    {
      tempMultiResult = oldRate * rate;
      if(tempMultiResult > 0)
      {
        quadruple_overHold_time++;
      }
      else
      {
        temp_big_drift_time = 0;
      }
    }
    
    if(temp_big_drift_time>4)
    {
      tempMultiResult = oldRate * rate;
      if(tempMultiResult > 0)
      {
         more_overHold_time++;
      }
      else
      {
        temp_big_drift_time = 0;
      }
    }
  }
  else
  {
    temp_big_drift_time = 0;
  }
  oldRate = rate;
  Serial.print("Rate1 : ");
  Serial.print(rate_1);
  Serial.print("\t");
  Serial.print("Rate2 : ");
  Serial.print(rate_2);
  Serial.print("\t");
  Serial.print("Rate : ");
  Serial.print(rate);
  Serial.print("\t");

  Serial.print("Totle Overhold Rate: ");
  Serial.print(total_diff_value);
  Serial.print("\t");
  
  //Serial.print("Totle Loop : ");
  //Serial.print(total_loop_time);
  //Serial.print("\t");
  Serial.print("Totle Overhold : ");
  Serial.print(total_overHold_time);
  Serial.print("\t");

  Serial.print("Temp Drift : ");
  Serial.print(temp_big_drift_time);
  Serial.print("\t");
  
  Serial.print("Doulbe Overhold : ");
  Serial.print(double_overHold_time);
  Serial.print("\t");
  Serial.print("Triple OverHold : ");
  Serial.print(triple_overHold_time);
  Serial.print("\t");
  Serial.print("Quadruple OverHold : ");
  Serial.print(quadruple_overHold_time);
  Serial.print("\t");
  Serial.print("More than Quadruple : ");
  Serial.println(more_overHold_time);
  
}

void setupADXRS453(int Gyro)
{
  int GyroSelectPin = 0;
  if(Gyro == 1)
  {
    GyroSelectPin = GyroSelectPin_1;
  }
  if(Gyro == 2)
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

double readRATE(int Gyro)
{
  int GyroSelectPin = 0;
  if(Gyro == 1)
  {
    GyroSelectPin = GyroSelectPin_1;
  }
  if(Gyro == 2)
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
