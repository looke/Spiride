#include <math.h>
#include <SPI.h>
//#include "L3G4200D_Command.h"
//#include "ADXL345_Command.h"
#include "SCA830.h"
#include "ADXRS453Z.h"
//#include "BasicKalman.h"
#include "SpirideKalman.h"
const int GyroSelectPin = 10;
const int AccSelectPin = 9;

Kalman bk;
unsigned long timer;
unsigned long timer2;
double pitch;
double degree;
double rate;
float bias;
double dt;
double pre_degree;
double K0;
double K1;

double basicBias;
double basicAngle;

int currentState;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //SPI读取初始化
  pinMode(AccSelectPin, OUTPUT);
  pinMode(GyroSelectPin, OUTPUT);

  //SPI读取初始化
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  //SPI.setDataMode(SPI_MODE3);//L3G4200D
  SPI.setDataMode(SPI_MODE0); //SCA830
  digitalWrite(AccSelectPin, HIGH); //disable the device
  digitalWrite(GyroSelectPin, HIGH); //disable the device
  delay(200);
  //setupACC_ADXL345();
  //setupGyro_L3G4200D();
  setupACC_SCA830();
  setupGyro_ADXRS453Z();
  
  delay(1000); //give device time to init

  basicBias = calcGyroBasicBias();
  basicAngle = calcACCBasicAngle();

  degree = readAccValue_SCA830();
  bk.setBasicBias(basicBias);
  bk.setBasicAngle(basicAngle);
  bk.setAngle(degree);
  pre_degree=0;
  timer = micros();
}

void loop() 
{
  // put your main code here, to run repeatedly:
  

  //degree = readAccValue_ADXL345();
  //rate = readGyroValue_L3G4200D();
  
  degree = readAccValue_SCA830();
  rate = readGyroValue_ADXRS453Z();
  
  dt = double(micros() - timer)/1000000;
  //pre_degree = pitch + rate*dt;
  pitch = bk.getAngle(degree,rate,dt);  // Calculate the angle using the Kalman filter
  timer = micros(); 

  
  bias = bk.getBias();
  K0=bk.getK0();
  K1=bk.getK1();
  currentState = bk.getCurrentState();
  basicAngle = bk.getBasicAngle();
  String outputContent;

  char temp[22];
  dtostrf(degree, 6, 4, temp); 
  String str_degree = String(temp);

  dtostrf(rate, 6, 4, temp); 
  String str_rate = String(temp);
  
  dtostrf(dt, 6, 4, temp);
  String str_dt = String(temp);

  dtostrf(pitch, 6, 4, temp);
  String str_pitch = String(temp);

  dtostrf(bias, 6, 4, temp);
  String str_bias = String(temp);

  dtostrf(basicAngle, 6, 4, temp);
  String str_basicAngle = String(temp);
  
  String str_K0 = String(K0);
  String str_K1 = String(K1);
  String state = String(currentState);
  outputContent = "State:" + state +";degree:"+str_degree + ";rate:" + str_rate + ";dt:" + str_dt + ";pitch:" + str_pitch + ";bias:" + str_bias + ";K0:" + str_K0 + ";K1:" + str_K1 + ";BAngle:" + str_basicAngle;
  Serial.println(outputContent);


  /*
  //---------------------------------------------
  Serial.print("degree:");
  Serial.print(degree);
  Serial.print('\t');
  Serial.print("rate:");
  Serial.print(rate);
  Serial.print('\t');
  
  Serial.print("dt:");
  Serial.print(dt);
  
  Serial.print('\t');  
  Serial.print("degree k|k-1 :");
  Serial.print(pre_degree);
  Serial.print('\t');
  
  Serial.print("pitch:");
  Serial.print(pitch);
  
  Serial.print('\t');
  Serial.print("bias:");
  Serial.print(bias);
  Serial.print('\t');
  Serial.print("K0:");
  Serial.println(K0);
  */
  //delay(100);
  
}
/*
void setupACC_ADXL345()
{
  digitalWrite(AccSelectPin, LOW);
  SPI.transfer(WRITECommand_POWER_CTL);
  SPI.transfer(0x08); //normal measure mode
  digitalWrite(AccSelectPin, HIGH);
  delay(10);
  digitalWrite(AccSelectPin, LOW);
  SPI.transfer(WRITECommand_DATA_FORMAT);
  SPI.transfer(0x01); //4g range
  digitalWrite(AccSelectPin, HIGH);
}

void setupGyro_L3G4200D()
{
  digitalWrite(GyroSelectPin, LOW);
  SPI.transfer(WRITECommand_CTRL_REG4);
  SPI.transfer(0b00010000);
  digitalWrite(GyroSelectPin, HIGH);
  delay(10);
  
  digitalWrite(GyroSelectPin, LOW);
  SPI.transfer(WRITECommand_CTRL_REG1);
  SPI.transfer(0b00001111);
  digitalWrite(GyroSelectPin, HIGH);
}

double readAccValue_ADXL345()
{
  digitalWrite(AccSelectPin, LOW);
  SPI.transfer(READCommand_DATA_X0);
  byte xLSB = SPI.transfer(0x00);
  byte xMSB = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);
  int x = ((xMSB << 8) | xLSB);
  x = x+5; //zero bias
  double acc_g = (double)x/161;
  double angel=0;
  double acc_times = 0;
  //consider all kinds of acc value in 4g range.
  if(acc_g>1 && acc_g<=2)
  {
    angel = 90;
    acc_g = acc_g-1;
    acc_times = 1;
  }
  else if(acc_g>2 && acc_g<=3)
  {
    angel = 180;
    acc_g = acc_g-2;
    acc_times = 2;
  }
  else if(acc_g>3 && acc_g<=4)
  {
    angel = 270;
    acc_g = acc_g-3;
    acc_times = 3;
  }
  else if(acc_g>=-2 && acc_g<-1)
  {
    angel = -90;
    acc_g = acc_g+1;
    acc_times = -1;
  }
  else if(acc_g>=-3 && acc_g<-2)
  {
    angel = -180;
    acc_g = acc_g+2;
    acc_times = -2;
  }
  if(acc_g>=-4 && acc_g<-3)
  {
    angel = -270;
    acc_g = acc_g+3;
    acc_times = -3;
  }
  angel = angel + asin(acc_g)*180/PI;
  //Serial.println(xLSB,BIN);
  //Serial.println(xMSB,BIN);
  //Serial.print(x);
 /// Serial.print("\t");
  //Serial.print("ACC(g): ");
  //Serial.print(acc_g + acc_times);
  //Serial.print("\t");
  //Serial.print("Angel(D): ");
  //Serial.println(angel);
  return angel;
}

double readGyroValue_L3G4200D()
{
  digitalWrite(GyroSelectPin, LOW);
  SPI.transfer(READCommand_OUT_Y_L);
  byte yLSB = SPI.transfer(0x00);
  //SPI.transfer(READCommand_OUT_Y_H);
  byte yMSB = SPI.transfer(0x00);
  digitalWrite(GyroSelectPin, HIGH);
  
  int y = ((yMSB << 8) | yLSB);
  double dps = y*0.0175;
  //Serial.println(yLSB,BIN);
  //Serial.println("\n");
  //Serial.println(yMSB,BIN);
  //Serial.println("\n");
  //Serial.println(y,BIN);
  //Serial.println("\n");
  //Serial.println(y);
  //Serial.println("Gyro(dps): ");
  //Serial.print(dps);
  return dps;
}
*/
void setupACC_SCA830()
{
  //read CTRL
  digitalWrite(AccSelectPin, LOW);
  byte head = SPI.transfer(WRITECommand_CTRL);
  byte result = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);
  
}

void setupGyro_ADXRS453Z()
{
  digitalWrite(GyroSelectPin, LOW);
  byte Start_4 = SPI.transfer(Init_4);
  byte Start_3 = SPI.transfer(Init_3); 
  byte Start_2 = SPI.transfer(Init_2); 
  byte Start_1 = SPI.transfer(Init_1); 
  digitalWrite(GyroSelectPin, HIGH);
  delay(100);
    
  digitalWrite(GyroSelectPin, LOW);
  SPI.transfer(ReadInit_4);
  SPI.transfer(ReadInit_3);
  SPI.transfer(ReadInit_2);
  SPI.transfer(ReadInit_1);
  digitalWrite(GyroSelectPin, HIGH);
  delay(100);

  digitalWrite(GyroSelectPin, LOW);
    byte Fault_4 = SPI.transfer(ReadInit_4);
    byte Fault_3 = SPI.transfer(ReadInit_3);
    byte Fault_2 = SPI.transfer(ReadInit_2);
    byte Fault_1 = SPI.transfer(ReadInit_1);
  digitalWrite(GyroSelectPin, HIGH);
    
    delay(100);
  
    digitalWrite(GyroSelectPin, LOW);
    Fault_4 = SPI.transfer(ReadInit_4);
    Fault_3 = SPI.transfer(ReadInit_3);
    Fault_2 = SPI.transfer(ReadInit_2);
    Fault_1 = SPI.transfer(ReadInit_1);
    digitalWrite(GyroSelectPin, HIGH);
    
    delay(100);

    digitalWrite(GyroSelectPin, LOW);
    Fault_4 = SPI.transfer(ReadInit_4);
    Fault_3 = SPI.transfer(ReadInit_3);
    Fault_2 = SPI.transfer(ReadInit_2);
    Fault_1 = SPI.transfer(ReadInit_1);
    digitalWrite(GyroSelectPin, HIGH);
   
}
double readAccValue_SCA830()
{
  digitalWrite(AccSelectPin, LOW);
  byte head_M = SPI.transfer(READCommand_Y_MSB);
  byte MSB = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);

  digitalWrite(AccSelectPin, LOW);
  byte head_L = SPI.transfer(READCommand_Y_LSB);
  byte LSB = SPI.transfer(0x00);
  digitalWrite(AccSelectPin, HIGH);

  short int x = ((MSB << 8) | LSB);
  //x = x+5; //zero bias
  
  double acc_g = (double)x/32000;
  double angel=0;
  double acc_times = 0;
  //consider all kinds of acc value in 4g range.
  if(acc_g>1 && acc_g<=2)
  {
    angel = 90;
    acc_g = acc_g-1;
    acc_times = 1;
  }
  angel = angel + asin(acc_g)*180/PI;
  return angel;
}

double readGyroValue_ADXRS453Z()
{
  digitalWrite(GyroSelectPin, LOW);
  byte DATA0_4 = SPI.transfer(READCommand_RATE_4);
  byte DATA0_3 = SPI.transfer(READCommand_RATE_3);
  byte DATA0_2 = SPI.transfer(READCommand_RATE_2);
  byte DATA0_1 = SPI.transfer(READCommand_RATE_1);
  digitalWrite(GyroSelectPin, HIGH);
  //delay(1);
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
  //Serial.println(yLSB,BIN);
  //Serial.println("\n");
  //Serial.println(yMSB,BIN);
  //Serial.println("\n");
  //Serial.println(y,BIN);
  //Serial.println("\n");
  //Serial.println(y);
  //Serial.println("Gyro(dps): ");
  //Serial.print(dps);
  return dps;
}

double calcGyroBasicBias()
{
  int i=70;
  double basicBias = 0.0;
  while(i>0)
  {
    basicBias += readGyroValue_ADXRS453Z();
    i--;
    delay(1);
  }
  basicBias = basicBias/70;
  Serial.print("Basic Bias:");
  Serial.println(basicBias,4);
  return basicBias;
}

double calcACCBasicAngle()
{
  int i=30;
  double basicAngle = 0.0;
  while(i>0)
  {
    basicAngle += readAccValue_SCA830();
    i--;
  }
  basicAngle = basicAngle/30;
  Serial.print("Basic Angle:");
  Serial.println(basicAngle,4);
  return basicAngle;
}
