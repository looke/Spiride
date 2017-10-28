//#include <math.h>
#include <SPI.h>
#include "L3G4200D_Command.h"
#include "ADXL345_Command.h"
#include "BasicKalman.h"

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



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //SPI读取初始化
  //pinMode(AccSelectPin, OUTPUT);
  //pinMode(GyroSelectPin, OUTPUT);

  //SPI读取初始化
  SPI.begin(AccSelectPin);
  SPI.begin(GyroSelectPin);
  
  SPI.setBitOrder(AccSelectPin,MSBFIRST);
  SPI.setBitOrder(GyroSelectPin,MSBFIRST);
  
  SPI.setClockDivider(AccSelectPin,11);
  SPI.setClockDivider(GyroSelectPin,11);
  
  SPI.setDataMode(AccSelectPin,SPI_MODE3);
  SPI.setDataMode(GyroSelectPin,SPI_MODE3);

  setupACC();
  setupGyro();
  
  delay(1000); //give device time to init
  bk.setAngle(0);
  pre_degree=0;
  timer = micros();
}

void loop() 
{
  // put your main code here, to run repeatedly:
  degree = readAccXValue();
  rate = readYGyroValue();
  dt = double(micros() - timer)/1000000;
  pre_degree = pitch + rate*dt;
  //pitch = bk.getAngle(degree,rate,dt);  // Calculate the angle using the Kalman filter

  
  timer = micros(); 

  //bias = bk.getBias();
  //K0=bk.getK0();
  pitch = 0;
  bias = 0;
  K0 = 0;
  
//  String outputContent;
  
//  String str_degree = String(degree);
//  String str_rate = String(rate);
//  String str_dt = String(dt);
//  String str_pitch = String(pitch);
//  String str_bias = String(bias);
//  String str_K0 = String(K0);

//  outputContent = "degree:"+str_degree + ";rate:" + str_rate + ";dt:" + str_dt + ";pitch:" + str_pitch + ";bias:" + str_bias + ";K0:" + str_K0;
  
  Serial.println(dt);
  /*
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

void setupACC()
{
  SPI.transfer(AccSelectPin,WRITECommand_POWER_CTL,SPI_CONTINUE);
  SPI.transfer(AccSelectPin,0x08); //normal measure mode
  delay(10);
  SPI.transfer(AccSelectPin,WRITECommand_DATA_FORMAT,SPI_CONTINUE);
  SPI.transfer(AccSelectPin,0x01); //4g range
}

void setupGyro()
{
  SPI.transfer(GyroSelectPin,WRITECommand_CTRL_REG4,SPI_CONTINUE);
  SPI.transfer(GyroSelectPin,0b00010000);
  delay(10);
  SPI.transfer(GyroSelectPin,WRITECommand_CTRL_REG1,SPI_CONTINUE);
  SPI.transfer(GyroSelectPin,0b00001111);
}

double readAccXValue()
{
  SPI.transfer(AccSelectPin,READCommand_DATA_X0,SPI_CONTINUE);
  byte xLSB = SPI.transfer(AccSelectPin,0x00,SPI_CONTINUE);
  byte xMSB = SPI.transfer(AccSelectPin,0x00);
  
  short int x = ((xMSB << 8) | xLSB);
  x = x+5; //zero bias
  double acc_g = (double)x/151;
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

double readYGyroValue()
{
  SPI.transfer(GyroSelectPin,READCommand_OUT_Y_L,SPI_CONTINUE);
  byte yLSB = SPI.transfer(GyroSelectPin,0x00,SPI_CONTINUE);
  byte yMSB = SPI.transfer(GyroSelectPin,0x00);
  
  short int y = ((yMSB << 8) | yLSB);
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
