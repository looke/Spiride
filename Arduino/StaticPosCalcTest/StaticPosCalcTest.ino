#include <math.h>
#include <SPI.h>
#include "StaticPosCalc.h"
float acc_180z[3];
float mag_180z[3];
float qStart_180z[4];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Mag传感器地磁倾角
  float mag_Sensor_b_Inclination = 52 *PI/180;

  //横滚角
  float acc_roll = 5 * PI/180;
  float mag_Sensor_b_y = cos(mag_Sensor_b_Inclination - acc_roll);
  float mag_Sensor_b_z = sin(mag_Sensor_b_Inclination - acc_roll);
  mag_Sensor_b_z = 0-mag_Sensor_b_z;
  float mag_Sensor_b_x = 0;

  float acc_y = sin(acc_roll);
  float acc_z = cos(acc_roll);
  acc_z = 0 - acc_z;

  acc_180z[0] = 0;
  acc_180z[1] = acc_y;
  acc_180z[2] = acc_z;
  
  mag_180z[0] = mag_Sensor_b_x;
  mag_180z[1] = mag_Sensor_b_y;
  mag_180z[2] = mag_Sensor_b_z;
    

  //初始位置位于y轴转N度
  float y_rotate = 11*PI/180;
  float Q0 = cos(y_rotate/2);
  float Q2 = sin(y_rotate/2);
  Q2 = 0-Q2;
  qStart_180z[0] = Q0;
  qStart_180z[1] = 0;
  qStart_180z[2] = Q2;
  qStart_180z[3] = 0;

  delay(1);
  
  unsigned long startTime;
  unsigned long endTime;
  StaticPosCalc myStaticPos(acc_180z, mag_180z, qStart_180z);
  myStaticPos.setPrecision(0.02);
  startTime = micros();
  //myStaticPos.setPrecision(0.07);
  myStaticPos.test();
  endTime = micros();

  Serial.print("Start:");
  Serial.println(startTime);
  Serial.print("End:");
  Serial.println(endTime);
}

void loop() {
  // put your main code here, to run repeatedly:

}
