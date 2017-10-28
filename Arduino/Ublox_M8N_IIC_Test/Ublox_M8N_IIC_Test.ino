#include <Wire.h>

int reading = 0;

//close the output of GGA(00)
byte GGA[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F};

//close the output of GLL(01)
byte GLL[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};

//close the output of GSA(02)
byte GSA[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13};

//close the output of GSV(03)
byte GSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};

//close the output of RMC(04)
byte RMC[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17};

//close the output of VTG(05)
byte VTG[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19};

byte re;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(9600);
  delay(5000);
  Serial.println("Start Close NMEA messages.");
  
  Wire.beginTransmission(0x42);
  Wire.write(GGA , 11);
  re = Wire.endTransmission();
  Serial.print("GGA result:");
  Serial.println(re);

  Wire.beginTransmission(0x42);
  Wire.write(GLL , 11);
  re = Wire.endTransmission();
  Serial.print("GLL result:");
  Serial.println(re);

  Wire.beginTransmission(0x42);
  Wire.write(GSA , 11);
  re = Wire.endTransmission();
  Serial.print("GSA result:");
  Serial.println(re);

  Wire.beginTransmission(0x42);
  Wire.write(GSV , 11);
  re = Wire.endTransmission();
  Serial.print("GSV result:");
  Serial.println(re);

  Wire.beginTransmission(0x42);
  Wire.write(VTG , 11);
  re = Wire.endTransmission();
  Serial.print("VTG result:");
  Serial.println(re);

  delay(1000);
  Serial.println("Start LOOP.");
}

void loop() {
  // put your main code here, to run repeatedly:
  //Wire.beginTransmission(0x42); //m8n default addr 0x42(last bit --> W)
  //Wire.write(byte(0xFF));
  //Serial.println("Write 0xFF.");
  //byte re = Wire.endTransmission();
  //Serial.println(re,HEX);
  //Serial.println("Request.");
  Wire.requestFrom(0x42, 50);

  while (Wire.available() > 0)
  {
    byte data = Wire.read();
    Serial.print(data, HEX);
    Serial.print(" ");
  }
  Serial.println("end");
  Serial.println("----------");
  delay(100);
}
