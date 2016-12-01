#include <math.h>
#include <SPI.h>
#include "LIS3MDL.h"
const int MagSelectPin = 10;

boolean isMagNormalized = false;
int Mag_X_Max = 0;
int Mag_X_Min = 0;
int Mag_X_Range = 0;
int Mag_X_ZP = 0;

int Mag_Y_Max = 0;
int Mag_Y_Min = 0;
int Mag_Y_Range = 0;
int Mag_Y_ZP = 0;

int Mag_Z_Max = 0;
int Mag_Z_Min = 0;
int Mag_Z_Range = 0;
int Mag_Z_ZP = 0;

void setup() {
  // put your setup code here, to run once:
  // put your setup code here, to run once:
  Serial.begin(115200);
  //SPI读取初始化
  pinMode(MagSelectPin, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setDataMode(SPI_MODE0);
  delay(200);
  setupMag();
  delay(1000);
  
  Serial.println("Mag Normalization X Start.");
  Mag_Normalization();
  showNormalizationResult();

  Serial.println("Mag Normalization Y Start.");
  Mag_Normalization();
  showNormalizationResult();

  Serial.println("Mag Normalization Z Start.");
  Mag_Normalization();
  showNormalizationResult();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //read_WHO_AM_I();
  //read_CTRL_REG_1();
  //read_CTRL_REG_2();
  //read_CTRL_REG_3();
  //read_CTRL_REG_4();
  //read_CTRL_REG_5();
  
  readDATA_XYZ();
  read_Status();
  delay(1000);
}
void setupMag()
{
  set_CTRL_REG_1();
  set_CTRL_REG_2();
  set_CTRL_REG_3();
  set_CTRL_REG_4();
  set_CTRL_REG_5();
  
}

void read_WHO_AM_I()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_WHO_AM_I);
  byte WHO = SPI.transfer(0x00);
  digitalWrite(MagSelectPin, HIGH);
  Serial.print("Who Am I:");
  Serial.println(WHO,BIN);
}

void read_CTRL_REG_1()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_CTRL_REG1);
  byte reg = SPI.transfer(0x00);
  digitalWrite(MagSelectPin, HIGH);
  Serial.print("CTRL Reg 1:");
  Serial.println(reg,BIN);
}

void set_CTRL_REG_1()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(WRITECommand_CTRL_REG1);
  byte reg = SPI.transfer(0xDC); //Temperature sensor:Enable{1} / X and Y axes operative mode:  high Performance{10} /Output data rate:80Hz{111} / self-test:disable{0}
  digitalWrite(MagSelectPin, HIGH);
}

void read_CTRL_REG_2()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_CTRL_REG2);
  byte reg = SPI.transfer(0x00);
  digitalWrite(MagSelectPin, HIGH);
  Serial.print("CTRL Reg 2:");
  Serial.println(reg,BIN);
}

void set_CTRL_REG_2()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(WRITECommand_CTRL_REG2);
  byte reg = SPI.transfer(0x00); //Full-scale:4 guass{00} / Reboot memory mode: Normal{0} /register reset:Default{0}
  digitalWrite(MagSelectPin, HIGH);
}

void read_CTRL_REG_3()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_CTRL_REG3);
  byte reg = SPI.transfer(0x00);
  digitalWrite(MagSelectPin, HIGH);
  Serial.print("CTRL Reg 3:");
  Serial.println(reg,BIN);
}

void set_CTRL_REG_3()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(WRITECommand_CTRL_REG3);
  byte reg = SPI.transfer(0x00); //Low-power mode:disable{0} / SPI mode:4 wire{0} /Operating mode:Continuous{00}
  digitalWrite(MagSelectPin, HIGH);
}

void read_CTRL_REG_4()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_CTRL_REG4);
  byte reg = SPI.transfer(0x00);
  digitalWrite(MagSelectPin, HIGH);
  Serial.print("CTRL Reg 4:");
  Serial.println(reg,BIN);
}

void set_CTRL_REG_4()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(WRITECommand_CTRL_REG4);
  byte reg = SPI.transfer(0x08); //Z-axis operative mode:high Performance{10} / Big/Little Endian data selection: LSB lower{0}
  digitalWrite(MagSelectPin, HIGH);
}

void read_CTRL_REG_5()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_CTRL_REG5);
  byte reg = SPI.transfer(0x00);
  digitalWrite(MagSelectPin, HIGH);
  Serial.print("CTRL Reg 5:");
  Serial.println(reg,BIN);
}
void set_CTRL_REG_5()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(WRITECommand_CTRL_REG5);
  byte reg = SPI.transfer(0x00); //Block data update:continuous update{0}
  digitalWrite(MagSelectPin, HIGH);
}

void readDATA_XYZ()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_DATA_ALL);
  byte X_LSB = SPI.transfer(0x00);
  byte X_MSB = SPI.transfer(0x00);

  byte Y_LSB = SPI.transfer(0x00);
  byte Y_MSB = SPI.transfer(0x00);
  
  byte Z_LSB = SPI.transfer(0x00);
  byte Z_MSB = SPI.transfer(0x00);
  digitalWrite(MagSelectPin, HIGH);

  short int x = ((X_MSB << 8) | X_LSB);
  short int y = ((Y_MSB << 8) | Y_LSB);
  short int z = ((Z_MSB << 8) | Z_LSB);

  float x_normal = x-Mag_X_Min;
  x_normal = x_normal/Mag_X_ZP -1;
  
  float y_normal = y-Mag_Y_Min;
  y_normal = y_normal/Mag_Y_ZP -1;
  
  float z_normal = z-Mag_Z_Min;
  z_normal = z_normal/Mag_Z_ZP -1;

  
  //double x_gauss = (double)x/6842;
  //double y_gauss = (double)y/6842;
  //double z_gauss = (double)z/6842;

  
  Serial.print("X:");
  Serial.print(x);
  Serial.print("\t");
  Serial.print("X Normal:");
  Serial.print(x_normal);
  Serial.print("\t");
  
  Serial.print("Y:");
  Serial.print(y);
  Serial.print("\t");
  Serial.print("Y Normal:");
  Serial.print(y_normal);
  Serial.print("\t");
  
  Serial.print("Z:");
  Serial.print(z);
  Serial.print("\t");
  Serial.print("Z Normal:");
  Serial.println(z_normal);

  
  /*
  Serial.print("X:");
  Serial.print(X_MSB,BIN);
  Serial.print("\t");
  Serial.println(X_LSB,BIN);

  Serial.print("Y:");
  Serial.print(Y_MSB,BIN);
  Serial.print("\t");
  Serial.println(Y_LSB,BIN);

  Serial.print("Z:");
  Serial.print(Z_MSB,BIN);
  Serial.print("\t");
  Serial.println(Z_LSB,BIN);
  */
  Serial.println("--------------------");
}

void read_Status()
{
  digitalWrite(MagSelectPin, LOW);
  SPI.transfer(READCommand_STATUS_REG);
  byte reg = SPI.transfer(0x00);
  digitalWrite(MagSelectPin, HIGH);
  Serial.print("Status:");
  Serial.println(reg,BIN);
}

void Mag_Normalization()
{
  if(!isMagNormalized)
  {
    digitalWrite(MagSelectPin, LOW);
    SPI.transfer(READCommand_DATA_ALL);
    byte X_LSB = SPI.transfer(0x00);
    byte X_MSB = SPI.transfer(0x00);

    byte Y_LSB = SPI.transfer(0x00);
    byte Y_MSB = SPI.transfer(0x00);
  
    byte Z_LSB = SPI.transfer(0x00);
    byte Z_MSB = SPI.transfer(0x00);
    digitalWrite(MagSelectPin, HIGH);

    short int x = ((X_MSB << 8) | X_LSB);
    short int y = ((Y_MSB << 8) | Y_LSB);
    short int z = ((Z_MSB << 8) | Z_LSB);

    Mag_X_Max = x;
    Mag_X_Min = x;

    Mag_Y_Max = y;
    Mag_Y_Min = y;

    Mag_Z_Max = x;
    Mag_Z_Min = z;

    isMagNormalized = true;
  }
  
  unsigned long timer = micros();
  int printTime = 0;
  
  while(1)
  {
    digitalWrite(MagSelectPin, LOW);
    SPI.transfer(READCommand_DATA_ALL);
    byte X_LSB = SPI.transfer(0x00);
    byte X_MSB = SPI.transfer(0x00);

    byte Y_LSB = SPI.transfer(0x00);
    byte Y_MSB = SPI.transfer(0x00);
  
    byte Z_LSB = SPI.transfer(0x00);
    byte Z_MSB = SPI.transfer(0x00);
    digitalWrite(MagSelectPin, HIGH);

    short int x = ((X_MSB << 8) | X_LSB);
    short int y = ((Y_MSB << 8) | Y_LSB);
    short int z = ((Z_MSB << 8) | Z_LSB);

    
    if(x > Mag_X_Max)
    {
      Mag_X_Max = x;
    }
    if(x < Mag_X_Min)
    {
      Mag_X_Min = x;
    }

    if(y > Mag_Y_Max)
    {
      Mag_Y_Max = y;
    }
    if(y < Mag_Y_Min)
    {
      Mag_Y_Min = y;
    }

    if(z > Mag_Z_Max)
    {
      Mag_Z_Max = z;
    }
    if(z < Mag_Z_Min)
    {
      Mag_Z_Min = z;
    }
     unsigned long currentTime = micros();
     int eclipseTime = (currentTime - timer)/1000000;
     if(eclipseTime > 30)
     {
        break;
     }
     if(eclipseTime > printTime)
     {
        printTime++;
        Serial.println(printTime);
     }
     delay(8);
  }

  Mag_X_Range = Mag_X_Max- Mag_X_Min;
  Mag_X_ZP = Mag_X_Range/2;

  Mag_Y_Range = Mag_Y_Max- Mag_Y_Min;
  Mag_Y_ZP = Mag_Y_Range/2;

  Mag_Z_Range = Mag_Z_Max- Mag_Z_Min;
  Mag_Z_ZP = Mag_Z_Range/2;
}

void showNormalizationResult()
{
  Serial.print("X Max:");
  Serial.print(Mag_X_Max);
  Serial.print("\t");
  Serial.print("X Min:");
  Serial.print(Mag_X_Min);
  Serial.print("\t");
  Serial.print("X Range:");
  Serial.print(Mag_X_Range);
  Serial.print("\t");
  Serial.print("X Zero Point:");
  Serial.println(Mag_X_ZP);
  
  
  Serial.print("Y Max:");
  Serial.print(Mag_Y_Max);
  Serial.print("\t");
  Serial.print("Y Min:");
  Serial.print(Mag_Y_Min);
  Serial.print("\t");
  Serial.print("Y Range:");
  Serial.print(Mag_Y_Range);
  Serial.print("\t");
  Serial.print("Y Zero Point:");
  Serial.println(Mag_Y_ZP);
  
  Serial.print("Z Max:");
  Serial.print(Mag_Z_Max);
  Serial.print("\t");
  Serial.print("Z Min:");
  Serial.print(Mag_Z_Min);
  Serial.print("\t");
  Serial.print("Z Range:");
  Serial.print(Mag_Z_Range);
  Serial.print("\t");
  Serial.print("Z Zero Point:");
  Serial.println(Mag_Z_ZP);
  
}
