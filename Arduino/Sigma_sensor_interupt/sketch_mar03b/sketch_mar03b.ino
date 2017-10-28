#include <math.h>
#include <SPI.h>

const byte READXCommand = 0b00010000;
const byte READYCommand = 0b00010001;
const int chipSelectPin = 10;

int pbIn = 0;                  // 定义中断引脚为0，也就是D2引脚
volatile int state = LOW;      // 定义默认输入状态
int counter = 0; 
void setup() {
  // put your setup code here, to run once:
  // 监视中断输入引脚的变化
  Serial.begin(9600);
  attachInterrupt(pbIn, stateChange, RISING);

  //SPI读取初始化
  pinMode(chipSelectPin, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE0);
}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(1000);
  //Serial.print(millis());
  //Serial.print(":");
  //Serial.println(counter);
}

void stateChange()
{
  counter = counter+1;
  //Serial.print(millis());
  //Serial.print(":");
  //Serial.println(counter);
  
  //Serial.println("123456789");

  unsigned int MSB = 0;
  unsigned int LSB = 0;
  
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(READXCommand);
  MSB = SPI.transfer(0x000);
  LSB = SPI.transfer(0x00);
  digitalWrite(chipSelectPin, HIGH);
  MSB = MSB<<3;
  LSB = LSB>>5;
  MSB=MSB|LSB;
  Serial.println(MSB);
  //Serial.println(MSB,BIN);
}
