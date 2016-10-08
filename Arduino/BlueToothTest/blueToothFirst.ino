
#include <MsTimer2.h> 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  MsTimer2::set(3000, report);        // 中断设置函数，每 10000ms 进入一次中断
  MsTimer2::start();                //开始计时
}

void report()
{                        
  Serial.println("123456789");
  Serial.println("123456789");
}


void loop() 
{
  
}
