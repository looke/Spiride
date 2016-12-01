int pbIn = 0;                  // 定义中断引脚为0，也就是D2引脚
volatile int state = LOW;      // 定义默认输入状态
int counter = 0; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  attachInterrupt(pbIn, stateChange, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void stateChange()
{
  counter = counter+1;
  Serial.println(counter);
}
