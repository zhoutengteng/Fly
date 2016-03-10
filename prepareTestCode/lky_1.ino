//灯的输出口是digital的32号
int ledpin = 32;
int on = 255;
int off = 0;

void setup() {
  // put your setup code here, to run once:
  //已经将蓝牙的波特率设为9600了
     Serial.begin(9600);
     pinMode(ledpin, OUTPUT);
}

//把程序安装到ARDUINO中的时候，先断开蓝牙模块，因为他们的数据都走串口，会引起冲突造成安装失败
void loop() {
  // put your main code here, to run repeatedly:
    while(Serial.available()) {
      char c = Serial.read();
      //输入的字符为A时候，灯亮
      if (c == 'A') {
        Serial.println("Hello I am arduino");
        digitalWrite(ledpin, on);
        delay(1000);
      }
      //否则灯灭
     else {
        digitalWrite(ledpin, off);
         delay(1000);
      }
    }
}
