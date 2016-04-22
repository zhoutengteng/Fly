#include<string.h>
char *setName = "AT+NAME=zhoutengteng\r\n";
char *setPassword = "AT+PSWD=123456\r\n";
char *setUART = "AT+UART=38400,0,0\r\n";

int port = 40;

void setup() {
  Serial1.begin(38400);
  writeName();
  writePassword();
  writeUart();
}

void writeName() {
    for (int i = 0; i < strlen(setName); i++) {
        Serial1.write(setName[i]);  
    }
}

void writePassword() {
    for (int i = 0; i < strlen(setPassword); i++) {
        Serial1.write(setPassword[i]);  
    }
}

void writeUart() {
    for (int i = 0; i < strlen(setUART); i++) {
        Serial1.write(setUART[i]);  
    }
}



void loop() {
  /*
        while (Serial1.available()) {
            char c = Serial1.read();
            if (c == 'a') {
                //digitalWrite(port, HIGH);  
                Serial1.write("你输入的是a");
                
            } else if (c == 'b') {
                //digitalWrite(port, LOW);
                Serial1.write("你输入的是b");
            }
          }
    */
    /*
    if (Serial.available()) {
        while (Serial.available()) {
            char c = Serial.read();
            Serial.write("==>");
            Serial.write(c);
          }
    }
    */
}
