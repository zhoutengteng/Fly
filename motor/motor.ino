#include <Servo.h> 

Servo myservo;

int pos = 0;

void setup() 
{ 
 myservo.attach(9);
} 


void loop() 
{ 
  myservo.write(150);
  delay(3000);
  myservo.write(20);
  delay(3000);
  
// if(pos <= 200)
// {
//   for(pos = 0; pos < 200; pos += 20)
//   {
//     myservo.write(pos);
//     delay(100);
//   }
// }
// else
// {
//   int pos = 50;
// }
}
