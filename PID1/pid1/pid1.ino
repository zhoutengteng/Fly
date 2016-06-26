#include "I2Cdev.h"
#include "MPU6050_9Axis_MotionApps41.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



#include <SoftwareSerial.h>
#include <Servo.h> 

Servo motor0;//front-left
Servo motor1;//front-right
Servo motor2;//back-left
Servo motor3;//back-right

SoftwareSerial BTSerial(10, 11); // RX | TX

int angle[2] = {0,0};
int tarangle[2] = {0,0};
int spe[4] = {40,40,40,40};
float PV = 0.3;
float IV = 0.002;
float DV = 1;

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);
  motor0.attach(9);
  motor1.attach(6);
  motor2.attach(5);
  motor3.attach(3); 
  delay(4000);

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    Serial.println(("Initializing I2C devices..."));
    mpu.initialize();
    mpu.setIntI2CMasterEnabled(0);
    mpu.setI2CBypassEnabled(1);
    if((!mpu.getI2CMasterModeEnabled()) && mpu.getI2CBypassEnabled()) {
        Serial.println("Set MPU6000 Bypass Mode success!\n");
    }
    pinMode(INTERRUPT_PIN, INPUT);
    // verify connection
    Serial.println(("Testing device connections..."));
    Serial.println(mpu.testConnection() ? ("MPU6050 connection successful") : ("MPU6050 connection failed"));
    // wait for ready
   // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //  while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());    // wait for data
    delay(5000);
 //   while (Serial.available() && Serial.read()); // empty buffer again
    // load and configure the DMP
    Serial.println(("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        Serial.println(("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        motor0.write(40);
        motor1.write(40);
        motor2.write(40);
        motor3.write(40);

    } else {
        Serial.print(("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println((")"));
    }


}


void getangles();


int er = 0;
int eri = 0;
int erd = 0;
int laster = 0;
int P,I,D = 0;

void loop() {
  unsigned char charreceived = BTSerial.read(); 
     switch(charreceived){
      case '1':  //speed up
        spe[0]++;
        spe[1]++;
        spe[2]++;
        spe[3]++;
        Serial.println("get 1 , speed : ");
        motor0.write(spe[0]);
        motor1.write(spe[1]);
        motor2.write(spe[2]);
        motor3.write(spe[3]);

       // Serial.println(spe);
        break;
      case '2': //speed down
        spe[0]--;
        spe[1]--;
        spe[2]--;
        spe[3]--;
        Serial.print("get 2 , speed : ");
        motor0.write(spe[0]);
        motor1.write(spe[1]);
        motor2.write(spe[2]);
        motor3.write(spe [3]);

        //Serial.println(spe);
      break; 
      case '0': //shutdown
         motor0.write(0);
         motor1.write(0);
         motor2.write(0);
         motor3.write(0);
         while(1){}
         Serial.println("Shutdown!"); 
         break; 
      default: 
         break; 
      }
  Serial.flush();
  
  getangles();//compute 2 angles add put them in angles[], roll and pitch
  
  //PID
  int pid[3] = {0,0,0};
  for(int i=0;i<2;i++){
    er = tarangle[i] - angle[i];
    P = PV*er;
    eri = eri*4/5 + er;
    I = IV*eri;
    erd = er - laster;
    laster = er;
    D = DV*erd;
    pid[i] = (int)(P+I+D);
  }

  //motor0.write(spe[0]-pid[0]-pid[1]);
  //motor1.write(spe[1]+pid[0]-pid[1]);
  //motor2.write(spe[2]-pid[0]+pid[1]);
  //motor3.write(spe[3]+pid[0]+pid[1]);
  
}




void getangles() {

    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            /*Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);/*/
            angle[0] = ypr[2];
            angle[1] = ypr[1];
        #endif
   }

  
  
 }

 
