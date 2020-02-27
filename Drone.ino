#include <MPU6050_6Axis_MotionApps_V6_12.h>



//MPU LIB example
//https://bit.ly/2SEH0IJ
#include <MPU6050.h>
#include <I2Cdev.h>

#include <Wire.h>

#include <Servo.h>

#include <Pixy2.h>
#include <Pixy2CCC.h>


//fnGetDistance
float duration, distance;


//Pixy
Pixy2 pixy;

int i;
int red = 1;
int blue = 2;

//LED
const int ledRed = 6;
const int ledBlue = 7;
const int ledGreen = 8; 

int on = HIGH;
int off = LOW;

//Servo
Servo servoRed;
Servo servoBlue;
const int servoPinRed = 9;
const int servoPinBlue = 3;

//Sonar
const int trigPin = 4;
const int echoPin = 5;

float lowDistance = 0;
float highDistance = 50;

//Gyro and Accelerometer
MPU6050 mpu;
int INTERRUPT_PIN = 2;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; 




void setup() {
  
  Serial.begin(115200);
  Serial.print("begin");
 
  pinMode(ledRed, OUTPUT); 
  pinMode(ledBlue, OUTPUT); 
  pinMode(ledGreen, OUTPUT); 
  
  pixy.init();
 
  
  servoRed.attach(servoPinRed);
  servoBlue.attach(servoPinBlue);
  
  servoRed.write(5);
  servoBlue.write(5);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  startI2Cdev();
  mpu.initialize();
  
  pinMode(INTERRUPT_PIN, INPUT);
  
  devStatus = mpu.dmpInitialize();
  
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  
  
  if (devStatus == 0) {
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  
  mpu.setDMPEnabled(true);
  

  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
  
  dmpReady = true;
  
  
  }else{
  Serial.print(F("DMP Initialization failed (code "));
  }
  
  getGyro();
  Serial.println("Initial Gyro Values:");
  printGyro();
  
}

void loop() {
  uint16_t blocks;
  
  
  blocks = pixy.ccc.getBlocks();
  delay(1000);
  getGyro();
  printGyro();
  
  if(blocks){
    
  for (i = 0; i< blocks;i++){
    if(pixy.ccc.blocks[i].m_signature==red){
      getDistance();
      pulseLED(ledRed, on);
//      Serial.println(getDistance());
      
      if(getDistance() <= highDistance && getDistance() >= lowDistance){
          pulseLED(ledGreen, on);
          float yaw = ypr[0] * 180 / M_PI;
          float pitch = ypr[2] * 180 / M_PI;
          float roll = ypr[1] * 180 / M_PI;
          
          if(roll <= 3 && roll >= -3 && pitch <= 3 && pitch >= -3){
          servoRed.write(180);
            }
      }else{
          pulseLED(ledGreen, off);
        }
      }
    else if(pixy.ccc.blocks[i].m_signature==blue){
      getDistance();
      pulseLED(ledBlue, on);
      
     if(getDistance() <= highDistance && getDistance() >= lowDistance){
       pulseLED(ledGreen, on);
       float yaw = ypr[0] * 180 / M_PI;
       float pitch = ypr[2] * 180 / M_PI;
       float roll = ypr[1] * 180 / M_PI;
       
       if(roll <= 3 && roll >= -3 && pitch <= 3 && pitch >= -3){
          servoBlue.write(180);
          pulseLED(ledBlue, on);
        }
       }else{
          pulseLED(ledGreen, off);
       
            }
          } 
        }
    } else {
         pulseLED(ledRed, off);
         pulseLED(ledBlue, off);
         pulseLED(ledGreen, off);
         Serial.println("No Signature");
      }
  }       

float getDistance(){

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) * .0343; //equation to convert to cm
    return distance;
  }

  void printDistance(){
  Serial.print("Distnace: "); //Range is 2 to 400 cm
  Serial.println(distance);
  }

void pulseLED(int LED, int state){
  
  digitalWrite(LED,state);
  }



void startI2Cdev(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
}

void getGyro(){
 delay(100); //gives time to allow Gyro to buffer
  if (!dmpReady) return;
    Serial.print("not ready");
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, fifoBuffer);
  }
}

void printGyro(){
   float yaw = ypr[0] * 180 / M_PI;
   float pitch = ypr[2] * 180 / M_PI;
   float roll = ypr[1] * 180 / M_PI;  
   Serial.println("[Yaw, Pitch, Roll]\t");
   Serial.print(yaw);
   Serial.print("\t");
   Serial.print(pitch);
   Serial.print("\t");
   Serial.println(roll);   
   
}
void dmpDataReady() {
    mpuInterrupt = true;
}
