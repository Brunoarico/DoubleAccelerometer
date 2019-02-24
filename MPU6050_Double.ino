#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define ACEL_N 2

int ACEL_PIN[] = {5, 6};

MPU6050 mpu;

bool blinkState = false;

// MPU control/status vars

bool dmpReady[ACEL_N] = {false, false};  // set true if DMP init was successful
uint8_t mpuIntStatus[ACEL_N];   // holds actual interrupt status byte from MPU
uint8_t devStatus[ACEL_N];      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

uint16_t fifoCount[ACEL_N];     // count of all bytes currently in FIFO
uint8_t fifoBuffer[ACEL_N][64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void init_conf(int num) {
  Serial.println("Initializing Module " + String(num));
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus[num] = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); 

  if (devStatus[num] == 0) {
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      mpuIntStatus[num] = mpu.getIntStatus();
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady[num] = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else Serial.println("DMP Initialization failed (code "+ String(devStatus[num])+" ) ");
}

void setup() {

    Wire.begin();
    Wire.setClock(400000); 
    Serial.begin(115200);
    while (!Serial);
    
    for(int i = 0; i < ACEL_N; i++) {
      pinMode(ACEL_PIN[i], OUTPUT);
      digitalWrite(ACEL_PIN[i],HIGH);
    }
    
    for(int i = 0; i < ACEL_N; i++) {
      digitalWrite(ACEL_PIN[i], LOW);
      init_conf(i);
      digitalWrite(ACEL_PIN[i], HIGH);
    }

}

void getvalues(int num) {
  digitalWrite(ACEL_PIN[num], LOW);
  if (!dmpReady[num]) return;
    while (fifoCount[num] < packetSize) {
          fifoCount[num] = mpu.getFIFOCount();
    }
    mpuIntStatus[num] = mpu.getIntStatus();
    fifoCount[num] = mpu.getFIFOCount();
    if ((mpuIntStatus[num] & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount[num] >= 1024) {
        mpu.resetFIFO();
        fifoCount[num] = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus[num] & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        while (fifoCount[num] < packetSize) fifoCount[num] = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer[num], packetSize);
        fifoCount[num] -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer[num]);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        Serial.print("ypr " + String(num) + "\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[2] * 180/M_PI);
        Serial.print("\t");
    }
    digitalWrite(ACEL_PIN[num], HIGH);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  for(int i = 0; i < ACEL_N; i++) getvalues(i);
  Serial.println();
    
}
