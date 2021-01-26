//=============================================================================
// i2cdevlib and MPU6050 are needed: https://github.com/jrowberg/i2cdevlib
// Big thanks to the amazing Jeff Rowberg :D, go check his repo to learn about more MPU6050.
//=============================================================================

// I2Cdev and MPU6050 must be installed as libraries

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2
#define _DEVICE_ID "left_controller" // should be one of "headset", "right_controller", "left_controller"

// IMU status and control:
bool dmpReady = false;  // true if DMP init was successful
uint8_t mpuIntStatus;
uint8_t devStatus;      // 0 = success, !0 = error
uint8_t fifoBuffer[64];

Quaternion q;           // [w, x, y, z]

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// hobo_vr communication
float to_be_sent[19] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // [w, x, y, z, ax, ay, az, gx, gy, gz, trgr, padx, pady, pad_clk, trgr_clk, sys, menu, grip, util]
// ^^^ le paket

void setup() {

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(1000000); // 1mHz I2C clock. change this to 400kHz if you have mpu comunication issues
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // serial to display data
  Serial.begin(115200);
  while (!Serial) {}
    Serial.println(_DEVICE_ID);
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    //If you are getting "SerialUSB was not declared" when compiling, change
    //the following line to Serial.println(.......)
    Serial.println(mpu.testConnection() ? F(_DEVICE_ID) : F("MPU6050 connection failed"));

    // configure the DMP
    devStatus = mpu.dmpInitialize();

    // ==================================
    // supply your own gyro offsets here:
    // ==================================
    // follow procedure here to calibrate offsets in the i2cdevlib's calibration examples
    mpu.setXGyroOffset(106);
    mpu.setYGyroOffset(5);
    mpu.setZGyroOffset(43);
    mpu.setZAccelOffset(908);
    mpu.setXAccelOffset(-3960);
    mpu.setYAccelOffset(2373);

    // devStatus if everything worked properly
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //digitalPinToInterrupt(INTERRUPT_PIN);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;
    } else {
        Serial.println("dead");
        while(1){}

    }

    // controller inputs
    // there is also 2 analog inputs on A1 and A2
    pinMode(3, INPUT); //trigger
    pinMode(9, INPUT); //pad click
    pinMode(7, INPUT); //sys
    pinMode(8, INPUT); //menu
    pinMode(4, INPUT); //grip
    pinMode(5, INPUT); //util
}

void loop() {
    if (Serial.available() > 0) {
      switch(Serial.read()) { 
      case 'q': 
          Serial.println(_DEVICE_ID);
          break; 
      default:
          break;
      }
    }
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

        mpu.dmpGetQuaternion(&q, fifoBuffer);


        to_be_sent[0] = q.w;
        to_be_sent[1] = q.x;
        to_be_sent[2] = q.y;
        to_be_sent[3] = q.z;
    }
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // raw accel and gyro readings
    // accel readings converted to g
    to_be_sent[4] = (float)ax/16384;
    to_be_sent[5] = (float)ay/16384;
    to_be_sent[6] = (float)az/16384;
    // gyro readings converted to degrees/second
    to_be_sent[7] = (float)gx/131;
    to_be_sent[8] = (float)gy/131;
    to_be_sent[9] = (float)gz/131;

    //controller inputs
    to_be_sent[10] = !digitalRead(3); //trigger
    to_be_sent[11] = analogRead(A2); //pad x
    to_be_sent[12] = analogRead(A1); //pad y
    to_be_sent[13] = !digitalRead(9); //pad click
    // no trigger click :(
    to_be_sent[15] = !digitalRead(7); //sys
    to_be_sent[16] = !digitalRead(8); //menu
    to_be_sent[17] = !digitalRead(4); //grip
    to_be_sent[18] = !digitalRead(5); //util


    // send the packet
    Serial.write((char*)to_be_sent, sizeof(float)*19);
    Serial.write("\t\r\n");
}
