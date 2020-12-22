#include "MPU9250.h"    // https://github.com/bolderflight/MPU9250  
#include "SensorFusion.h" // Note: currently requires making q0,q1,q2,q3 public
#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

SF fusion;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU9250 IMU(Wire, 0x68);
float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float deltat;

#define _DEVICE_ID "right_controller" // should be one of "headset", "right_controller", "left_controller"
#define _DEVICE_UID "22222" //A per-device UIDto prevent name conflicts between multiple headsets
#define _BT_PIN_CODE "000000" //000000 allows anything to connect. Other codes will add security.
// IMU status and control:
uint8_t mpuIntStatus;

// hobo_vr communication
float to_be_sent[13] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // [w, x, y, z, trgr, padx, pady, pad_clk, trgr_clk, sys, menu, grip, util]
// ^^^ le paket

const int joyx_pin = A0;
const int joyy_pin = A1;
const int joy_clk_pin = 8;
const int trgr_pin = A2;
const int sys_pin = 2;
const int menu_pin = 7;
const int grip_pin = 5;
const int util_pin = 6;

inline void readback_myserial(String& the_str){
  if (mySerial.available()) {    
    while(mySerial.available()) { 
      // While there is more to be read, keep reading.      
      the_str += (char)mySerial.read();    
      }    
    }
}

void setup() {
  //increase baud rate for slow HM-10 or HM-08 bluetooth devices
  //If the baud rate is already high, this will just be a nonsense string

  Serial.begin(115200);
  while (!Serial) {}

  pinMode(joyx_pin,INPUT);
  pinMode(joyy_pin,INPUT);
  pinMode(joy_clk_pin,INPUT);
  pinMode(trgr_pin,INPUT);
  pinMode(sys_pin,INPUT);
  pinMode(menu_pin,INPUT);
  pinMode(grip_pin,INPUT);
  pinMode(util_pin,INPUT);
  
  pinMode(9,OUTPUT); digitalWrite(9,HIGH); // for HC-05
  delay(10);
  mySerial.begin(38400);
  while (!mySerial) {}
  delay(10);
  String readback = "";
  
  readback_myserial(readback);
  Serial.println(readback);
  if(readback!="OK"){
    Serial.println("Could not connect to bluetooth device in AT mode at 38400 baud. Bluetooth may not function correctly.");
  }
  
  mySerial.print("AT+UART=115200,1,0\r\n");
  delay(10);
  readback_myserial(readback);Serial.println(readback);
  if(readback!="OK"){
    Serial.println("Could not set bluetooth to baud 115200, stop_bits=1. parity=0.");
  }

  mySerial.print("AT+NAME=");mySerial.print("bt");mySerial.print(_DEVICE_ID);mySerial.print(_DEVICE_UID);mySerial.print("\r\n");
  delay(10);
  readback_myserial(readback);Serial.println(readback);
  if(readback!="OK"){
    Serial.println("Could not set bluetooth name.");
  }
  
  mySerial.print("AT+PSWD=");mySerial.print(_BT_PIN_CODE);mySerial.print("\r\n");
  delay(10);
  readback_myserial(readback);Serial.println(readback);
  if(readback!="OK"){
    Serial.println("Could not set bluetooth pin.");
  }
  
  mySerial.end();

  digitalWrite(9,LOW); // turn off AT mode
  delay(10);
  mySerial.begin(115200);

   // serial to display data
  
  // set bluetooth name to bt_DEVICE_ID_DEVICE_UID
  
  mySerial.println(_DEVICE_ID); // send name of device on first to match other controller codes
  
  mpuIntStatus = IMU.begin();

  // devStatus if everything worked properly
  if (mpuIntStatus < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(mpuIntStatus);
    while (1) {
      delay(1000);
      Serial.print("IMU is dead: ");
      Serial.println(mpuIntStatus);
    }
  }
}

void loop() {
  if (mySerial.available() > 0) {
    switch(mySerial.read()) { 
    case 'q': 
        mySerial.println(_DEVICE_ID);
        break; 
    default:
        break;
    }
  }
  
  IMU.readSensor();
  
  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();
  temp = IMU.getTemperature_C();
  
  deltat = fusion.deltatUpdate();
  //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony is suggested if there isn't the mag
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick
  
  to_be_sent[0] = fusion.q0;
  to_be_sent[1] = fusion.q1;
  to_be_sent[2] = fusion.q2;
  to_be_sent[3] = fusion.q3;

  float joyx = analogRead(joyx_pin);
  float joyy = analogRead(joyy_pin);
  float trgr = analogRead(trgr_pin);
  float joy_clk = digitalRead(joy_clk_pin);
  float sys = digitalRead(sys_pin);
  float menu = digitalRead(menu_pin);
  float grip = digitalRead(grip_pin);
  float util = digitalRead(util_pin);

  to_be_sent[4] = joyx;
  to_be_sent[5] = joyy;
  to_be_sent[6] = trgr;
  to_be_sent[7] = joy_clk;
  to_be_sent[8] = sys;
  to_be_sent[9] = menu;
  to_be_sent[10] = grip;
  to_be_sent[11] = util;
  
  // send the packet
  /*
  for(int i=0; i<12; i++){
    Serial.print(int(to_be_sent[i])); Serial.print(",");
  }
  Serial.print("\t\r\n");
  */
  
  mySerial.write((char*)to_be_sent, sizeof(float)*12);
  mySerial.write("\t\r\n");
}
