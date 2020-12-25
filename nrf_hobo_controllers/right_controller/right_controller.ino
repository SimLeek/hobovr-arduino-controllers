#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "MPU9250.h"    // https://github.com/bolderflight/MPU9250  
//#include "Streaming.h"    // needed for the Serial output https://github.com/geneReeves/ArduinoStreaming
#include "SensorFusion.h" // Note: currently requires making q0,q1,q2,q3 public
#include <printf.h>

/***********
 * GLOBALS *
 ***********/

//radio
RF24 radio(9,10);
const uint64_t right_controller_talking_pipe = 0xF0F0F0F0C3LL;
const uint64_t right_controller_listening_pipe = 0x3A3A3A3AC3LL;

/*enum controller_command{ 
  low_power_mode,
  power_up
} cmd_in;*/

//imu
SF fusion;
MPU9250 IMU(Wire, 0x68);
float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float deltat;

//buttons
#define PIN_JOY_UP A1
#define PIN_JOY_LEFT A0
#define PIN_JOY_TRIGGER A2
#define PIN_JOY_PRESS 4
#define PIN_BUTTON_GRIP 2
#define PIN_BUTTON_UTIL 3
#define PIN_BUTTON_SYS 5
#define PIN_BUTTON_MENU 6

//__attribute__((packed)) removes packing so this ends up the same as a list, and should be easier to read from Python.
struct __attribute__((packed)) ControllerState{
  float qw,qx,qy,qz;
  float stick_x, stick_y, trigger;
  bool stick_click, sys, menu, grip, util;
};

/*********
 * SETUP *
 *********/
void setup() {
  //debug
  Serial.begin(115200);
  printf_begin();
  while (!Serial) {}
  const char* device_name = "nrf24_vr_right_controller";  // used only for debugging
  Serial.println(device_name);
  Serial.println("debug setup complete");
  
  //radio
  radio.begin();
  radio.openWritingPipe(right_controller_talking_pipe);
  radio.openReadingPipe(1,right_controller_listening_pipe);
  //radio.startListening();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  Serial.println("radio setup complete");
  radio.printDetails(); // debug info

  //imu
  int imu_status;
  imu_status = IMU.begin();
  if (imu_status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(imu_status);
    while (1) {}
  }
  Serial.println("imu setup complete");

  //buttons
  pinMode(PIN_JOY_PRESS, INPUT);
  pinMode(PIN_BUTTON_GRIP, INPUT);
  pinMode(PIN_BUTTON_UTIL, INPUT);
  pinMode(PIN_BUTTON_SYS, INPUT);
  pinMode(PIN_BUTTON_MENU, INPUT);
  Serial.println("buttons setup complete. Now starting transmission loop.");
}

/********
 * LOOP *
 ********/
void loop() {
  //buttons
  ControllerState controller_state;
  controller_state.stick_click = 1-digitalRead(PIN_JOY_PRESS);
  controller_state.grip = 1-digitalRead(PIN_BUTTON_GRIP);
  controller_state.util = 1-digitalRead(PIN_BUTTON_UTIL);
  controller_state.sys = 1-digitalRead(PIN_BUTTON_SYS);
  controller_state.menu = 1-digitalRead(PIN_BUTTON_MENU);
  controller_state.stick_x = analogRead(PIN_JOY_UP);
  controller_state.stick_y = analogRead(PIN_JOY_LEFT);
  controller_state.trigger = analogRead(PIN_JOY_TRIGGER);

  //imu
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
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);
  controller_state.qw = fusion.q0;
  controller_state.qx = fusion.q1;
  controller_state.qy = fusion.q2;
  controller_state.qz = fusion.q3;

  //radio
  //radio.stopListening(); // needed before transmission
    //Serial.print("Controller state:");//Serial.println(controller_state);
    /*auto msg =  String(controller_state.qw, 6) + ',' + 
                String(controller_state.qx, 6) + ',' + 
                String(controller_state.qy, 6) + ',' + 
                String(controller_state.qz, 6) + ',' + 
                String(controller_state.stick_click) + ',' + 
                String(controller_state.grip) + ',' + 
                String(controller_state.util) + ',' + 
                String(controller_state.sys) + ',' + 
                String(controller_state.menu) + ',' + 
                String(controller_state.stick_x) + ',' + 
                String(controller_state.stick_y) + ',' + 
                String(controller_state.trigger);
    Serial.println(msg);*/
    radio.write( &controller_state, sizeof(controller_state) );
  //radio.startListening();
  //unsigned long started_waiting_at = millis();
  /*bool timeout = false;
  while ( ! radio.available() && ! timeout )
      if (millis() - started_waiting_at > 4 ) // 240fps update speed
        timeout = true;
  if ( !timeout ){
    radio.read( &cmd_in, sizeof(controller_command) ); // Example command input. Not used yet.
  }*/
}
