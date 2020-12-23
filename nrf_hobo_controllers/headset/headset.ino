#include <printf.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "MPU9250.h"    // https://github.com/bolderflight/MPU9250  
//#include "Streaming.h"    // needed for the Serial output https://github.com/geneReeves/ArduinoStreaming
#include "SensorFusion.h" // Note: currently requires making q0,q1,q2,q3 public


/***********
 * GLOBALS *
 ***********/
 // Used to tell what data we'll be sending. Options are "left_controller", "right_controller", 
 // "headset", and "all". Since we're receiving the left and right controllers, as a headset, 
 // we will be sending "all" data.
const char* device_name = "all";  
const int fps_limit=240;
int time_prev;

//radio
RF24 radio(9,10);
const uint64_t right_controller_talking_pipe = 0xF0F0F0F0C3LL;
const uint64_t right_controller_listening_pipe = 0x3A3A3A3AC3LL;
const uint64_t left_controller_talking_pipe = 0xF0F0F0F0D2LL;
const uint64_t left_controller_listening_pipe = 0x3A3A3A3AD2LL;

enum controller_command{ 
  low_power_mode,
  power_up
} cmd_out;

//imu
SF fusion;
float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float pitch, roll, yaw;
float deltat;
MPU9250 IMU(Wire, 0x68);
int imu_status;


//buttons
// no buttons on the headset currently, but they could be used for digital IPD adjustment, position resetting, etc.

//__attribute__((packed)) removes packing so this ends up the same as a list, and should be easier to read from Python.
struct __attribute__((packed)) ControllerState {
  float qw,qx,qy,qz;
  float stick_x, stick_y, trigger;
  bool stick_click, sys, menu, grip, util;
};

ControllerState left_controller_state, right_controller_state;

void setup() {
  time_prev = micros();
  //debug
  Serial.begin(1000000);
  printf_begin();
  while (!Serial) {}
  Serial.print(device_name);Serial.write("\t\r\n");
  
  //radio
  radio.begin();
  
  Serial.println("Radio setup complete.");
  //Serial.println(radio.get_status());
  //radio.setPALevel(4); //3 is max
  /*radio.setAutoAck(false);*/
  //radio.setDataRate(RF24_1MBPS);
  //radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setPALevel(RF24_PA_MAX);
  radio.openReadingPipe(1,left_controller_talking_pipe);
  radio.openReadingPipe(2,right_controller_talking_pipe);
  radio.startListening();
  radio.printDetails();


  //imu
  imu_status = IMU.begin();
  if (imu_status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(imu_status);
    while (1) {}
  }
  Serial.println("IMU setup complete.");

  //buttons

}

/********
 * LOOP *
 ********/
void loop() {
  if (Serial.available() > 0) {
      switch(Serial.read()) { 
      case 'q': 
          Serial.print(device_name);Serial.write("\t\r\n");
          break; 
      default:
          break;
      }
    }

  //skip updating so Python side can catch up, and should then catch all controller updates as well as headset:
  if(micros()-time_prev<1e6/fps_limit){
    return; 
  }
  time_prev = micros();
    
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

  //radio
  uint8_t pipe_num;
  while ( radio.available(&pipe_num) )
    {
      //Serial.print("Radio available. Pipe num is ");Serial.println(pipe_num);
      // returns true if this is the last read
      if(pipe_num==1){
        radio.read( &left_controller_state, sizeof(ControllerState) );
        /*auto msg =  String(left_controller_state.qw, 6) + ',' + 
              String(left_controller_state.qx, 6) + ',' + 
              String(left_controller_state.qy, 6) + ',' + 
              String(left_controller_state.qz, 6) + ',' + 
              String(left_controller_state.stick_click) + ',' + 
              String(left_controller_state.grip) + ',' + 
              String(left_controller_state.util) + ',' + 
              String(left_controller_state.sys) + ',' + 
              String(left_controller_state.menu) + ',' + 
              String(left_controller_state.stick_x) + ',' + 
              String(left_controller_state.stick_y) + ',' + 
              String(left_controller_state.trigger);
        Serial.print("Left Controller State:");//Serial.println(msg);*/
        Serial.write("lc:");Serial.write((char*)&left_controller_state, sizeof(ControllerState));Serial.write("\t\r\n");
      }else if(pipe_num==2){
        radio.read( &right_controller_state, sizeof(ControllerState) );
        /*auto msg =  String(right_controller_state.qw, 6) + ',' + 
              String(right_controller_state.qx, 6) + ',' + 
              String(right_controller_state.qy, 6) + ',' + 
              String(right_controller_state.qz, 6) + ',' + 
              String(right_controller_state.stick_click) + ',' + 
              String(right_controller_state.grip) + ',' + 
              String(right_controller_state.util) + ',' + 
              String(right_controller_state.sys) + ',' + 
              String(right_controller_state.menu) + ',' + 
              String(right_controller_state.stick_x) + ',' + 
              String(right_controller_state.stick_y) + ',' + 
              String(right_controller_state.trigger);
        Serial.print("Right Controller State:");Serial.println(msg);*/
        Serial.write("rc:");Serial.write((char*)&right_controller_state, sizeof(ControllerState));Serial.write("\t\r\n");
      }
    }

  //headset data
  auto msg =  String(fusion.q0, 6) + ',' + 
        String(fusion.q1, 6) + ',' + 
        String(fusion.q2, 6) + ',' + 
        String(fusion.q3, 6);
  Serial.print("Headset State:");Serial.println(msg);
  Serial.write("hmd:");
  Serial.write((char*)&fusion.q0, sizeof(float));Serial.write((char*)&fusion.q1, sizeof(float));
  Serial.write((char*)&fusion.q2, sizeof(float));Serial.write((char*)&fusion.q3, sizeof(float));
  Serial.write("\t\r\n");
}
