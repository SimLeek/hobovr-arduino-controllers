#include "MPU9250.h"    // https://github.com/bolderflight/MPU9250  
#include "SensorFusion.h" // Note: currently requires making q0,q1,q2,q3 public
#include "I2Cdev.h"

SF fusion;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU9250 IMU(Wire, 0x68);

#define _DEVICE_ID "headset" // should be one of "headset", "right_controller", "left_controller"
#define _DEVICE_UID "12345" //A per-device UIDto prevent name conflicts between multiple headsets
#define _BT_PIN_CODE "000000" //000000 allows anything to connect. Other codes will add security.
// IMU status and control:
uint8_t mpuIntStatus;

// hobo_vr communication
float to_be_sent[13] = {1, 0, 0, 0}; // [w, x, y, z, trgr, padx, pady, pad_clk, trgr_clk, sys, menu, grip, util]
// ^^^ le paket

void setup() {

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  //increase baud rate for slow HM-10 or HM-08 bluetooth devices
  //If the baud rate is already high, this will just be a nonsense string
  Serial.begin(9600);
  while (!Serial) {}
  Serial.println("AT+BAUD4");
  Serial.end();

   // serial to display data
  Serial.begin(115200);
  while (!Serial) {}
  // set bluetooth name to bt_DEVICE_ID_DEVICE_UID
  Serial.print("AT+NAME");Serial.print("bt");Serial.print(_DEVICE_ID);Serial.println(_DEVICE_UID); 
  Serial.println(_DEVICE_ID); // send name of device on first to match other controller codes
  mpuIntStatus = IMU.begin();

  // devStatus if everything worked properly
  if (mpuIntStatus < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
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
  
  // send the packet
  Serial.write((char*)to_be_sent, sizeof(float)*4);
  Serial.write("\t\r\n");
}
