#include "MPU9250.h"    // https://github.com/bolderflight/MPU9250  
#include "Streaming.h"    // needed for the Serial output https://github.com/geneReeves/ArduinoStreaming
#include "SensorFusion.h" // Note: currently requires making q0,q1,q2,q3 public
SF fusion;

const char* device_name = "headset";
float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float pitch, roll, yaw;
float deltat;
MPU9250 IMU(Wire, 0x68);
int status;

void setup() {
    
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  Serial.println(device_name);
}

void loop() {
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
  
  if (Serial.available() > 0) {
    switch(Serial.read()) { 
    case 'q': 
        Serial.println(device_name);
        break; 
    default:
        break;
    }
  }

  auto msg =  String(fusion.q0, 6) + ',' + String(fusion.q1, 6) + ',' + String(fusion.q2, 6) + ',' + String(fusion.q3, 6) + '\n';
  Serial.write(msg.c_str());

  //delay(200); //for readability

}
