#include <MPU6050_JFL.h>
#define I2C_ADDRESS 0x68

MPU6050_JFL myMPU = MPU6050_JFL(I2C_ADDRESS);

void setup() {
  Serial.begin(9600);
  if(myMPU.init()){
    Serial.println("MPU6050 connected");
  }
  else{
    Serial.println("MPU6050 not connected");
    while(1); 
  }
  myMPU.setGyroRange(gyro_range::MPU6050_GYRO_RANGE_500DPS);
}

void loop(){
  myMPU.update();
  xyzFloat gyr = myMPU.getGyroscopeDataFromAllRawData();
 
  Serial.print("gyr_x: ");
  Serial.print(gyr.x); Serial.print('\t');
  Serial.print("gyr_y: ");
  Serial.print(gyr.y); Serial.print('\t');
  Serial.print("gyr_z: ");
  Serial.println(gyr.z);

  delay(2000); 
} 
