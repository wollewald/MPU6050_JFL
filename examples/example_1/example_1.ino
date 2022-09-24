#include <MPU6050_JFL.h>
#include <Wire.h>
#define I2C_ADDRESS 0x68

MPU6050_JFL myMPU = MPU6050_JFL(I2C_ADDRESS);

void setup() {
  Wire.begin();
  Serial.begin(9600);
  if(myMPU.init()){
    Serial.println("MPU6050 connected");
  }
  else{
    Serial.println("MPU6050 not connected");
    while(1); 
  }
  Serial.print("Who Am I: 0x");
  Serial.println(myMPU.whoAmI(),HEX);
  myMPU.setAccelRange(MPU6050_ACCEL_RANGE_2G);
  myMPU.setGyroRange(MPU6050_GYRO_RANGE_500DPS);
  Serial.print("Acceleration Range (0-3): ");
  Serial.println(myMPU.getAccelRange());
  Serial.println(myMPU.getOnlyTemperature());
}

void loop(){
  xyzFloat acc = myMPU.getAccelerationData();
  xyzFloat gyr = {0.0, 0.0, 0.0};
  myMPU.getGyroscopeData(&gyr);
  Serial.print("acc_x: ");
  Serial.print(acc.x); Serial.print('\t');
  Serial.print("acc_y: ");
  Serial.print(acc.y); Serial.print('\t');
  Serial.print("acc_z: ");
  Serial.println(acc.z);
  
  Serial.print("gyr_x: ");
  Serial.print(gyr.x); Serial.print('\t');
  Serial.print("gyr_y: ");
  Serial.print(gyr.y); Serial.print('\t');
  Serial.print("gyr_z: ");
  Serial.println(gyr.z);
  Serial. println();

  delay(2000); 
} 
