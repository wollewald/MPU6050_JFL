#include "Arduino.h"
#include "Wire.h"

#ifndef MPU6050_JFL_H_
#define MPU6050_JFL_H_

enum accel_range{
    MPU6050_ACCEL_RANGE_2G = 0,
    MPU6050_ACCEL_RANGE_4G,
    MPU6050_ACCEL_RANGE_8G,
    MPU6050_ACCEL_RANGE_16G
};

enum class GyroRange : uint8_t{
    MPU6050_250DPS  = 0x00,
    MPU6050_500DPS  = 0x08,
    MPU6050_1000DPS = 0x10,
    MPU6050_2000DPS = 0x18
};

struct xyzFloat {
    float x;
    float y;
    float z;
};

class MPU6050_JFL
{
public:
    MPU6050_JFL(const uint8_t addr = 0x68);
        
    bool init();
    bool reset();
    void sleep(bool sl);
    uint8_t whoAmI();
    void setAccelRange(accel_range range);
    uint8_t getAccelRange();
    void setGyroRange(GyroRange range);
    float getOnlyTemperature();
    xyzFloat getAccelerationData();
    void getGyroscopeData(xyzFloat *gyro);
    void update();
    xyzFloat getGyroscopeDataFromAllRawData();
        
private:
    uint8_t i2cAddress;
    float accelRangeFactor;
    float gyroRangeFactor;
    uint8_t allRawData[14];
    uint8_t writeRegister(uint8_t reg, uint8_t regValue);
    uint8_t readRegister(uint8_t reg);
    uint16_t read2Registers(uint8_t reg);
    void readMultipleRegisters(uint8_t reg, uint8_t count, uint8_t *buf); 
};

#endif
