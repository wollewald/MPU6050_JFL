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
        static constexpr uint8_t MPU6050_GYRO_CONFIG       {0x1B};
        static constexpr uint8_t MPU6050_ACCEL_CONFIG      {0x1C};
        static constexpr uint8_t MPU6050_ACCEL_XOUT_H      {0x3B};
        static constexpr uint8_t MPU6050_ACCEL_XOUT_L      {0x3C};
        static constexpr uint8_t MPU6050_ACCEL_YOUT_H      {0x3D};
        static constexpr uint8_t MPU6050_ACCEL_YOUT_L      {0x3E};
        static constexpr uint8_t MPU6050_ACCEL_ZOUT_H      {0x3F};
        static constexpr uint8_t MPU6050_ACCEL_ZOUT_L      {0x40};
        static constexpr uint8_t MPU6050_TEMP_OUT_H        {0x41};
        static constexpr uint8_t MPU6050_TEMP_OUT_L        {0x42};
        static constexpr uint8_t MPU6050_GYRO_XOUT_H       {0x43};
        static constexpr uint8_t MPU6050_GYRO_XOUT_L       {0x44};
        static constexpr uint8_t MPU6050_GYRO_YOUT_H       {0x45};
        static constexpr uint8_t MPU6050_GYRO_YOUT_L       {0x46};
        static constexpr uint8_t MPU6050_GYRO_ZOUT_H       {0x47};
        static constexpr uint8_t MPU6050_GYRO_ZOUT_L       {0x48};
        static constexpr uint8_t MPU6050_PWR_MGMT_1        {0x6B}; // Device defaults to the SLEEP mode
        static constexpr uint8_t MPU6050_WHO_AM_I          {0x75}; // Should return 0x68
        static constexpr uint8_t MPU6050_ACCEL_RANGE_MASK  {0x18}; // = 0b00011000
        static constexpr uint8_t MPU6050_GYRO_RANGE_MASK   {0x18}; // = 0b00011000
        static constexpr uint8_t MPU6050_DEVICE_RESET      {0x80};
        static constexpr uint8_t MPU6050_DEVICE_SLEEP      {0x40};
        
        MPU6050_JFL(const uint8_t addr = 0x68)
        : i2cAddress{addr} {/* empty */}   
            
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
            
    protected:
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
