#include <MPU6050_JFL.h>

bool MPU6050_JFL::init(){
    //Wire.setWireTimeout();
    accelRangeFactor = 1.0;
    gyroRangeFactor = 1.0; 
    bool connected = !reset();
    delay(100); // MPU6050 needs 100 ms to reset
    sleep(false); // disable sleep
    delay(100); // give the device some time to wake up!
    return connected;
}

bool MPU6050_JFL::reset(){
    return writeRegister(MPU6050_PWR_MGMT_1, MPU6050_DEVICE_RESET);
}

void MPU6050_JFL::sleep(bool sl){
    uint8_t regVal = readRegister(MPU6050_PWR_MGMT_1);
    if(sl){
        regVal |= MPU6050_DEVICE_SLEEP;
    }
    else{
        regVal &= ~MPU6050_DEVICE_SLEEP;
    }
    writeRegister(MPU6050_PWR_MGMT_1, regVal);
}   

uint8_t MPU6050_JFL::whoAmI() {
    return readRegister(MPU6050_WHO_AM_I);
}

void MPU6050_JFL::setAccelRange(accel_range range){
    uint8_t regVal = readRegister(MPU6050_ACCEL_CONFIG);
    switch(range){
        case MPU6050_ACCEL_RANGE_2G:  accelRangeFactor = 2.0;  break;
        case MPU6050_ACCEL_RANGE_4G:  accelRangeFactor = 4.0;  break;
        case MPU6050_ACCEL_RANGE_8G:  accelRangeFactor = 8.0;  break;
        case MPU6050_ACCEL_RANGE_16G: accelRangeFactor = 16.0;  break;
    }
    regVal &= ~MPU6050_ACCEL_RANGE_MASK;
    regVal |= range<<3;
    writeRegister(MPU6050_ACCEL_CONFIG, regVal);
}

void MPU6050_JFL::setGyroRange(GyroRange range){
    uint8_t regVal = readRegister(MPU6050_ACCEL_CONFIG);
    switch(range){
        case GyroRange::MPU6050_250DPS:  gyroRangeFactor = 250.0;  break;
        case GyroRange::MPU6050_500DPS:  gyroRangeFactor = 500.0;  break;
        case GyroRange::MPU6050_1000DPS: gyroRangeFactor = 1000.0;  break;
        case GyroRange::MPU6050_2000DPS: gyroRangeFactor = 2000.0;  break;  
    }
    regVal &= ~MPU6050_GYRO_RANGE_MASK;
    regVal |= static_cast<uint8_t>(range);
    writeRegister(MPU6050_GYRO_CONFIG, regVal);
}

uint8_t MPU6050_JFL::getAccelRange(){
    uint8_t regVal = readRegister(MPU6050_ACCEL_CONFIG);
    regVal = (regVal >> 3) & 0x03;
    return regVal;
}

float MPU6050_JFL::getOnlyTemperature(){
    uint16_t rawTemp = read2Registers(MPU6050_TEMP_OUT_H);
    float temp = (static_cast<int16_t>(rawTemp))/340.0 + 36.53; // see RegMap page 30;
    return temp;
}

xyzFloat MPU6050_JFL::getAccelerationData(){
    uint8_t rawData[6]; 
    readMultipleRegisters(MPU6050_ACCEL_XOUT_H, 6, rawData);
    accel.x = (static_cast<int16_t>((rawData[0] << 8) | rawData[1]))/32768.0 * accelRangeFactor;
    accel.y = (static_cast<int16_t>((rawData[2] << 8) | rawData[3]))/32768.0 * accelRangeFactor;
    accel.z = (static_cast<int16_t>((rawData[4] << 8) | rawData[5]))/32768.0 * accelRangeFactor;
    return accel;
}   

void MPU6050_JFL::getGyroscopeData(xyzFloat *gyro){
    uint8_t rawData[6]; 
    readMultipleRegisters(MPU6050_GYRO_XOUT_H, 6, rawData);
    gyro->x = (static_cast<int16_t>((rawData[0] << 8) | rawData[1]))/32768.0 * gyroRangeFactor;
    gyro->y = (static_cast<int16_t>((rawData[2] << 8) | rawData[3]))/32768.0 * gyroRangeFactor;
    gyro->z = (static_cast<int16_t>((rawData[4] << 8) | rawData[5]))/32768.0 * gyroRangeFactor;
}  

void MPU6050_JFL::update(){
    readMultipleRegisters(MPU6050_ACCEL_XOUT_H, 14, allRawData);
}

void MPU6050_JFL::getGyroscopeDataFromAllRawData(xyzFloat *gyro){
    readMultipleRegisters(MPU6050_GYRO_XOUT_H, 6, allRawData);
    gyro->x = ((static_cast<int16_t>(allRawData[8] << 8) | allRawData[9]))/32768.0 * gyroRangeFactor;
    gyro->y = ((static_cast<int16_t>(allRawData[10] << 8) | allRawData[11]))/32768.0 * gyroRangeFactor;
    gyro->z = ((static_cast<int16_t>(allRawData[12] << 8) | allRawData[13]))/32768.0 * gyroRangeFactor;
}   

uint8_t MPU6050_JFL::writeRegister(uint8_t reg, uint8_t regValue){
    Wire.beginTransmission(i2cAddress);
    Wire.write(reg);
    Wire.write(regValue);

    return Wire.endTransmission();
}

uint8_t MPU6050_JFL::readRegister(uint8_t reg){
    uint8_t data; 
    Wire.beginTransmission(i2cAddress);         
    Wire.write(reg);                    
    Wire.endTransmission(false);           
    Wire.requestFrom(i2cAddress, static_cast<uint8_t>(1));  
    data = Wire.read();                      
    return data;                             
}

uint16_t MPU6050_JFL::read2Registers(uint8_t reg){
    uint8_t MSB = 0, LSB = 0; // (Most / Least Significant Byte)
    uint16_t regValue = 0;
    Wire.beginTransmission(i2cAddress);         
    Wire.write(reg);                    
    Wire.endTransmission(false);           
    Wire.requestFrom(i2cAddress, static_cast<uint8_t>(2));  
    MSB = Wire.read(); 
    LSB = Wire.read();
    
    regValue = (MSB<<8) + LSB;
    return regValue;                             
}

void MPU6050_JFL::readMultipleRegisters(uint8_t reg, uint8_t count, uint8_t *buf){
    Wire.beginTransmission(i2cAddress);   
    Wire.write(reg);            
    Wire.endTransmission(false);       
    uint8_t i = 0;
    Wire.requestFrom(i2cAddress, count); 
    while (Wire.available()) {
        buf[i] = Wire.read();
        i++;
    }       
}
