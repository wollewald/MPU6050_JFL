#include <CoolCarLib.h>

/* ###############  CoolCarBasic ############### */

/************  Constructor ************/

CoolCarBasic::CoolCarBasic(uint8_t mP){
    maxPassengers = mP;
    speed = 0;
}

/**********  Public Functions **********/

uint8_t CoolCarBasic::getMaxPassengers(){    
    return maxPassengers;
}

uint16_t CoolCarBasic::getSpeed(){    
    return speed;
}

void CoolCarBasic::setSpeed(uint16_t sp){    
    speed = sp;
}

void CoolCarBasic::hoot(){
    Serial.println("beep! beep! beep!");
}

/* #################  CoolCar ################ */

/**********  Public Functions **********/

void CoolCar::init(){    
    currentSpeed = 0;
    airConLevel = CC_AC_OFF;
}

uint8_t CoolCar::getMaxPassengers(){    
    return maxPassengers;
}

uint16_t CoolCar::getMaxSpeed(){    
    return maxSpeed;
}

float CoolCar::getLengthInMeters(){
    return length;
}

uint16_t CoolCar::getCurrentSpeed(){
    return currentSpeed;
}

void CoolCar::hoot(){
    Serial.println("beep! beep! beep!");
}

bool CoolCar::accelerate(uint16_t accVal){
    bool noLimitViolation = true;
    uint16_t newSpeed = static_cast<uint16_t>(calculateNewSpeed(accVal));
    if(newSpeed > maxSpeed){
        currentSpeed = maxSpeed;
        noLimitViolation = false;
    }
    else{
        currentSpeed = newSpeed;
    }
    return noLimitViolation;
}

void CoolCar::brake(uint16_t brakeVal){
    int16_t newSpeed = calculateNewSpeed(brakeVal * (-1));
    if(newSpeed <= 0){
        currentSpeed = 0;
    }
    else{
        currentSpeed = (uint16_t)newSpeed;
    }
}

void CoolCar::setAirConLevel(cc_ac_level level){
    airConLevel = level;
}

cc_ac_level CoolCar::getAirConLevel(){
    return airConLevel;
}
        
/*********  Private Functions *********/

int16_t CoolCar::calculateNewSpeed(int16_t value){
    int16_t speed = currentSpeed + value;
    return speed;   
}