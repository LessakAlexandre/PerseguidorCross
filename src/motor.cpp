#include "motor.hpp"

motor::motor(int PWM_pin,int DIR_pin,int BASE_SPEED){ 
    PWM_PIN = PWM_pin;
    DIR_PIN = DIR_pin;
    baseSpeed = BASE_SPEED;
    pinMode(DIR_pin, OUTPUT);

}
void motor::setMotorSpeed(int speed){
// Se speed >= 0, direção para frente; caso contrário, ré.
    if (speed >= 0) {
        digitalWrite(this->DIR_PIN, LOW);
    } else {
        digitalWrite(this->DIR_PIN, HIGH);
        speed = -speed;
    }
    speed = constrain(speed, 0, 1023);
    ledcWrite(this->PWM_PIN, speed);
}
int motor::getPin(){;
    return PWM_PIN;
}