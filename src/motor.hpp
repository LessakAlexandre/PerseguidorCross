#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>

class MOTOR{
    private:
        int PWM_PIN;
        int DIR_PIN;
        int baseSpeed;

    public:
        //---------------------construtor-------------------------//
        MOTOR(int PWM_pin,int DIR_pin,int BASE_SPEED);
        //----------------------Funções---------------------------//
        //pinMode(FAULT_PIN, INPUT_PULLUP); -> INPUT_PULLUP valor HIGH por padrão
        void setMotorSpeed(int speed);
        int getPin();
};

#endif // MOTOR_HPP