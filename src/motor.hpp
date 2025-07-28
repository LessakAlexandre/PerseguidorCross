#ifndef MOTOR_HPP
#define MOTOR_HPP
#include <Arduino.h>
//----------------Defiições----------------//
    #define PWM_PIN3              40      // pino para MCPWM (exemplo)
    #define PWM_FREQ              37500   // Frequência PWM para os motores
    #define PWM_RESOLUTION        10
    const float posicaoDesejada = 550;
    int baseSpeedIncrement =      300;     // Resolução: 10 bits (0-1023)
    int baseSpeed =               140;

class motor{
    private:
        int PWM_PIN;
        int DIR_PIN;
        int baseSpeed;

    public:
        //---------------------construtor-------------------------//
        motor(int PWM_pin,int DIR_pin,int BASE_SPEED);
        //----------------------Funções---------------------------//
        //pinMode(FAULT_PIN, INPUT_PULLUP); -> INPUT_PULLUP valor HIGH por padrão
        void setMotorSpeed(int speed);
        int getPin();
};
#endif // motor_HPP