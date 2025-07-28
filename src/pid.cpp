#include "pid.hpp"

pid::pid(float KP, float KI , float KD,float POSICAO_DESEJADA ){
    kp = KP;
    ki = KI;
    kd = KD;
    posicaoDesejada = POSICAO_DESEJADA;
}
float pid::corretcion(volatile float posicao,float dt){
    float error = this->posicaoDesejada - posicao;
    float derivative = (error - lastError) / dt;
    integral += error * dt;
    lastError = error;
    return float(kp * error + ki * integral + kd * derivative);
}