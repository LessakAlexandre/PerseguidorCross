#include "sensores.hpp"
#include "defines.h"
#include "BT.h"
#include "BT.cpp"
//-----------Frontal----------//
sensor::sensor(float PosicaoInicial){
    lastPosicao = PosicaoInicial;
}
void sensor::calibracao(){
    for (int i = 0; i < NUM_SENSORES; i++) {
            sensorMin[i] = 65535;
            sensorMax[i] = 0;
        }
        unsigned long tempoCalibracao = millis();
        while (millis() - tempoCalibracao < 5000) {
            for (int i = 0; i < NUM_SENSORES; i++) {
            setMuxChannel(i);
            int leitura = analogRead(MUX_SIG);
            if (leitura < sensorMin[i]) sensorMin[i] = leitura;
            if (leitura > sensorMax[i]) sensorMax[i] = leitura;
            }
        }
}
volatile float sensor::leitura(){
    for (int i = 0; i < NUM_SENSORES; i++) {
            setMuxChannel(i);
            int leitura = analogRead(MUX_SIG);
            int valorSensor = mapearValorSensorInvertido(leitura, sensorMin[i], sensorMax[i]);
            sensorValues[i] = valorSensor;
            somaPonderada += (long)valorSensor * sensorPos[i];
            somaValores += valorSensor;
        }
        float pos = (somaValores > 0) ? ((float)somaPonderada / somaValores) : lastPosicao;
        lastPosicao = pos;
        return pos;
}
//----------Lateral-----------//
sensor_lateral::sensor_lateral(int sensor_pin){
    SENSOR_PIN = sensor_pin;
}
int sensor_lateral::count_lap(){
    if(!robotEnabled) return;
    if(millis() - last_lap_time < 200/*Deboucing time*/) return;
    last_lap_time = millis();
    end_lap_counter += 1;
    if(end_lap_counter >= endLap_goal){
        bluetooth_states_t state = STOP_STATE;
        Serial.println("Stoping!");
    robotEnabled = false;       
    }
}
int sensor_lateral::getPin(){
    return SENSOR_PIN;
}