#include "sensores.hpp"
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
    if(digitalRead(SENSOR_PIN)==HIGH){
        count++;
    }
    return count;
}
