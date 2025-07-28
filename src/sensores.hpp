#ifndef SENSORES_hpp
#define SENSORES_hpp
    #include <Arduino.h>

//---------Definições-------------//
    //Sensores    
        #define NUM_SENSORES          12
        const int VALOR_ESCALA =      1000;
        bool calibrado         =      false;
    //MUX
        #define MUX_S0                42
        #define MUX_S1                41
        #define MUX_S2                9
        #define MUX_S3                3
        #define MUX_SIG               2
        SemaphoreHandle_t xMutex;

//---------Funções---------------//
inline void setMuxChannel(int canal) {
    digitalWrite(MUX_S0, (canal & 0x01) ? HIGH : LOW);
    digitalWrite(MUX_S1, (canal & 0x02) ? HIGH : LOW);
    digitalWrite(MUX_S2, (canal & 0x04) ? HIGH : LOW);
    digitalWrite(MUX_S3, (canal & 0x08) ? HIGH : LOW);
}
int mapearValorSensorInvertido(int valor, int minVal, int maxVal) {
        if (valor <= minVal) return VALOR_ESCALA;
        if (valor >= maxVal) return 0;
        return VALOR_ESCALA - ((valor - minVal) * VALOR_ESCALA / (maxVal - minVal));
}

class sensor{
    private:
        float lastPosicao;
        long somaPonderada = 0;
        long somaValores = 0;
        int sensorMin[NUM_SENSORES];
        int sensorMax[NUM_SENSORES];
        int sensorValues[NUM_SENSORES];
        const int sensorPos[NUM_SENSORES] = {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100};
    public:

        sensor(float PosicaoInicial);
        void calibracao();
        volatile float leitura();
    };
class sensor_lateral{
    private:
        int SENSOR_PIN;
        int count;

    public:
        sensor_lateral(int sensor_pin);
        int count_lap();
    };   

#endif