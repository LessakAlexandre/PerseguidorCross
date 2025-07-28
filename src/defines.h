#include <Arduino.h>
#include <vector>
#include <Adafruit_NeoPixel.h>

//----------------DEFINIÇÕES----------------------------------//
    // Pose do robô
        volatile float g_x     =      0.0f; // posição em X (cm)
        volatile float g_y     =      0.0f; // posição em Y (cm)
        volatile float g_theta =      0.0f; // orientação (rad)
        float radius;
        std::vector<float>            radiusArray;
        float velo =                  450;
    //LED
        #define NEOPIXEL_PIN          43
        #define NEOPIXEL2_PIN         15
        #define NUMPIXELS             1
        Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
        Adafruit_NeoPixel pixels2(NUMPIXELS, NEOPIXEL2_PIN, NEO_GRB + NEO_KHZ800);
    //IR
        #define IR_RECV_PIN 39    // Pino conectado ao receptor IR
        #define IR_POWER_CODE 92  // Código IR para ligar/desligar (exemplo NEC)
        #define IR_FAN 12  // Código IR para ligar/desligar (exemplo NEC)
    //GLOBAIS
        unsigned long last_lap_time;
        uint16_t end_lap_counter = 0;
        bool last_lap_state = false;
        #define BRAKE_TIME_MS 500;
        unsigned long stop_time = 0;



        volatile float posicao =      0.0;   // Posição calculada dos sensores
        volatile bool robotEnabled =  false; // Se falso, os motores são desligados
        volatile bool fanControl =    false;
                    #define BUZZER 6
                    #define FAN_CHANNEL    2
    //MEDIDAS
        const float GEAR_RATIO      = 11.1f / 39.1f;   // Razão de engrenagem
        const float WHEEL_DIAMETER  = 22.5f;           // Diâmetro da roda (mm)
        const float WHEEL_CIRCUMF_CM=(WHEEL_DIAMETER * 3.14159265359f) / 10.0f; // Circunferência em cm
        const float WHEEL_BASE      = 0.135f;           // Distância entre as rodas (m)

