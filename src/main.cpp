#include <Arduino.h>
#include <AS5047P.h>
#include <Wire.h>
#include <LSM6DSRSensor.h>
#include <SPI.h>
#include "SPIFFS.h"   // Biblioteca para manipulação do sistema de arquivos SPIFFS
#include <Adafruit_NeoPixel.h>
#include <iostream>
#include <vector>

//----------------DEFINIÇÕES-------------
    //MUX
        #define MUX_S0                42
        #define MUX_S1                41
        #define MUX_S2                9
        #define MUX_S3                3
        #define MUX_SIG               2
    SemaphoreHandle_t xMutex;
    //MOTORES
        #define PWM_PIN3              40      // pino para MCPWM (exemplo)
        #define PWM_FREQ              37500   // Frequência PWM para os motores
        #define PWM_RESOLUTION        10
        const float posicaoDesejada = 550;
        int baseSpeedIncrement =      300;      // Resolução: 10 bits (0-1023)
    //SENSORES
        #define NUM_SENSORES          12
        const int VALOR_ESCALA =      1000;
        bool calibrado =              false;
    //GIROSCOPIO
        volatile float g_angle1 = 0.0f;
        volatile float g_angle2 = 0.0f;

        volatile float g_lastAngle1 = 0.0f;
        volatile float g_lastAngle2 = 0.0f;

        volatile float g_totalDisp1 = 0.0f; // cm
        volatile float g_totalDisp2 = 0.0f; // cm

    // Giroscópio (Z) em graus/s (dps)
        volatile float g_gyroZ_dps = 0.0f;
        float gyroOffset = 0;

    // Pose do robô
        volatile float g_x     = 0.0f; // posição em X (cm)
        volatile float g_y     = 0.0f; // posição em Y (cm)
        volatile float g_theta = 0.0f; // orientação (rad)
        float radius;
        std::vector<float> radiusArray;
        float velo = 450;
    //LED
        #define NEOPIXEL_PIN          43
        #define NEOPIXEL2_PIN         15
        #define NUMPIXELS             1
        Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
        Adafruit_NeoPixel pixels2(NUMPIXELS, NEOPIXEL2_PIN, NEO_GRB + NEO_KHZ800);
    //I2C
        #define I2C_SDA               5
        #define I2C_SCL               4
        #define LSM6DSR_ADDRESS LSM6DSR_I2C_ADD_L
        TwoWire I2C_BUS = TwoWire(0);
        LSM6DSRSensor AccGyr(&I2C_BUS, LSM6DSR_ADDRESS);
    //GLOBAIS
        volatile float posicao =      0.0;   // Posição calculada dos sensores
        volatile bool robotEnabled =  false; // Se falso, os motores são desligados
        volatile bool fanControl =    false;
    //ROBO
        const float GEAR_RATIO      = 11.1f / 39.1f;   // Razão de engrenagem
        const float WHEEL_DIAMETER  = 22.5f;           // Diâmetro da roda (mm)
        const float WHEEL_CIRCUMF_CM= (WHEEL_DIAMETER * 3.14159265359f) / 10.0f; // Circunferência em cm
        const float WHEEL_BASE      = 0.135f;           // Distância entre as rodas (m)
//--------------FUNÇÕES------------------
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

//--------------CLASSES------------------
    class MOTOR{
    private:
        const int PWM_PIN;
        const int DIR_PIN;
        int baseSpeed = 140;

    public:
        //---------------------construtor-------------------------//
        MOTOR(int PWM_pin,int DIR_pin)
        :PWM_PIN(PWM_pin), DIR_PIN(DIR_pin){ 
        pinMode(DIR_PIN, OUTPUT);
        }
        //----------------------Funções---------------------------//
        //pinMode(FAULT_PIN, INPUT_PULLUP); -> INPUT_PULLUP valor HIGH por padrão
        void setMotorSpeed(int speed){
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
    };
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

        sensor(float PosicaoInicial){
        lastPosicao = PosicaoInicial;
        }
        void calibracao(){
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
        volatile float leitura(){
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
    };
    class sensor_lateral{
    private:
        const int SENSOR_PIN;

    public:
        sensor_lateral(int sensor_pin):SENSOR_PIN(sensor_pin){}
        int leitura(){
        return digitalRead(SENSOR_PIN);
        }
    };
    class encoder{
    private:
        const int PIN;
        const int SPI_BUS_SPEED = 10000000;
        //AS5047P ENCONDER(PIN, SPI_BUS_SPEED);
    public:
        encoder(int PIN_ENCODER):PIN(PIN_ENCODER){  
        }
    
        float leitura(){
            AS5047P ENCODER(PIN, SPI_BUS_SPEED);
            return ENCODER.readAngleDegree();
        }
    };
    class giroscopio{
    private:
        int32_t gyroRaw[3];
        const int numSamples = 1000;  
        float sumGyro = 0;
        float Offset;
    public:
        float calibrateSensor() {
            int32_t gyroData[3];
            Serial.println("Calibrando o sensor... Mantenha-o imóvel.");
            delay(2000);
            for (int i = 0; i < numSamples; i++) {
                AccGyr.Get_G_Axes(gyroData);
                sumGyro += gyroData[2];
                delay(5);
            }
            Offset = sumGyro / numSamples;
            
            Serial.print("Giroscópio Offsets: ");
            Serial.println(gyroOffset);
            return Offset;
        }
        float leitura(){
            AccGyr.Get_G_Axes(gyroRaw);
            return gyroRaw[2];
        }
    };
    class pid{
    private:
      float kp;
      float ki;
      float kd;
      float integral = 0.0;
      float lastError = 0.0;
      const float posicaoDesejada;
      float lastPosicao = posicaoDesejada;
    
    public:
      pid(float KP, float KI , float KD,float POSICAO_DESEJADA ):posicaoDesejada(POSICAO_DESEJADA){
        kp = KP;
        ki = KI;
        kd = KD;
      }
      float corretcion(volatile float posicao,float dt){
        float error = this->posicaoDesejada - posicao;
        float derivative = (error - lastError) / dt;
        integral += error * dt;
        lastError = error;
        return float(kp * error + ki * integral + kd * derivative);
    }

};
//---------------------------OBJETOS------------------------------------//
    //MOTORES
        MOTOR esquerdo(13,12);
        MOTOR direito(48,47);
    //SENSORES
        sensor Frontais(550);
    //sensor Lateral(/*PINO*/);
    //ENCONDER
        encoder ENCODER_11(1);
        encoder ENCODER_39(8);
    //GIROSCOPIO
        giroscopio GYRO; //Ta certo assim?
        pid PID(0.475 , 0.0 , 0.075 , 550);
//------------------------------TASKS-----------------------------------//
void taskReadSensors(void *pvParameters){
  (void) pvParameters;

  float initAngle1 = ENCODER_11.leitura();
  float initAngle2 = ENCODER_39.leitura();

  xSemaphoreTake(xMutex, portMAX_DELAY);
  g_lastAngle1 = initAngle1;
  g_lastAngle2 = initAngle2;
  xSemaphoreGive(xMutex);

  // Exemplo de sensibilidade para ±4000 dps (ajuste conforme datasheet)
  const float SENSITIVITY_4000DPS = 0.140f;

  for (;;)
  {
    float angle1 = ENCODER_11.leitura();
    float angle2 = ENCODER_39.leitura();

    // Ler giroscópio bruto
    int32_t gyroRaw[3];
    AccGyr.Get_G_Axes(gyroRaw);

    // Converte para dps (exemplo LSB -> dps)
    float gyroZ_dps = gyroRaw[2];

    xSemaphoreTake(xMutex, portMAX_DELAY);
    g_angle1    = angle1;
    g_angle2    = angle2;
    g_gyroZ_dps = gyroZ_dps;
    xSemaphoreGive(xMutex);

    // Espera 5 ms (aprox.) até a próxima leitura
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
void taskComputeOdom(void *pvParameters)
{
  (void) pvParameters;
  static uint32_t lastTime = micros();
  const float straight = 30;
  const float adjust_x = 100;
  const float adjust_y = -0;
  const float gain = 0.15;
  const float max_vel = 400;
  const float min_vel = 150;
  float facter = straight / adjust_x;
  float lastDisp = 0;
  float lastTheta = 0;

  for (;;)
  {
    uint32_t currentTime = micros();
    float dt = (currentTime - lastTime) / 1e6; // Converte micros para segundos
    lastTime = currentTime;

    // Captura as leituras compartilhadas
    xSemaphoreTake(xMutex, portMAX_DELAY);
    float angle1       = g_angle1;
    float angle2       = g_angle2;
    float lastAngle1   = g_lastAngle1;
    float lastAngle2   = g_lastAngle2;
    float gyroZ_dps    = g_gyroZ_dps;
    xSemaphoreGive(xMutex);

    // Calcula as diferenças dos ângulos dos encoders
    float deltaAngle1 = angle1 - lastAngle1;
    float deltaAngle2 = angle2 - lastAngle2;

    // Corrige wrap-around
    if (deltaAngle1 > 180.0f)  deltaAngle1 -= 360.0f;
    if (deltaAngle1 < -180.0f) deltaAngle1 += 360.0f;
    if (deltaAngle2 > 180.0f)  deltaAngle2 -= 360.0f;
    if (deltaAngle2 < -180.0f) deltaAngle2 += 360.0f;

    // Converte a variação dos ângulos dos encoders em rotação das rodas (em rotações)
    float wheelRotation1 = (deltaAngle1 * GEAR_RATIO) / 360.0f;
    float wheelRotation2 = (deltaAngle2 * GEAR_RATIO) / 360.0f;

    // Calcula o deslocamento linear (em cm) de cada roda
    float displacement1_cm = -(wheelRotation1 * WHEEL_CIRCUMF_CM);
    float displacement2_cm = wheelRotation2 * WHEEL_CIRCUMF_CM;
    g_totalDisp1 += displacement1_cm;
    g_totalDisp2 += displacement2_cm;

    float totalDisp = (g_totalDisp1 + g_totalDisp2) * 0.5f;

    // Estima a variação de ângulo a partir dos encoders (modelo diferencial)
    // Converte WHEEL_BASE para centímetros: WHEEL_BASE * 100
    float deltaThetaEncoder = (displacement2_cm - displacement1_cm) / (WHEEL_BASE * 100.0f);

    // Estima a velocidade angular dos encoders (rad/s)
    float encoderOmega = -deltaThetaEncoder / dt;

    // Converte a taxa angular do giroscópio de dps para rad/s
    float gyroOmega = (gyroZ_dps - gyroOffset) * (3.14159265359f / 180.0f) / 997.856;

    // Aplica o filtro complementar nas velocidades angulares

    const float alpha = 1; // pondera maiormente o giroscópio
    float filteredOmega = alpha * gyroOmega + (1.0f - alpha) * (encoderOmega);

    // Integra a velocidade angular filtrada para atualizar o ângulo
    xSemaphoreTake(xMutex, portMAX_DELAY);
    g_theta += filteredOmega * dt; // integrando velocidade angular para encontrar theta
    float thetaRounded = roundf(g_theta * 100.0f) / 100.0f;
    // Atualiza os ângulos anteriores dos encoders para a próxima iteração
    g_lastAngle1 = angle1;
    g_lastAngle2 = angle2;
    xSemaphoreGive(xMutex);

    // Atualiza também a posição (odometria linear) usando a média dos deslocamentos
    float v_cm_s = (displacement1_cm + displacement2_cm) * 0.5f;
    xSemaphoreTake(xMutex, portMAX_DELAY);
    g_x += v_cm_s * cosf(thetaRounded);
    g_y += v_cm_s * sinf(thetaRounded);
    if (abs(totalDisp) - lastDisp >= 5) {
      radius = abs(((abs(totalDisp) - lastDisp) / (g_theta - lastTheta)));
      radiusArray.push_back(radius);

      velo = (1 / (1 + exp(-(gain / facter) * radius + (adjust_x / 2) * gain))) * (max_vel - min_vel) + min_vel + adjust_y ;
      //velo = 1e-3 * radius * radius * ((max_vel - min_vel) / 100) + min_vel;
      lastDisp = abs(totalDisp);
      lastTheta = g_theta;

    }
    if (totalDisp > 85 * 6) { // duas voltas {
      robotEnabled = false;
      vTaskDelete(NULL);
    }
    xSemaphoreGive(xMutex);

    //Serial.print("Velocidade Angular Filtrada (rad/s): ");
    //Serial.print(g_x, 4);
    //Serial.print(" | ");
    //Serial.print(g_totalDisp1, 4);
    //Serial.print(" | ");
    //Serial.println(thetaRounded * 57.2958, 4);

    vTaskDelay(pdMS_TO_TICKS(3));
  }
}
void taskRecordCoordinates(void *pvParameters)
{
  (void) pvParameters;
  // Aguarda 2 segundos para que os sensores estejam estabilizados
  vTaskDelay(pdMS_TO_TICKS(2000));
  File readFile = SPIFFS.open("/data.txt", FILE_READ);
  if (!readFile) {
    Serial.println("Erro ao abrir data.txt para leitura!");
    vTaskDelete(NULL);
  }
  Serial.println("Conteúdo de data.txt:");
  while (readFile.available()) {
    vTaskDelay(pdMS_TO_TICKS(1));
    Serial.write(readFile.read());
  }
  readFile.close();

  // Abre ou cria o arquivo data.txt para escrita
  File file = SPIFFS.open("/data.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Erro ao abrir data.txt para escrita!");
    vTaskDelete(NULL);
  }
  pixels.setPixelColor(0, pixels.Color(255, 100, 0));
  pixels2.setPixelColor(0, pixels2.Color(255, 100, 0));
  pixels.show();
  pixels2.show();
  robotEnabled = true;
  uint32_t startTime = millis();
  // Registra as coordenadas a cada 0,5 s por 10 segundos (20 amostras)
  for (;;) {
    if (!robotEnabled) {
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels2.setPixelColor(0, pixels2.Color(0, 0, 0));
      pixels.show();
      pixels2.show();
      file.close();
      vTaskDelete(NULL);
    }
    float x, y, v;
    xSemaphoreTake(xMutex, portMAX_DELAY);
    x = g_x;
    y = g_y;
    v = velo;
    xSemaphoreGive(xMutex);

    float t = (millis() - startTime) / 1000.0f; // tempo em segundos
    // Formata a linha no formato "x,y,t"
    String line = String(x, 2) + "," + String(y, 2) + "," + String(t, 2) + "," + String(v, 2) + "\n";
    file.print(line);
    vTaskDelay(pdMS_TO_TICKS(15)); // aguarda 0,5 s
  }
}


void SensorTask(void*pvParameters){
    (void)pvParameters;
    while (true){
        posicao = Frontais.leitura()//aqui poderia não usar essa task e colocar direto?
    }
}
void ControlTask(void *pvParameters) {
    (void) pvParameters;
    while (true) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0;
        if(dt>=0){
            float correcao=PID.corretcion(posicao,dt)
            
            int leftSpeed  = baseSpeed - (int)correcao;
            int rightSpeed = -(baseSpeed + (int)correcao);
            
            leftSpeed  = constrain(leftSpeed, 10, 800);
            rightSpeed = constrain(rightSpeed, -800, -10);
            direito.setMotorSpeed(rightSpeed);
            esquerdo.setMotorSpeed(leftSpeed);

        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup(){
    Serial.begin(115200);
    delay(1000);
    Serial.println("Inicializando...");
    I2C_BUS.setClock(400000);
    I2C_BUS.begin(I2C_SDA, I2C_SCL);
    
    AccGyr.begin();
    AccGyr.Set_G_ODR_With_Mode(6667.0f, LSM6DSR_GYRO_LOW_POWER_NORMAL_MODE);
    AccGyr.Set_G_FS(2000); // Se suportado, configure o giroscópio para ±4000 dps
    AccGyr.Enable_X();
    AccGyr.Enable_G();
    //
    float gyroOffset = GYRO.calibrateSensor();
    
    if (!SPIFFS.begin(true)) {
        Serial.println("Erro ao montar SPIFFS");
        return;
    }
    SPI.begin(18, 17, 16);

    pixels.begin();
    pixels.clear();
    pixels.show();
    pixels2.begin();
    pixels2.clear();
    pixels2.show();
    pixels.setBrightness(50);
    pixels2.setBrightness(50);
    
    // Configuração dos pinos para o multiplexador
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_S3, OUTPUT);

    // Configuração do pino analógico do multiplexador
    pinMode(MUX_SIG, INPUT);

    analogReadResolution(16);

    // Configuração do receptor IR

    ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PWM_PIN2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PWM_PIN3, PWM_FREQ, PWM_RESOLUTION);
    pixels.setPixelColor(0, pixels.Color(51, 51, 204));
    pixels2.setPixelColor(0, pixels2.Color(51, 51, 204));
    pixels.show();
    pixels2.show();
    
    //Calibração dos sensores(5 segundos)
    Frontais.calibracao();
    calibrado = true;
    Serial.println("Calibração dos sensores concluída.");

    pixels.setPixelColor(0, pixels.Color(0, 30, 255));
    pixels2.setPixelColor(0, pixels2.Color(0, 30, 255));
    pixels.show();
    pixels2.show();

    vTaskDelay(pdMS_TO_TICKS(2500));
    while (!ENCODER_11.initSPI()) {
        Serial.println(F("Erro ao conectar com o Encoder 1!"));
        delay(2000);
    }
    while (!ENCODER_39.initSPI()) {
        Serial.println(F("Erro ao conectar com o Encoder 1!"));
        delay(2000);
    }
    // Inicializa I2C e IMU
    // Cria Mutex
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        Serial.println("Erro ao criar Mutex!");
    }
    //Cria as Task
    xTaskCreatePinnedToCore(taskReadSensors,  "TaskReadSensors",  4096, NULL,3, NULL, 1);
    xTaskCreatePinnedToCore(ControlTask,  "ControlTask",  4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(SensorTask,  "SensorTask",  4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(taskComputeOdom,  "TaskComputeOdom",  4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(taskRecordCoordinates, "TaskRecordCoordinates", 4096, NULL, 1, NULL, 1);
}
void loop(){}