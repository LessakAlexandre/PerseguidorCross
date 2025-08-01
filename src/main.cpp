#include <Arduino.h>
#include <AS5047P.h>
#include <Wire.h>
#include <LSM6DSRSensor.h>
#include <SPI.h>
#include "SPIFFS.h"             // Biblioteca para manipulação do sistema de arquivos SPIFFS
#include <Adafruit_NeoPixel.h>  // LED
#include <iostream>
#include <vector> 
#include <IRremote.h>

#include "BT.h" // Manipulação de Bluetooth  
#include "BT.cpp" 
#include "defines.h"
#include "gyro.hpp"
#include "encoder.hpp"
#include "pid.hpp"
#include "sensores.hpp"
#include "motor.hpp"
//---------------------------OBJETOS------------------------------------//
    //MOTORES
        motor Esquerdo(13,12,baseSpeed);
        motor Direito(48,47,baseSpeed);
    //SENSORES 
        sensor Frontais(550);
        sensor_lateral Lateral(1/*Pino sensor lateral*/);
    //ENCONDER
        encoder Enconder_11(1);
        encoder Encoder_39(8);
    //GIROSCOPIO
        giroscopio Gyro; //Ta certo assim?
    //PID
        pid Pid(0.475 , 0.0 , 0.075 , 550);
//------------------------------TASKS-----------------------------------//
void taskReadSensors(void *pvParameters){
  (void) pvParameters;

  float initAngle1 = Enconder_11.leitura();
  float initAngle2 = Encoder_39.leitura();

  xSemaphoreTake(xMutex, portMAX_DELAY);
  g_lastAngle1 = initAngle1;
  g_lastAngle2 = initAngle2;
  xSemaphoreGive(xMutex);

  // Exemplo de sensibilidade para ±4000 dps (ajuste conforme datasheet)
  const float SENSITIVITY_4000DPS = 0.140f;

  for (;;)
  {
    float angle1 = Enconder_11.leitura();
    float angle2 = Encoder_39.leitura();

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
  File readFile = SPIFFS.open("../data/coordenadas.txt", FILE_READ);
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
        posicao = Frontais.leitura();//aqui poderia não usar essa task e colocar direto?
    }
}
void ControlTask(void *pvParameters) {
    (void) pvParameters;
    unsigned long lastTime = millis();
    while (true) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0;
        if(dt>=0){
            float correcao=Pid.corretcion(posicao,dt);
            
            int leftSpeed  = baseSpeed - (int)correcao;
            int rightSpeed = -(baseSpeed + (int)correcao);
            
            leftSpeed  = constrain(leftSpeed, 10, 800);
            rightSpeed = constrain(rightSpeed, -800, -10);
            Direito.setMotorSpeed(rightSpeed);
            Esquerdo.setMotorSpeed(leftSpeed);

        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
void IRTask(void *pvParameters) {
  (void) pvParameters;
  while (true) {
    if (IrReceiver.decode()) {
      int comando = IrReceiver.decodedIRData.command;
      Serial.println(comando);
      // Se o código recebido for o definido para ligar/desligar:
      if (comando == IR_POWER_CODE) {
        robotEnabled = !robotEnabled;  // Alterna o estado
        if(!robotEnabled) {
          digitalWrite(Direito.getDir(), LOW);
          digitalWrite(Esquerdo.getDir(), LOW);
        } else {
          baseSpeed = baseSpeedIncrement;
          Serial.println(baseSpeed);
        }
        Serial.print("Robot ");
        Serial.println(robotEnabled ? "ligado" : "desligado");
        pixels.show();
        pixels2.show();
      }
      
      
      if (comando == IR_FAN) {
        fanControl = !fanControl;
        if(!fanControl) {
          ledcWrite(FAN_CHANNEL, 0);
        } else {
          for (int duty = 10; duty <= 600; duty+=10) {
              ledcWrite(FAN_CHANNEL, duty);
              vTaskDelay(pdMS_TO_TICKS(35));
          }
        }
      }
      if (comando == 88) {
        baseSpeedIncrement = baseSpeedIncrement + 20;
        digitalWrite(BUZZER, 1);
        delay(100);
        digitalWrite(BUZZER,0);
        
      }
      if (comando == 89) {
        baseSpeedIncrement = baseSpeedIncrement - 20;
        digitalWrite(BUZZER, 1);
        delay(100);
        digitalWrite(BUZZER,0);
      }
      int red = constrain(map(baseSpeedIncrement, 250, 400, 0, 255), 0, 255);
      pixels.setPixelColor(0, pixels.Color(red, 100, 0));
      pixels2.setPixelColor(0, pixels2.Color(red, 100, 0));
      pixels.show();
      pixels2.show();
    }
    IrReceiver.resume();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
void BtTask(void *pvParameters){
  (void) pvParameters;
  while (true){
    // Verifica se o dispositivo está conectado
    if(!dispositivo_Conectado && dispositivoAntigo_Conectado){
      delay(500);
      pServer->startAdvertising(); // Reinicia a propagação do BLE
      dispositivoAntigo_Conectado = dispositivo_Conectado;
    }

    // Se o dispositivo estiver conectado e não houver um dispositivo antigo conectado
    if(dispositivo_Conectado && !dispositivoAntigo_Conectado){
      dispositivoAntigo_Conectado = dispositivo_Conectado;
    }
    //Recebe os comandos do BT
    switch (BT_Handler.getState()){
    case STOP_STATE:
      digitalWrite(Direito.getDir(), LOW);
      digitalWrite(Esquerdo.getDir(), LOW);
      break;
    
    case CALIBRATION_STATE:
      if(calibrado){
        Serial.println("Tudo certo com os sensores");
      }
      else{
        Serial.println("Problema com a calibração");
      }
      break;
    case STAR_STATE:
        bool end_of_lap_state = digitalRead(Lateral.getPin());
        if(last_lap_state && !end_of_lap_state) Lateral.count_lap();  
        last_lap_state = end_of_lap_state;
        if(!robotEnabled && millis()-stop_time>500 /*BRAKE_TIME_MS*/){
          robotEnabled = true;
          bluetooth_states_t state = STOP_STATE;
        }
    default:
      break;
    }
  }
}




void setup(){
  Serial.begin(115200);
  // Inicializa o Bluetooth
    BLEDevice::init("THANOS_BLE"); // Nome do dispositivo Bluetooth
    // Cria o servidor BLE
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    //Cria o serviço BLE
    BLEService *pService = pServer->createService(SERVICE_UUID);
    //Cria a característica BLE para TX
    pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());
    //Cria a característica BLE para RX
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new MyCallbacks());
    // Inicia o serviço BLE
    pService->start();
    // Inicia a propagação do BLE
    pServer->getAdvertising()->start();
    Serial.println("Esperando por um dispositivo conectado...");
  
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
  gyroOffset = Gyro.calibrateSensor();
  
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

  ledcSetup(Direito.getPin(), PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(Esquerdo.getPin(), PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_PIN3, PWM_FREQ, PWM_RESOLUTION);
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
  while (!Enconder_11.SPI()){
      Serial.println(F("Erro ao conectar com o Encoder 1!"));
      delay(2000);
  }
  while (!Encoder_39.SPI()){
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
  xTaskCreatePinnedToCore(taskReadSensors,  "TaskReadSensors",  4096, NULL,4, NULL, 1);
  xTaskCreatePinnedToCore(ControlTask,  "ControlTask",  4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(SensorTask,  "SensorTask",  4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskComputeOdom,  "TaskComputeOdom",  4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskRecordCoordinates, "TaskRecordCoordinates", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(IRTask, "IRTask", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(BtTask, "BtTask", 2048,NULL, 0, NULL, 1);
}
void loop(){}