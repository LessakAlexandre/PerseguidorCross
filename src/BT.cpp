#include "BT.h"
bt_handler BtHandler;

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool dispositivo_Conectado = false;
bool dispositivoAntigo_Conectado = false;
uint8_t txValue = 0;
extern volatile float kp, ki, kd, porcentagem, velocidade;
extern volatile uint16_t endLap_goal;
extern volatile bool modoTempo;
extern volatile unsigned long lapTime;
// =================== FUNÇÕES AUXILIARES ===================
void MyServerCallbacks::onConnect(BLEServer* pServer) 
{
  dispositivo_Conectado = true;
  Serial.println("Dispositivo conectado.");
}

void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
  dispositivo_Conectado = false;
  Serial.println("Dispositivo desconectado.");
}

void MyCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if(rxValue.length() > 0)
    {
        for(uint16_t i = 0; i < rxValue.length(); i++)
        {
            char incomingChar = rxValue[i];
            if(incomingChar == Start_Byte)
            {
                BtHandler.rxBuffer = ""; // Limpa o buffer ao receber o byte inicial
                continue;
            }
            if(incomingChar == End_Byte)
            {
                BtHandler.messageHandler(); // Chama o manipulador de mensagens ao receber o byte final
                continue;
            }
            BtHandler.rxBuffer += incomingChar; // Adiciona o caractere ao buffer
        }
    }
}

void bt_handler::messageHandler(){
    this->println("Mensagem recebida: ");
    this->println(BtHandler.rxBuffer);

    //Trata os comando recebidos pelo BT
    switch (this->state)
    {
    case(STOP_STATE):
        if(this->rxBuffer == "start")
        {
            this->println("Iniciando...");
            this->state = STAR_STATE;
            break;
        }
        
        if (this->rxBuffer == "set_pid")
        {
            this->println("Valor do KP: ");
            this->state = SET_KP_STATE;
            break;
        }
        
        if (this->rxBuffer == "calibrate")
        {
            this->println("Calibrando...");
            this->state = CALIBRATION_STATE;
            break;
        }

        if (this->rxBuffer == "set_goal")
        {
            this->println("Configurando meta...");
            this->state = SET_GOAL_STATE;
            break;
        }

        if(this->rxBuffer == "toggle_mode")
        {
            if(modoTempo)
            {
                this->println("Alternando para modo de contagem de marcas");
                modoTempo = false;
            }
            else
            {
                this->println("Alternando para modo de tempo");
                modoTempo = true;
            }
        }
        break;

    case(CALIBRATION_STATE):
        break;

    case(STAR_STATE):
        if(this->rxBuffer == "stop")
        {
            this->println("Parando...");
            this->state = STOP_STATE;
        }
        break;

    case(SET_KP_STATE):
        kp = this->rxBuffer.toFloat();
        this->println("Valor do KI: ");
        this->state = SET_KI_STATE;
        break;

    case(SET_KI_STATE):
        ki = this->rxBuffer.toFloat();
        this->println("Valor do KD: ");
        this->state = SET_KD_STATE;
        break;

    case(SET_KD_STATE):
        kd = this->rxBuffer.toFloat();
        this->println("Valor do porcentagem: ");
        this->state = SET_GOAL_STATE;
        break;
    
    case(SET_GOAL_STATE):
        if(modoTempo)
        {
            lapTime = this->rxBuffer.toInt();
            this->print("Tempo de volta: ");
            this->println(String(lapTime));
            this->state = STOP_STATE;
        }
        else
        {
            endLap_goal = this->rxBuffer.toInt();
            this->print("Meta: ");
            this->println(String(endLap_goal));
            this->state = STOP_STATE;
        }
        break;

    default:
        this->println("Estado desconhecido.");
        break;
    }
}

void bt_handler::clearBuffer(){
    this->rxBuffer = ""; // Limpa o buffer
}

bluetooth_states_t bt_handler::getState(){
    return this->state;
}

int bt_handler::read(){
    return 0;
}

void bt_handler::println(String message){
    this->print(message);
    this->print("\n");
}

void bt_handler::print(String message){
    pTxCharacteristic->setValue(message.c_str());
    pTxCharacteristic->notify(); // Notifica o cliente BLE sobre a nova mensagem
}