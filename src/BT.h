#ifndef BT_h
#define BT_H
#include <Arduino.h>
#include <BluetoothSerial.h>

//servicos BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

extern BLEServer *pServer;
extern BLECharacteristic *pTxCharacteristic;
extern bool dispositivo_Conectado;
extern bool dispositivoAntigo_Conectado;
extern uint8_t txValue;

//Site para gerar UID: https://www.uuidgenerator.net/

#define SERVICE_UUID           "be6c1ef9-efcd-4a47-93ec-6736214655c7" //servico UART
#define CHARACTERISTIC_UUID_RX "9d67cd8d-46a2-482c-b018-5ce71625e72a" //caracteristica UART RX
#define CHARACTERISTIC_UUID_TX "aa82e8ff-b233-4420-b14f-71aba80f100a" //caracteristica UART TX

const char Start_Byte = '#';
const char End_Byte = '*';

typedef enum BT_STATES{
    STOP_STATE,
    CALIBRATION_STATE,
    STAR_STATE,
    SET_KP_STATE,
    SET_KI_STATE,
    SET_KD_STATE,
    SET_GOAL_STATE,
} bluetooth_states_t;

class MyServerCallbacks: public BLEServerCallbacks{
    public:
        void onConnect(BLEServer* pServer);
        void onDisconnect(BLEServer* pServer);
};

class MyCallbacks: public BLECharacteristicCallbacks{
    public:
        void onWrite(BLECharacteristic *pCharacteristic);
};

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth nao ativado, verifique o menuconfig para habilitar o Bluetooth
#endif

class bt_handler{
    public:
        bluetooth_states_t state;    
        String rxBuffer;
        uint8_t rxBufferPointer;
        
        
        void messageHandler();
        void clearBuffer();
        void println(String message);
        void print(String message);
        virtual int available();
        virtual int read(); // Read a byte from the buffer
        
        bluetooth_states_t getState();

        
};

extern bt_handler BT_Handler; // precisa mesmo?



#endif