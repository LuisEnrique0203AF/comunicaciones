#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>

// --- PINES (sin cambios) ---
#define SS_PIN    5
#define RST_PIN   4
#define LED_ACCESS_GRANTED_PIN  13
#define LED_ACCESS_DENIED_PIN   12
#define SERVO_PIN               2

// --- OBJETOS ---
MFRC522 rfid(SS_PIN, RST_PIN);
Servo plumaServo;

// --- UIDs AUTORIZADOS (sin cambios) ---
const int numTarjetas = 4;
byte tarjetasAutorizadas[numTarjetas][4] = {
  {0x83, 0xE8, 0xED, 0x2C}, // Azul con marca
  {0x93, 0xB8, 0xB7, 0x14}, // Azul sin marca
  {0xC3, 0x28, 0x1D, 0x2A}, // Tarjeta blanca
  {0xF3, 0xB9, 0x7C, 0x29}  // Tarjeta blanca con marca
};

// --- 1. DEFINICIÓN DE LA COLA Y MENSAJES ---
QueueHandle_t cardStatusQueue;
enum CardStatus { NO_CARD, CARD_VALID, CARD_INVALID };

// --- PROTOTIPOS DE FUNCIONES ---
bool compareUID(byte *uidLeido, byte *uidAutorizado);
void taskReadRFID(void *parameter);
void taskControlActuators(void *parameter);

void setup() { 
  Serial.begin(115200);
  SPI.begin();
  rfid.PCD_Init();

  plumaServo.attach(SERVO_PIN);
  plumaServo.write(0); // Posición cerrada
  
  pinMode(LED_ACCESS_GRANTED_PIN, OUTPUT);
  pinMode(LED_ACCESS_DENIED_PIN, OUTPUT);
  digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
  digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);

  // --- 2. CREACIÓN DE LA COLA ---
  // Creamos una cola que puede almacenar hasta 5 mensajes de tipo 'CardStatus'
  cardStatusQueue = xQueueCreate(5, sizeof(CardStatus));

  Serial.println(F("Sistema de estacionamiento (RTOS + Queue) listo."));

  // Creación de las tareas (sin cambios)
  xTaskCreate(taskReadRFID, "Read RFID Task", 4096, NULL, 1, NULL);
  xTaskCreate(taskControlActuators, "Control Actuators Task", 4096, NULL, 1, NULL);
}

// ===============================================
// TAREA 1: Leer el RFID y ENVIAR a la cola
// ===============================================
void taskReadRFID(void *parameter) {
  for (;;) {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      bool tarjetaValida = false;
      for (int i = 0; i < numTarjetas; i++) {
        if (compareUID(rfid.uid.uidByte, tarjetasAutorizadas[i])) {
          tarjetaValida = true;
          break;
        }
      }

      // --- 3. ENVIAR MENSAJE A LA COLA ---
      CardStatus statusToSend = tarjetaValida ? CARD_VALID : CARD_INVALID;
      xQueueSend(cardStatusQueue, &statusToSend, portMAX_DELAY);

      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ===============================================
// TAREA 2: RECIBIR de la cola y actuar
// ===============================================
void taskControlActuators(void *parameter) {
  CardStatus receivedStatus;

  for (;;) {
    // --- 4. ESPERAR Y RECIBIR MENSAJE DE LA COLA ---
    // La tarea se bloqueará aquí hasta que reciba un mensaje. No consume CPU.
    if (xQueueReceive(cardStatusQueue, &receivedStatus, portMAX_DELAY) == pdPASS) {
      
      if (receivedStatus == CARD_VALID) {
        Serial.println(F("ACCESO PERMITIDO"));
        digitalWrite(LED_ACCESS_GRANTED_PIN, HIGH);
        digitalWrite(LED_ACCESS_DENIED_PIN, LOW);

        Serial.println(F("...Pluma abriendo..."));
        plumaServo.write(90);

        vTaskDelay(pdMS_TO_TICKS(5000));

        Serial.println(F("...Pluma cerrando..."));
        plumaServo.write(0);

        digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
        digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
      
      } else if (receivedStatus == CARD_INVALID) {
        Serial.println(F("ACCESO DENEGADO - Tarjeta incorrecta"));
        for (int i = 0; i < 3; i++) {
          digitalWrite(LED_ACCESS_DENIED_PIN, LOW);
          vTaskDelay(pdMS_TO_TICKS(150));
          digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
          vTaskDelay(pdMS_TO_TICKS(150));
        }
      }
    }
  }
}

// Función de comparación (sin cambios)
bool compareUID(byte *uidLeido, byte *uidAutorizado) {
  for (byte i = 0; i < 4; i++) {
    if (uidLeido[i] != uidAutorizado[i]) {
      return false;
    }
  }
  return true;
}

void loop() {
  vTaskDelete(NULL);
}