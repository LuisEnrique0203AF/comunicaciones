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

// --- VARIABLE GLOBAL PARA COMUNICAR TAREAS ---
// volatile asegura que la variable sea segura para usar entre tareas
enum CardStatus { NO_CARD, CARD_VALID, CARD_INVALID };
volatile CardStatus currentCardStatus = NO_CARD;

// --- PROTOTIPOS DE FUNCIONES ---
bool compareUID(byte *uidLeido, byte *uidAutorizado);
void taskReadRFID(void *parameter);
void taskControlActuators(void *parameter);

void setup() { 
  Serial.begin(9600); // Usar 115200 es más rápido y estándar
  SPI.begin();
  rfid.PCD_Init();

  plumaServo.attach(SERVO_PIN);
  plumaServo.write(0); // Posición cerrada (corregido)
  
  pinMode(LED_ACCESS_GRANTED_PIN, OUTPUT);
  pinMode(LED_ACCESS_DENIED_PIN, OUTPUT);
  digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
  digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);

  Serial.println(F("Sistema de estacionamiento con FreeRTOS listo."));

  // --- CREACIÓN DE LAS TAREAS DE FREERTOS ---
  xTaskCreate(
    taskReadRFID,         // Función de la tarea
    "Read RFID Task",     // Nombre de la tarea
    4096,                 // Tamaño de la pila (stack)
    NULL,                 // Parámetros de la tarea
    1,                    // Prioridad
    NULL                  // Handle de la tarea
  );

  xTaskCreate(
    taskControlActuators, // Función de la tarea
    "Control Actuators Task", // Nombre
    4096,                 // Stack
    NULL,                 // Parámetros
    1,                    // Prioridad
    NULL                  // Handle
  );
}

// ===============================================
// TAREA 1: Leer constantemente el sensor RFID
// ===============================================
void taskReadRFID(void *parameter) {
  for (;;) { // Bucle infinito de la tarea
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      bool tarjetaValida = false;
      for (int i = 0; i < numTarjetas; i++) {
        if (compareUID(rfid.uid.uidByte, tarjetasAutorizadas[i])) {
          tarjetaValida = true;
          break;
        }
      }

      if (tarjetaValida) {
        currentCardStatus = CARD_VALID; // Avisa a la otra tarea que la tarjeta es válida
      } else {
        currentCardStatus = CARD_INVALID; // Avisa que la tarjeta es inválida
      }

      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Pequeña pausa para ceder el control a otras tareas
  }
}

// ===============================================
// TAREA 2: Controlar el servo y los LEDs
// ===============================================
void taskControlActuators(void *parameter) {
  for (;;) { // Bucle infinito de la tarea
    if (currentCardStatus == CARD_VALID) {
      Serial.println(F("ACCESO PERMITIDO"));
      digitalWrite(LED_ACCESS_GRANTED_PIN, HIGH);
      digitalWrite(LED_ACCESS_DENIED_PIN, LOW);

      Serial.println(F("...Pluma abriendo..."));
      plumaServo.write(90); // Posición abierta

      vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 5 segundos SIN bloquear la otra tarea

      Serial.println(F("...Pluma cerrando..."));
      plumaServo.write(0); // Posición cerrada

      digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
      digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
      
      currentCardStatus = NO_CARD; // Restablece el estado
    
    } else if (currentCardStatus == CARD_INVALID) {
      Serial.println(F("ACCESO DENEGADO - Tarjeta incorrecta"));
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_ACCESS_DENIED_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(150));
        digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(150));
      }
      currentCardStatus = NO_CARD; // Restablece el estado
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Revisa el estado cada 100ms
  }
}

// ===============================================
// Función de comparación (sin cambios)
// ===============================================
bool compareUID(byte *uidLeido, byte *uidAutorizado) {
  for (byte i = 0; i < 4; i++) {
    if (uidLeido[i] != uidAutorizado[i]) {
      return false;
    }
  }
  return true;
}

// El loop principal ahora está vacío, porque el planificador de FreeRTOS se encarga de todo.
void loop() {
  vTaskDelete(NULL); // Opcional: Borra la tarea del loop de Arduino para liberar recursos.
}