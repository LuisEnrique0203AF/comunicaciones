#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <WiFi.h>      // <-- Librería para WiFi
#include <time.h>      // <-- Librería para la hora

// --- PINES (sin cambios) ---
#define SS_PIN    5
#define RST_PIN   4
#define LED_ACCESS_GRANTED_PIN  13
#define LED_ACCESS_DENIED_PIN   12
#define SERVO_PIN               2

// --- CONFIGURACIÓN WIFI Y HORA (NTP) ---
const char* ssid = "EnlaceFTTH_CASA_2.4G";     // 
const char* password = "6481211001"; // 

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -6 * 3600; // Offset para CST (Chihuahua, GMT-6)
const int   daylightOffset_sec = 0;    // Sin horario de verano

// --- OBJETOS ---
MFRC522 rfid(SS_PIN, RST_PIN);
Servo plumaServo;

// --- 1. ESTRUCTURA PARA DEFINIR USUARIOS ---
struct AuthorizedUser {
  byte uid[4];
  const char* name;
};

// --- LISTA DE USUARIOS AUTORIZADOS ---
const int numTarjetas = 4;
AuthorizedUser tarjetasAutorizadas[numTarjetas] = {
  {{0x83, 0xE8, 0xED, 0x2C}, "Usuario 1 (Acm)"},
  {{0x93, 0xB8, 0xB7, 0x14}, "Usuario 2 (asm)"},
  {{0xC3, 0x28, 0x1D, 0x2A}, "Usuario 3 (tbsm)"},
  {{0xF3, 0xB9, 0x7C, 0x29}, "Usuario 4 (tbcm)"}
};

// --- COLA (QUEUE) ---
// Ahora la cola enviará un número entero (el índice del usuario)
// -1 significará tarjeta inválida.
QueueHandle_t userIndexQueue;

// --- PROTOTIPOS DE FUNCIONES ---
void taskReadRFID(void *parameter);
void taskControlActuators(void *parameter);
void printLocalTime();

void setup() { 
  Serial.begin(115200);

  // --- Conexión WiFi ---
  Serial.print("Conectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");

  // --- Sincronización de Hora ---
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime(); // Imprime la hora actual una vez sincronizado

  // --- Inicialización de hardware ---
  SPI.begin();
  rfid.PCD_Init();
  plumaServo.attach(SERVO_PIN);
  plumaServo.write(0);
  
  pinMode(LED_ACCESS_GRANTED_PIN, OUTPUT);
  pinMode(LED_ACCESS_DENIED_PIN, OUTPUT);
  digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
  digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);

  // --- Creación de la Cola ---
  userIndexQueue = xQueueCreate(5, sizeof(int));

  Serial.println(F("\nSistema de estacionamiento listo."));

  // --- Creación de Tareas ---
  xTaskCreate(taskReadRFID, "Read RFID Task", 4096, NULL, 1, NULL);
  xTaskCreate(taskControlActuators, "Control Actuators Task", 4096, NULL, 1, NULL);
}

// ===============================================
// TAREA 1: Leer RFID y enviar el ÍNDICE del usuario a la cola
// ===============================================
void taskReadRFID(void *parameter) {
  for (;;) {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      int userIndex = -1; // Por defecto, -1 (inválido)

      // Busca el UID en la lista de usuarios autorizados
      for (int i = 0; i < numTarjetas; i++) {
        bool match = true;
        for (int j = 0; j < 4; j++) {
          if (rfid.uid.uidByte[j] != tarjetasAutorizadas[i].uid[j]) {
            match = false;
            break;
          }
        }
        if (match) {
          userIndex = i; // Encontramos el usuario, guardamos su índice
          break;
        }
      }

      // Envía el índice a la cola (-1 si no se encontró)
      xQueueSend(userIndexQueue, &userIndex, portMAX_DELAY);

      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ===============================================
// TAREA 2: Recibir de la cola y actuar
// ===============================================
void taskControlActuators(void *parameter) {
  int receivedUserIndex;

  for (;;) {
    if (xQueueReceive(userIndexQueue, &receivedUserIndex, portMAX_DELAY) == pdPASS) {
      
      if (receivedUserIndex != -1) { // Si el índice es válido
        // --- Obtener nombre y hora ---
        const char* userName = tarjetasAutorizadas[receivedUserIndex].name;
        char timeBuffer[20];
        struct tm timeinfo;
        getLocalTime(&timeinfo);
        strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", &timeinfo);

        Serial.printf("ACCESO PERMITIDO: %s a las %s\n", userName, timeBuffer);
        
        // --- Abrir pluma ---
        digitalWrite(LED_ACCESS_GRANTED_PIN, HIGH);
        digitalWrite(LED_ACCESS_DENIED_PIN, LOW);
        plumaServo.write(90);
        vTaskDelay(pdMS_TO_TICKS(5000));
        plumaServo.write(0);
        digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
        digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
      
      } else { // Si el índice es -1 (inválido)
        Serial.println(F("ACCESO DENEGADO - Tarjeta no registrada"));
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

// --- Función para imprimir la hora en la terminal ---
void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Fallo al obtener la hora");
    return;
  }
  char timeBuffer[20];
  strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  Serial.print("Hora actual: ");
  Serial.println(timeBuffer);
}

void loop() {
  vTaskDelete(NULL);
}