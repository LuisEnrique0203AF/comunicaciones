#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <time.h>
#include "PubSubClient.h" // <-- Librería MQTT

// =========================================================
// --- CONFIGURACIÓN (Tus datos) ---
// =========================================================

// --- WiFi ---
const char* ssid = "EnlaceFTTH_CASA_2.4G";
const char* password = "6481211001";

// --- MQTT (RabbitMQ) ---
const char* mqttServer = "192.168.1.241";
const int   mqttPort = 1883;
const char* mqttUser = "luisenrique";
const char* mqttPassword = "enrique02";

// --- HORA (NTP) ---
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -6 * 3600; // CST (Chihuahua, GMT-6)
const int   daylightOffset_sec = 0;

// =========================================================
// --- PINES Y OBJETOS ---
// =========================================================
#define SS_PIN    5
#define RST_PIN   4
#define LED_ACCESS_GRANTED_PIN  13
#define LED_ACCESS_DENIED_PIN   12
#define SERVO_PIN               2

MFRC522 rfid(SS_PIN, RST_PIN);
Servo plumaServo;
WiFiClient espClient;
PubSubClient client(espClient);

// =========================================================
// --- USUARIOS Y COLA (QUEUE) ---
// =========================================================
struct AuthorizedUser {
  byte uid[4];
  const char* name;
};

const int numTarjetas = 4;
AuthorizedUser tarjetasAutorizadas[numTarjetas] = {
  {{0x83, 0xE8, 0xED, 0x2C}, "Usuario 1 (Acm)"},
  {{0x93, 0xB8, 0xB7, 0x14}, "Usuario 2 (asm)"},
  {{0xC3, 0x28, 0x1D, 0x2A}, "Usuario 3 (tbsm)"},
  {{0xF3, 0xB9, 0x7C, 0x29}, "Usuario 4 (tbcm)"}
};

QueueHandle_t userIndexQueue;

// =========================================================
// --- PROTOTIPOS DE FUNCIONES ---
// =========================================================
void taskReadRFID(void *parameter);
void taskControlActuators(void *parameter);
void taskMqttManager(void *parameter); // <-- Nueva tarea para MQTT
void mqttReconnect();
void callback(char* topic, byte* message, unsigned int length);
void publishAccessEvent(int userIndex);

// =========================================================
// --- SETUP ---
// =========================================================
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
  Serial.println("\nWiFi conectado! IP: ");
  Serial.println(WiFi.localIP());

  // --- Sincronización de Hora ---
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // --- Configuración MQTT ---
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

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

  // --- Creación de Tareas FreeRTOS ---
  xTaskCreate(taskReadRFID, "Read RFID Task", 4096, NULL, 1, NULL);
  xTaskCreate(taskControlActuators, "Control Actuators Task", 4096, NULL, 1, NULL);
  xTaskCreate(taskMqttManager, "MQTT Manager Task", 4096, NULL, 1, NULL); // <-- Iniciamos la tarea MQTT
}

// =========================================================
// --- TAREAS FREERTOS ---
// =========================================================

// TAREA 1: Leer RFID (sin cambios)
void taskReadRFID(void *parameter) {
  for (;;) {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      int userIndex = -1;
      for (int i = 0; i < numTarjetas; i++) {
        bool match = true;
        for (int j = 0; j < 4; j++) {
          if (rfid.uid.uidByte[j] != tarjetasAutorizadas[i].uid[j]) {
            match = false;
            break;
          }
        }
        if (match) {
          userIndex = i;
          break;
        }
      }
      xQueueSend(userIndexQueue, &userIndex, portMAX_DELAY);
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// TAREA 2: Controlar actuadores y llamar a la publicación MQTT
void taskControlActuators(void *parameter) {
  int receivedUserIndex;
  for (;;) {
    if (xQueueReceive(userIndexQueue, &receivedUserIndex, portMAX_DELAY) == pdPASS) {
      publishAccessEvent(receivedUserIndex); // <-- Publica el evento a MQTT

      if (receivedUserIndex != -1) {
        // Lógica de servo y LEDs (sin cambios)
        digitalWrite(LED_ACCESS_GRANTED_PIN, HIGH);
        digitalWrite(LED_ACCESS_DENIED_PIN, LOW);
        plumaServo.write(90);
        vTaskDelay(pdMS_TO_TICKS(5000));
        plumaServo.write(0);
        digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
        digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
      } else {
        // Lógica de acceso denegado (sin cambios)
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

// TAREA 3: Gestionar la conexión MQTT
void taskMqttManager(void *parameter) {
  for (;;) {
    if (!client.connected()) {
      mqttReconnect();
    }
    client.loop(); // Esencial para mantener la conexión y recibir mensajes
    vTaskDelay(pdMS_TO_TICKS(50)); // Pausa para no saturar el CPU
  }
}

// =========================================================
// --- FUNCIONES MQTT ---
// =========================================================

// Función para publicar el evento de acceso
void publishAccessEvent(int userIndex) {
  if (!client.connected()) {
    Serial.println("No se puede publicar, cliente MQTT desconectado.");
    return;
  }

  // Obtiene la hora actual
  char timeBuffer[20];
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", &timeinfo);

  // Define el estado y el nombre de usuario
  const char* status = (userIndex != -1) ? "Permitido" : "Denegado";
  const char* userName = (userIndex != -1) ? tarjetasAutorizadas[userIndex].name : "Desconocido";

  // Crea el mensaje en formato JSON
  char jsonPayload[256];
  snprintf(jsonPayload, sizeof(jsonPayload), 
    "{\"status\":\"%s\", \"usuario\":\"%s\", \"timestamp\":\"%s\"}", 
    status, userName, timeBuffer);

  // Publica el mensaje
  const char* topic = "estacionamiento/accesos";
  client.publish(topic, jsonPayload);

  Serial.printf("MQTT Publicado -> Tópico: %s | Mensaje: %s\n", topic, jsonPayload);
}

// Función de reconexión (tomada de tu ejemplo)
void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    char clientId[50];
    sprintf(clientId, "ESP32_Estacionamiento-%ld", random(1000));
    
    if (client.connect(clientId, mqttUser, mqttPassword)) {
      Serial.println(" conectado!");
      // Suscribirse a un tópico para comandos remotos (ej. abrir pluma)
      client.subscribe("estacionamiento/comandos");
      Serial.println("Suscrito a 'estacionamiento/comandos'");
    } else {
      Serial.print(" falló, rc=");
      Serial.print(client.state());
      Serial.println(" -> Intentando de nuevo en 5 segundos");
      delay(5000); // Usamos delay() aquí porque estamos en un bucle de reconexión crítico
    }
  }
}

// Callback para mensajes entrantes (tomado de tu ejemplo)
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Mensaje recibido en [");
  Serial.print(topic);
  Serial.print("] ");
  String stMessage;
  for (int i = 0; i < length; i++) {
    stMessage += (char)message[i];
  }
  Serial.println(stMessage);
  
  // Aquí podrías añadir lógica para comandos remotos
  // ej: if (stMessage == "abrir") { plumaServo.write(90); }
}

void loop() {
  vTaskDelete(NULL);
}