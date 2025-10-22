#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <time.h>
#include "PubSubClient.h"

// =========================================================
// --- CONFIGURACIÓN (Tus datos actualizados) ---
// =========================================================
const char* ssid = "EnlaceFTTH_CASA_2.4G";
const char* password = "6481211001";
const char* mqttServer = "192.168.1.241";
const int   mqttPort = 1883;
const char* mqttUser = "luisenrique";
const char* mqttPassword = "enrique02";
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -6 * 3600;
const int   daylightOffset_sec = 0;

// =========================================================
// --- PINES Y OBJETOS ---
// =========================================================
#define SS_PIN    5
#define RST_PIN   4
#define LED_ACCESS_GRANTED_PIN  13
#define LED_ACCESS_DENIED_PIN   12
#define SERVO_PIN               2

// --- PINES PARA EL SENSOR ULTRASÓNICO ---
#define TRIG_PIN 26
#define ECHO_PIN 25
#define UMBRAL_DISTANCIA 15 // Distancia en cm para detectar un auto

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
const int REMOTE_OPEN_COMMAND = -2;

// =========================================================
// --- PROTOTIPOS DE FUNCIONES ---
// =========================================================
void taskReadRFID(void *parameter);
void taskControlActuators(void *parameter);
void taskMqttManager(void *parameter);
void mqttReconnect();
void callback(char* topic, byte* message, unsigned int length);
void publishAccessEvent(int userIndex);
long getHcsr04Distance(); // Función para leer el sensor

// --- NUEVOS PROTOTIPOS DE APERTURA ---
void openBarrierWithSensorLogic(); // Para el RFID
void openBarrierRemote();          // Para Node-RED

// =========================================================
// --- SETUP ---
// =========================================================
void setup() {
  Serial.begin(115200);
  Serial.print("Conectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado! IP: ");
  Serial.println(WiFi.localIP());
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  
  SPI.begin();
  rfid.PCD_Init();
  plumaServo.attach(SERVO_PIN);
  plumaServo.write(0);
  pinMode(LED_ACCESS_GRANTED_PIN, OUTPUT);
  pinMode(LED_ACCESS_DENIED_PIN, OUTPUT);

  // --- INICIALIZAR PINES DEL SENSOR ---
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
  digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
  
  userIndexQueue = xQueueCreate(5, sizeof(int));
  Serial.println(F("\nSistema de estacionamiento listo."));
  
  xTaskCreate(taskReadRFID, "Read RFID Task", 4096, NULL, 1, NULL);
  xTaskCreate(taskControlActuators, "Control Actuators Task", 4096, NULL, 1, NULL);
  xTaskCreate(taskMqttManager, "MQTT Manager Task", 4096, NULL, 1, NULL);
}

// =========================================================
// --- TAREAS FREERTOS ---
// =========================================================

/**
 * TAREA 1: Leer RFID (Con estabilización)
 * (Esta tarea queda sin cambios respecto a la versión anterior)
 */
void taskReadRFID(void *parameter) {
  
  bool isCarStablePresent = false; 
  const int confirmationTime = 300; 

  for (;;) {
    if (!isCarStablePresent) {
      // --- ESTADO: NO HAY AUTO ---
      if (getHcsr04Distance() < UMBRAL_DISTANCIA) {
        vTaskDelay(pdMS_TO_TICKS(confirmationTime)); 
        if (getHcsr04Distance() < UMBRAL_DISTANCIA) {
          Serial.println("Auto detectado. Por favor, presente su tarjeta RFID.");
          isCarStablePresent = true; 
        }
      } else {
        vTaskDelay(pdMS_TO_TICKS(200)); 
      }
    } else {
      // --- ESTADO: HAY UN AUTO ---
      // 1. ¿Hay una tarjeta RFID?
      if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
        Serial.println("Leyendo tarjeta...");
        int userIndex = -1;
        
        for (int i = 0; i < numTarjetas; i++) {
          bool match = true;
          for (int j = 0; j < 4; j++) {
            if (rfid.uid.uidByte[j] != tarjetasAutorizadas[i].uid[j]) {
              match = false;
              break;
            }
          }
          if (match) { userIndex = i; break; }
        }
        
        xQueueSend(userIndexQueue, &userIndex, portMAX_DELAY);
        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();
        
        Serial.println("Tarjeta procesada. Esperando a que el auto pase...");
        isCarStablePresent = false; 
        vTaskDelay(pdMS_TO_TICKS(5000)); 

      } 
      // 2. No hay tarjeta, ¿El auto se fue?
      else if (getHcsr04Distance() >= UMBRAL_DISTANCIA) {
        vTaskDelay(pdMS_TO_TICKS(confirmationTime));
        if (getHcsr04Distance() >= UMBRAL_DISTANCIA) {
           Serial.println("Auto ya no está presente. Esperando nuevo auto...");
           isCarStablePresent = false; 
        }
      }
      // 3. No hay tarjeta y el auto sigue ahí
      else {
         vTaskDelay(pdMS_TO_TICKS(50));
      }
    }
  }
}

/**
 * TAREA 2: Controlar actuadores (MODIFICADA)
 * Llama a la función de apertura correcta según el caso.
 */
void taskControlActuators(void *parameter) {
  int receivedUserIndex;
  for (;;) {
    if (xQueueReceive(userIndexQueue, &receivedUserIndex, portMAX_DELAY) == pdPASS) {
      // Publicar el evento (permitido, denegado, remoto)
      publishAccessEvent(receivedUserIndex);

      // --- ESTA ES LA LÓGICA CORREGIDA ---
      
      // Si el acceso fue permitido (por RFID)
      if (receivedUserIndex >= 0) {
        openBarrierWithSensorLogic(); // <--- LLAMA A LA FUNCIÓN CON SENSOR
      
      // Si el acceso fue permitido (por Comando Remoto)
      } else if (receivedUserIndex == REMOTE_OPEN_COMMAND) {
        openBarrierRemote(); // <--- LLAMA A LA FUNCIÓN SIMPLE CON TIMER
      
      } else { 
        // Acceso denegado (userIndex == -1)
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

/**
 * TAREA 3: Gestionar MQTT (Sin cambios)
 */
void taskMqttManager(void *parameter) {
  for (;;) {
    if (!client.connected()) {
      mqttReconnect();
    }
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =========================================================
// --- FUNCIÓN: LEER SENSOR HC-SR04 ---
// =========================================================
long getHcsr04Distance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 1000000); 
  long distance = (duration * 0.0343) / 2;
  if (distance == 0) {
    return 999;
  }
  return distance;
}


// =========================================================
// --- FUNCIONES AUXILIARES DE APERTURA ---
// =========================================================

/**
 * NUEVA FUNCIÓN: Apertura para MODO REMOTO (Node-RED)
 * Simplemente abre, espera 5 seg, y cierra. No usa el sensor.
 */
void openBarrierRemote() {
  digitalWrite(LED_ACCESS_GRANTED_PIN, HIGH);
  digitalWrite(LED_ACCESS_DENIED_PIN, LOW);
  plumaServo.write(90);
  Serial.println("Pluma abierta (Remoto).");

  // Espera un tiempo fijo
  vTaskDelay(pdMS_TO_TICKS(5000)); 

  Serial.println("Cerrando pluma (Remoto).");
  plumaServo.write(0);
  digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
  digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
}


/**
 * FUNCIÓN RENOMBRADA: Apertura para RFID (CON LÓGICA DE SENSOR)
 * Usa el sensor para NO cerrar si hay un auto.
 */
void openBarrierWithSensorLogic() {
  digitalWrite(LED_ACCESS_GRANTED_PIN, HIGH);
  digitalWrite(LED_ACCESS_DENIED_PIN, LOW);
  plumaServo.write(90);
  Serial.println("Pluma abierta (RFID).");

  vTaskDelay(pdMS_TO_TICKS(1500)); 
  Serial.println("Esperando a que el auto pase...");

  long distancia = 0;
  bool autoDetectado = false; 

  do {
    distancia = getHcsr04Distance();
    Serial.printf("Distancia medida: %ld cm\n", distancia);

    if (distancia < UMBRAL_DISTANCIA) {
      if (!autoDetectado) {
        Serial.println("¡Auto detectado! Esperando a que pase...");
        autoDetectado = true;
      }
      digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(200));
      digitalWrite(LED_ACCESS_GRANTED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(200));

    } else if (autoDetectado) {
      Serial.println("Auto parece haber pasado. Dando 1 segundo de gracia...");
      vTaskDelay(pdMS_TO_TICKS(1000));
      break; 

    } else {
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    
  } while (distancia < UMBRAL_DISTANCIA || !autoDetectado); 
  
  // --- CERRAR PLUMA ---
  Serial.println("Camino libre. Cerrando pluma.");
  plumaServo.write(0);
  digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
  digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
}


// =========================================================
// --- FUNCIONES MQTT (Sin cambios) ---
// =========================================================

// Función de publicación MQTT (Sin cambios)
void publishAccessEvent(int userIndex) {
    if (!client.connected()) return;
    char timeBuffer[20];
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    const char* status;
    const char* userName;

    if (userIndex == REMOTE_OPEN_COMMAND) {
        status = "Permitido (Remoto)";
        userName = "Dashboard";
    } else if (userIndex != -1) {
        status = "Permitido";
        userName = tarjetasAutorizadas[userIndex].name;
    } else {
        status = "Denegado";
        userName = "Desconocido";
    }
    client.publish("estacionamiento/entrada/usuario", userName);
    client.publish("estacionamiento/entrada/tiempo", timeBuffer);
    client.publish("estacionamiento/entrada/status", status);
    Serial.printf("--- Evento publicado: Usuario: %s, Status: %s ---\n", userName, status);
}

// Función de reconexión MQTT (Sin cambios)
void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    char clientId[50];
    sprintf(clientId, "ESP32_Estacionamiento-%ld", random(1000));
    if (client.connect(clientId, mqttUser, mqttPassword)) {
      Serial.println(" conectado!");
      client.subscribe("estacionamiento/entrada/comando");
      Serial.println("Suscrito a 'estacionamiento/entrada/comando'");
    } else {
      Serial.print(" falló, rc=");
      Serial.print(client.state());
      Serial.println(" -> Intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

// Función de Callback (Sin cambios)
void callback(char* topic, byte* message, unsigned int length) {
  String stMessage;
  for (int i = 0; i < length; i++) {
    stMessage += (char)message[i];
  }
  Serial.printf("Mensaje recibido en [%s]: %s\n", topic, stMessage.c_str());

  if (String(topic) == "estacionamiento/entrada/comando") {
    if (stMessage == "abrir") {
      Serial.println("Comando de apertura remota recibido!");
      int command = REMOTE_OPEN_COMMAND;
      xQueueSend(userIndexQueue, &command, portMAX_DELAY);
    }
  }
}

void loop() {
  vTaskDelete(NULL);
}