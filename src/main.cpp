/*
 * Proyecto de Estacionamiento con 3 Sensores VL53L0X y FreeRTOS
 * - Usa FreeRTOS para crear una tarea independiente por sensor.
 * - Implementa lógica de ocupación:
 * - Si < umbral_cm por 2 segundos -> OCUPADO
 * - Si ya no está < umbral_cm -> VACANTE (inmediato)
 * - Usa un Mutex para proteger el bus I2C.
 * - [NUEVO] Agrega una tarea para manejar la conexión WiFi/MQTT.
 * - [NUEVO] Agrega una tarea para publicar cambios de estado en tópicos MQTT.
 */

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <WiFi.h>             // <-- NUEVO
#include "PubSubClient.h"     // <-- NUEVO

// --- Pines XSHUT para cada sensor ---
#define XSHUT_1 25
#define XSHUT_2 26
#define XSHUT_3 27

// --- Parámetros de lógica ---
#define DISTANCIA_UMBRAL_CM_DEF 9     // Umbral por defecto (en cm)
#define TIEMPO_OCUPADO_MS 2000         // Tiempo para confirmar ocupación
#define POLLING_RATE_MS 200            // Frecuencia de muestreo
#define TOTAL_SENSORES 3               // <-- NUEVO (Total de cajones)

// --- Configuración WiFi y MQTT (Tomada de tu ejemplo) ---
const char* ssid = "EnlaceFTTH_CASA_2.4G";      // <-- NUEVO
const char* password = "6481211001";           // <-- NUEVO
const char* mqttServer = "192.168.1.241";     // <-- NUEVO
const int   mqttPort = 1883;                  // <-- NUEVO
const char* mqttUser = "luisenrique";         // <-- NUEVO
const char* mqttPassword = "enrique02";       // <-- NUEVO

// --- Objetos de sensor ---
VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

// --- Objetos de Red ---
WiFiClient espClient;                         // <-- NUEVO
PubSubClient client(espClient);               // <-- NUEVO

// --- RTOS ---
SemaphoreHandle_t i2cMutex; // Mutex para proteger el bus I2C
QueueHandle_t stateChangeQueue; // <-- NUEVO (Cola para reportar cambios de estado)

// --- Estados del sensor ---
enum State { VACANTE, PENDIENTE, OCUPADO };

// --- Estructura de datos para cada sensor ---
struct SensorData {
  VL53L0X* sensor;
  const char* name;
  State state;
  unsigned long timer;
  uint8_t umbral_cm; 
  int id; // <-- NUEVO (Para identificar el sensor en la cola)
};

// --- Estructura para la cola de cambios de estado ---
struct StateChangeMessage { // <-- NUEVO
  int sensorId;
  State newState;
};

// --- Datos por sensor ---
// MODIFICADO: Se añade el ID único (1, 2, 3)
SensorData dataS1 = {&sensor1, "Sensor 1", VACANTE, 0, DISTANCIA_UMBRAL_CM_DEF, 1};
SensorData dataS2 = {&sensor2, "Sensor 2", VACANTE, 0, DISTANCIA_UMBRAL_CM_DEF, 2};
SensorData dataS3 = {&sensor3, "Sensor 3", VACANTE, 0, 9, 3}; // Umbral especial

// --- Arreglo para guardar el estado de todos los sensores ---
State sensorStates[TOTAL_SENSORES + 1] = {VACANTE, VACANTE, VACANTE, VACANTE}; // <-- NUEVO (Ignoramos índice 0)

// --- Prototipos de Funciones ---
void taskMqttManager(void *pvParameters);     // <-- NUEVO
void taskPublishState(void *pvParameters);    // <-- NUEVO
void mqttReconnect();                         // <-- NUEVO
void callback(char* topic, byte* message, unsigned int length); // <-- NUEVO

// --- Función de tarea para cada sensor ---
// MODIFICADO: Ahora envía un mensaje a 'stateChangeQueue' cuando hay un cambio
void sensorTask(void *pvParameters) {
  SensorData* data = (SensorData*)pvParameters;

  uint16_t dist_mm = 0;
  bool timeout = false;
  bool objetoPresente = false;

  for (;;) {
    // --- Lectura protegida por Mutex ---
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      dist_mm = data->sensor->readRangeContinuousMillimeters();
      timeout = data->sensor->timeoutOccurred();
      xSemaphoreGive(i2cMutex);
    }

    // --- Lógica de detección ---
    objetoPresente = (!timeout) && (dist_mm < 8000) && (dist_mm < (data->umbral_cm * 10));

    // --- Máquina de estados ---
    switch (data->state) {
      case VACANTE:
        if (objetoPresente) {
          data->state = PENDIENTE;
          data->timer = millis();
        }
        break;

      case PENDIENTE:
        if (objetoPresente) {
          if (millis() - data->timer > TIEMPO_OCUPADO_MS) {
            data->state = OCUPADO;
            Serial.printf("--- %s: CAJÓN OCUPADO --- (%.1f cm)\n", data->name, dist_mm / 10.0);
            
            // <-- NUEVO: Notificar a la cola
            StateChangeMessage msg = {data->id, OCUPADO};
            xQueueSend(stateChangeQueue, &msg, 0); 
          }
        } else {
          data->state = VACANTE;
        }
        break;

      case OCUPADO:
        if (!objetoPresente) {
          data->state = VACANTE;
          Serial.printf("--- %s: CAJÓN DESOCUPADO ---\n", data->name);
          
          // <-- NUEVO: Notificar a la cola
          StateChangeMessage msg = {data->id, VACANTE};
          xQueueSend(stateChangeQueue, &msg, 0);
        }
        break;
    }

    vTaskDelay(POLLING_RATE_MS / portTICK_PERIOD_MS);
  }
}

// --- TAREA NUEVA: Publicar estados en MQTT ---
void taskPublishState(void *pvParameters) {
  StateChangeMessage msg;

  // Al arrancar, publica el estado inicial (todos vacantes)
  for (int i = 1; i <= TOTAL_SENSORES; i++) {
     StateChangeMessage initialMsg = {i, sensorStates[i]};
     xQueueSend(stateChangeQueue, &initialMsg, 0); // Envía a la cola para ser procesado
  }

  for(;;) {
    // Espera a que llegue un mensaje de cambio de estado
    if (xQueueReceive(stateChangeQueue, &msg, portMAX_DELAY) == pdPASS) {
      
      // 1. Actualiza el estado interno
      sensorStates[msg.sensorId] = msg.newState;

      char topic[64];
      char payload[5]; // "1" o "0" para ocupado/vacante, o un número para el total
      
      // 2. Publicar estado individual (1 para OCUPADO, 0 para VACANTE)
      //    Tópico: "estacionamiento/cajones/cajon_1", "estacionamiento/cajones/cajon_2", etc.
      //    Payload: "1" (Ocupado) o "0" (Vacante)
      sprintf(topic, "estacionamiento/cajones/cajon_%d", msg.sensorId);
      strcpy(payload, (msg.newState == OCUPADO) ? "1" : "0");
      
      if (client.connected()) {
        client.publish(topic, payload);
      } else {
        Serial.println("[MQTT] Desconectado. No se publicó estado individual.");
      }

      // 3. Calcular y publicar el total de DISPONIBLES
      //    Tópico: "estacionamiento/cajones/disponibles"
      //    Payload: "3", "2", "1", "0"
      int ocupados = 0;
      for (int i = 1; i <= TOTAL_SENSORES; i++) {
        if (sensorStates[i] == OCUPADO) {
          ocupados++;
        }
      }
      int disponibles = TOTAL_SENSORES - ocupados;
      sprintf(payload, "%d", disponibles);

      if (client.connected()) {
        client.publish("estacionamiento/cajones/disponibles", payload);
      } else {
         Serial.println("[MQTT] Desconectado. No se publicó el total.");
      }
    }
  }
}

// --- TAREA NUEVA: Gestionar conexión MQTT ---
void taskMqttManager(void *pvParameters) {
  for (;;) {
    if (!client.connected()) {
      mqttReconnect();
    }
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}


void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // Pines I2C (no los cambio)
  delay(100);

  // --- Conexión WiFi ---
  Serial.print("Conectando a ");             // <-- NUEVO
  Serial.println(ssid);                     // <-- NUEVO
  WiFi.begin(ssid, password);                 // <-- NUEVO
  while (WiFi.status() != WL_CONNECTED) {     // <-- NUEVO
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado! IP: "); // <-- NUEVO
  Serial.println(WiFi.localIP());           // <-- NUEVO

  // --- Configuración MQTT ---
  client.setServer(mqttServer, mqttPort);     // <-- NUEVO
  client.setCallback(callback);               // <-- NUEVO

  // --- Crear el Mutex ---
  i2cMutex = xSemaphoreCreateMutex();
  if (i2cMutex == NULL) {
    Serial.println("Error: No se pudo crear el Mutex I2C");
    while (1);
  }

  // --- Crear la Cola de Estado ---
  stateChangeQueue = xQueueCreate(5, sizeof(StateChangeMessage)); // <-- NUEVO
  if (stateChangeQueue == NULL) {
    Serial.println("Error: No se pudo crear la Cola de Estado");
    while (1);
  }

  // --- Configurar pines XSHUT ---
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);

  // Apagar todos los sensores
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  delay(10);

  Serial.println("Inicializando sensores VL53L0X...");

  // ===== Sensor 1 =====
  Serial.println("Activando sensor #1 (dirección 0x30)...");
  digitalWrite(XSHUT_1, HIGH);
  delay(10);
  if (!sensor1.init(true)) {
     Serial.println("Error: no se detecta el sensor #1");
     // while (1); // No detengo el programa si un sensor falla
  } else {
    sensor1.setAddress(0x30);
    sensor1.startContinuous();
  }


  // ===== Sensor 2 =====
  Serial.println("Activando sensor #2 (dirección 0x31)...");
  digitalWrite(XSHUT_2, HIGH);
  delay(10);
  if (!sensor2.init(true)) {
    Serial.println("Error: no se detecta el sensor #2");
    // while (1);
  } else {
    sensor2.setAddress(0x31);
    sensor2.startContinuous();
  }

  // ===== Sensor 3 =====
  Serial.println("Activando sensor #3 (dirección 0x32)...");
  digitalWrite(XSHUT_3, HIGH);
  delay(10);
  if (!sensor3.init(true)) {
    Serial.println("Error: no se detecta el sensor #3");
    // while (1);
  } else {
    sensor3.setAddress(0x32);
    sensor3.startContinuous();
  }

  Serial.println("¡Sensores inicializados!");
  Serial.println("--- Creando Tareas de RTOS ---");

  // --- Crear tareas ---
  xTaskCreate(sensorTask, "Sensor 1 Task", 2048, (void*)&dataS1, 1, NULL); 
  xTaskCreate(sensorTask, "Sensor 2 Task", 2048, (void*)&dataS2, 1, NULL);
  xTaskCreate(sensorTask, "Sensor 3 Task", 2048, (void*)&dataS3, 1, NULL);
  
  // --- Crear tareas de Red ---
  xTaskCreate(taskMqttManager, "MQTT Manager Task", 4096, NULL, 1, NULL); // <-- NUEVO
  xTaskCreate(taskPublishState, "Publish State Task", 4096, NULL, 1, NULL); // <-- NUEVO

  Serial.println("¡Tareas creadas! Sistema operativo iniciado.");
}

// --- MODIFICADO: Eliminamos la tarea del loop ---
void loop() {
  vTaskDelete(NULL); // El loop principal no hace nada, todo está en las tareas.
}

// =========================================================
// --- FUNCIONES MQTT (NUEVAS) ---
// =========================================================

void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    
    // ClientID único para los CAJONES
    char clientId[50];
    sprintf(clientId, "ESP32_Estacionamiento_Cajones-%ld", random(1000));
    
    if (client.connect(clientId, mqttUser, mqttPassword)) {
      Serial.println(" conectado!");
      // Este ESP32 solo publica, no necesita suscribirse a nada.
    } else {
      Serial.print(" falló, rc=");
      Serial.print(client.state());
      Serial.println(" -> Intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

// Función de Callback (no la usamos para recibir, pero es necesaria)
void callback(char* topic, byte* message, unsigned int length) {
  String stMessage;
  for (int i = 0; i < length; i++) {
    stMessage += (char)message[i];
  }
  Serial.printf("Mensaje recibido en [%s]: %s\n", topic, stMessage.c_str());
}