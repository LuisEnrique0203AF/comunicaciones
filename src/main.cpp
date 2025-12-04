/**
 * ===================================================================================
 * Proyecto de Estacionamiento con 3 Sensores VL53L0X y FreeRTOS
 * Autores: 
 * Luis Enrique Aguilera Fierro						                                     21060671
 * Victor Francisco Duarte Villalobos                                          21060670
 * Laura Liliana Morones Báez						                                       21060693
 * 
 * Fecha: Diciembre 2025
 * Institución: Instituto Tecnologico de Chihuahua
 *
 * Descripción:
 * Este código controla 3 cajones de estacionamiento usando sensores VL53L0X (ToF).
 * - Usa FreeRTOS para crear una tarea independiente por sensor, más tareas para MQTT.
 * - Implementa lógica de ocupación:
 * - Si (distancia < umbral_cm) por más de 2 segundos -> OCUPADO
 * - Si (distancia > umbral_cm) -> VACANTE (inmediato)
 * - Usa un Mutex (i2cMutex) para proteger el bus I2C, ya que las 3 tareas
 * de sensor no pueden usarlo al mismo tiempo.
 * - Usa una Cola (stateChangeQueue) para que las tareas de sensor reporten
 * cambios de estado a una tarea de publicación (taskPublishState).
 * - Publica el estado de cada cajón ("0" o "1") y el total de cajones
 * disponibles en tópicos MQTT.
 * ===================================================================================
 */

// =========================================================
// --- INCLUDES (LIBRERÍAS REQUERIDAS) ---
// =========================================================
#include <Arduino.h>
#include <Wire.h>            // Para el bus I2C
#include <VL53L0X.h>         // Para los sensores de Tiempo de Vuelo (ToF)
#include <WiFi.h>            // Para la conectividad WiFi
#include "PubSubClient.h"    // Para el protocolo MQTT

// =========================================================
// --- CONFIGURACIÓN DE PINES Y LÓGICA ---
// =========================================================
// --- Pines XSHUT para cada sensor (para cambiar su dirección I2C) ---
#define XSHUT_1 25
#define XSHUT_2 26
#define XSHUT_3 27

// --- Parámetros de lógica de detección ---
#define DISTANCIA_UMBRAL_CM_DEF 9     // Distancia (cm) para considerar un objeto.
#define TIEMPO_OCUPADO_MS 2000      // Milisegundos que un objeto debe estar presente para confirmar "OCUPADO"
#define POLLING_RATE_MS 200         // Frecuencia de muestreo (cada 200ms la tarea revisa el sensor)
#define TOTAL_SENSORES 3            // Total de cajones a monitorear

// =========================================================
// --- CONFIGURACIÓN WIFI y MQTT ---
// =========================================================
const char* ssid = "EnlaceFTTH_CASA_2.4G";
const char* password = "6481211001";
const char* mqttServer = "192.168.1.241";
const int   mqttPort = 1883;
const char* mqttUser = "luisenrique";
const char* mqttPassword = "enrique02";

// =========================================================
// --- OBJETOS GLOBALES ---
// =========================================================
// --- Objetos de sensor (uno por cada sensor físico) ---
VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

// --- Objetos de Red ---
WiFiClient espClient;                 // Cliente WiFi base
PubSubClient client(espClient);       // Cliente MQTT sobre el cliente WiFi

// =========================================================
// --- RECURSOS DE FreeRTOS ---
// =========================================================
/**
 * Mutex para proteger el bus I2C.
 * Solo una tarea a la vez puede "tomar" el mutex para hablar por I2C.
 * Esto evita que las lecturas de los 3 sensores choquen entre sí.
 */
SemaphoreHandle_t i2cMutex;
/**
 * Cola para comunicar cambios de estado.
 * Las tareas 'sensorTask' (productoras) envían un mensaje a esta cola
 * cada vez que un cajón cambia de VACANTE a OCUPADO o viceversa.
 * La tarea 'taskPublishState' (consumidora) lee de esta cola.
 */
QueueHandle_t stateChangeQueue;

// =========================================================
// --- ESTRUCTURAS DE DATOS Y ESTADOS ---
// =========================================================
// --- Máquina de estados para cada cajón ---
enum State { VACANTE, PENDIENTE, OCUPADO };

/**
 * Estructura de datos para cada sensor.
 * Cada tarea 'sensorTask' recibe un puntero a una de estas estructuras.
 * Contiene toda la información que la tarea necesita para operar.
 */
struct SensorData {
  VL53L0X* sensor;      // Puntero al objeto del sensor (sensor1, sensor2, o sensor3)
  const char* name;     // Nombre para el Serial.print (ej. "Sensor 1")
  State state;          // Estado actual del cajón (VACANTE, PENDIENTE, OCUPADO)
  unsigned long timer;  // Cronómetro para el estado PENDIENTE
  uint8_t umbral_cm;    // Umbral de distancia específico para este sensor
  int id;               // ID único (1, 2, o 3) para identificarlo en la cola MQTT
};

/**
 * Estructura del mensaje que se envía por la cola 'stateChangeQueue'.
 */
struct StateChangeMessage {
  int sensorId;     // Qué sensor cambió (1, 2, o 3)
  State newState;   // Cuál es su nuevo estado
};

// --- Creación de las estructuras de datos para cada sensor ---
// Se pasa el ID (1, 2, 3) que se usará para MQTT
SensorData dataS1 = {&sensor1, "Sensor 1", VACANTE, 0, DISTANCIA_UMBRAL_CM_DEF, 1};
SensorData dataS2 = {&sensor2, "Sensor 2", VACANTE, 0, DISTANCIA_UMBRAL_CM_DEF, 2};
SensorData dataS3 = {&sensor3, "Sensor 3", VACANTE, 0, 9, 3}; // Umbral especial de 9cm

/**
 * Arreglo global que almacena el estado "maestro" de todos los cajones.
 * La tarea 'taskPublishState' usa este arreglo para calcular el total de
 * cajones disponibles.
 * Ignoramos el índice 0 para que sensorStates[1] corresponda al sensor 1.
 */
State sensorStates[TOTAL_SENSORES + 1] = {VACANTE, VACANTE, VACANTE, VACANTE};

// --- Prototipos de Funciones de Tareas y Callbacks ---
void taskMqttManager(void *pvParameters);
void taskPublishState(void *pvParameters);
void mqttReconnect();
void callback(char* topic, byte* message, unsigned int length);
void sensorTask(void *pvParameters); // Prototipo de la tarea del sensor


// =========================================================
// --- TAREA 1: TAREA DE SENSOR (UNA POR CADA SENSOR) ---
// =========================================================
/**
 * Esta función es el "molde" para las 3 tareas de los sensores.
 * Cada tarea se ejecuta en paralelo y maneja un solo sensor.
 */
void sensorTask(void *pvParameters) {
  // 1. Obtener los datos específicos de este sensor (pasados en xTaskCreate)
  SensorData* data = (SensorData*)pvParameters;

  uint16_t dist_mm = 0;
  bool timeout = false;
  bool objetoPresente = false;

  for (;;) { // Bucle infinito de la tarea
    
    // 2. Tomar el Mutex I2C (espera si está ocupado)
    // --- INICIO DE SECCIÓN CRÍTICA ---
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      // Ahora somos dueños del bus I2C, ningún otro sensor puede leer.
      dist_mm = data->sensor->readRangeContinuousMillimeters();
      timeout = data->sensor->timeoutOccurred();
      
      // Liberar el Mutex I2C para que otra tarea lo pueda usar
      xSemaphoreGive(i2cMutex);
    }
    // --- FIN DE SECCIÓN CRÍTICA ---

    
    // 3. Lógica de detección
    // (dist_mm < 8000) filtra lecturas inválidas (el sensor da >8190 en error)
    objetoPresente = (!timeout) && (dist_mm < 8000) && (dist_mm < (data->umbral_cm * 10));

    // 4. Máquina de estados
    switch (data->state) {
      
      case VACANTE:
        // Si estaba vacante y aparece un objeto...
        if (objetoPresente) {
          data->state = PENDIENTE; // Pasa a "pendiente" (revisando...)
          data->timer = millis();  // Inicia el cronómetro
        }
        break;

      case PENDIENTE:
        // Si sigue presente...
        if (objetoPresente) {
          // Revisa si ya pasó el tiempo de confirmación
          if (millis() - data->timer > TIEMPO_OCUPADO_MS) {
            data->state = OCUPADO; // Confirma la ocupación
            Serial.printf("--- %s: CAJÓN OCUPADO --- (%.1f cm)\n", data->name, dist_mm / 10.0);
            
            // 5. Notificar a la cola
            StateChangeMessage msg = {data->id, OCUPADO};
            xQueueSend(stateChangeQueue, &msg, 0); // Envía el mensaje (sin esperar)
          }
        } else {
          // Si el objeto desapareció antes del tiempo, fue falsa alarma.
          data->state = VACANTE;
        }
        break;

      case OCUPADO:
        // Si estaba ocupado y el objeto ya no está...
        if (!objetoPresente) {
          data->state = VACANTE; // Se desocupa (inmediatamente)
          Serial.printf("--- %s: CAJÓN DESOCUPADO ---\n", data->name);
          
          // 5. Notificar a la cola
          StateChangeMessage msg = {data->id, VACANTE};
          xQueueSend(stateChangeQueue, &msg, 0); // Envía el mensaje
        }
        break;
    }

    // 6. Pausar la tarea (ceder CPU) según el polling rate
    vTaskDelay(POLLING_RATE_MS / portTICK_PERIOD_MS);
  }
}

// =========================================================
// --- TAREA 2: PUBLICAR CAMBIOS DE ESTADO (MQTT) ---
// =========================================================
/**
 * Tarea consumidora. Espera (bloqueada) a que lleguen mensajes
 * a la 'stateChangeQueue' y los publica en MQTT.
 * Esta tarea centraliza toda la lógica de MQTT.
 */
void taskPublishState(void *pvParameters) {
  StateChangeMessage msg;

  // Al arrancar, publica el estado inicial (todos vacantes)
  // para que Node-RED sepa el estado al conectarse.
  for (int i = 1; i <= TOTAL_SENSORES; i++) {
     StateChangeMessage initialMsg = {i, sensorStates[i]};
     xQueueSend(stateChangeQueue, &initialMsg, 0); // Envía a la cola para ser procesado
  }

  for(;;) {
    // 1. Espera (bloquea) hasta que llegue un mensaje de una tarea de sensor
    if (xQueueReceive(stateChangeQueue, &msg, portMAX_DELAY) == pdPASS) {
      
      // 2. Actualiza el estado en el arreglo maestro
      sensorStates[msg.sensorId] = msg.newState;

      char topic[64];
      char payload[5]; // "1", "0" o un número
      
      // 3. Publicar estado individual (1 para OCUPADO, 0 para VACANTE)
      //    Tópico: "estacionamiento/cajones/cajon_1", "cajon_2", etc.
      sprintf(topic, "estacionamiento/cajones/cajon_%d", msg.sensorId);
      strcpy(payload, (msg.newState == OCUPADO) ? "1" : "0");
      
      if (client.connected()) {
        client.publish(topic, payload);
      } else {
        Serial.println("[MQTT] Desconectado. No se publicó estado individual.");
      }

      // 4. Calcular y publicar el total de DISPONIBLES
      //    Tópico: "estacionamiento/cajones/disponibles"
      int ocupados = 0;
      // Recorre el arreglo maestro para contar
      for (int i = 1; i <= TOTAL_SENSORES; i++) {
        if (sensorStates[i] == OCUPADO) {
          ocupados++;
        }
      }
      int disponibles = TOTAL_SENSORES - ocupados;
      sprintf(payload, "%d", disponibles); // Convierte el número a string

      if (client.connected()) {
        client.publish("estacionamiento/cajones/disponibles", payload);
      } else {
         Serial.println("[MQTT] Desconectado. No se publicó el total.");
      }
    }
  }
}

// =========================================================
// --- TAREA 3: GESTIONAR CONEXIÓN MQTT ---
// =========================================================
/**
 * Tarea dedicada a mantener la conexión MQTT viva.
 * Llama a client.loop() y reconecta si es necesario.
 */
void taskMqttManager(void *pvParameters) {
  for (;;) { // Bucle infinito
    if (!client.connected()) {
      mqttReconnect();
    }
    client.loop(); // Procesa mensajes entrantes y mantiene la conexión
    vTaskDelay(pdMS_TO_TICKS(50)); // Pausa breve
  }
}

// =========================================================
// --- FUNCIÓN DE CONFIGURACIÓN PRINCIPAL (SETUP) ---
// =========================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // Pines I2C (SDA=21, SCL=22)
  delay(100);

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

  // --- Configuración MQTT ---
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback); // Asigna la función de callback

  // --- Crear el Mutex ---
  i2cMutex = xSemaphoreCreateMutex();
  if (i2cMutex == NULL) {
    Serial.println("Error: No se pudo crear el Mutex I2C");
    while (1); // Detiene el programa
  }

  // --- Crear la Cola de Estado ---
  stateChangeQueue = xQueueCreate(5, sizeof(StateChangeMessage)); // 5 mensajes de buffer
  if (stateChangeQueue == NULL) {
    Serial.println("Error: No se pudo crear la Cola de Estado");
    while (1); // Detiene el programa
  }

  // --- Configurar pines XSHUT ---
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);

  // Apagar todos los sensores (poniendo XSHUT en LOW)
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  delay(10);

  Serial.println("Inicializando sensores VL53L0X (uno por uno)...");

  // --- SECUENCIA DE INICIALIZACIÓN DE SENSORES I2C MÚLTIPLES ---
  // Todos los sensores VL53L0X inician en la dirección 0x29.
  // Esta secuencia los activa uno por uno para cambiarles la dirección.
  
  // ===== Sensor 1 =====
  Serial.println("Activando sensor #1 (dirección 0x30)...");
  digitalWrite(XSHUT_1, HIGH); // Activa el primer sensor
  delay(10);
  if (!sensor1.init(true)) { // 'true' = modo de alta velocidad
     Serial.println("Error: no se detecta el sensor #1");
     // No detengo el programa, aunque podría fallar
  } else {
    sensor1.setAddress(0x30); // Le asigna una nueva dirección I2C
    sensor1.startContinuous(); // Inicia la medición continua
  }


  // ===== Sensor 2 =====
  Serial.println("Activando sensor #2 (dirección 0x31)...");
  digitalWrite(XSHUT_2, HIGH); // Activa el segundo sensor
  delay(10);
  // El bus I2C ahora ve dos sensores: 0x30 (S1) y 0x29 (S2)
  if (!sensor2.init(true)) {
    Serial.println("Error: no se detecta el sensor #2");
  } else {
    sensor2.setAddress(0x31); // Le asigna una nueva dirección I2C
    sensor2.startContinuous();
  }

  // ===== Sensor 3 =====
  Serial.println("Activando sensor #3 (dirección 0x32)...");
  digitalWrite(XSHUT_3, HIGH); // Activa el tercer sensor
  delay(10);
  // El bus I2C ahora ve tres sensores: 0x30 (S1), 0x31 (S2) y 0x29 (S3)
  if (!sensor3.init(true)) {
    Serial.println("Error: no se detecta el sensor #3");
  } else {
    sensor3.setAddress(0x32); // Le asigna una nueva dirección I2C
    sensor3.startContinuous();
  }

  Serial.println("¡Sensores inicializados!");
  Serial.println("--- Creando Tareas de RTOS ---");

  // --- Crear tareas de sensores ---
  // (Función, Nombre, Stack, Parámetros, Prioridad, Handle)
  xTaskCreate(sensorTask, "Sensor 1 Task", 2048, (void*)&dataS1, 1, NULL); 
  xTaskCreate(sensorTask, "Sensor 2 Task", 2048, (void*)&dataS2, 1, NULL);
  xTaskCreate(sensorTask, "Sensor 3 Task", 2048, (void*)&dataS3, 1, NULL);
  
  // --- Crear tareas de Red ---
  xTaskCreate(taskMqttManager, "MQTT Manager Task", 4096, NULL, 1, NULL);
  xTaskCreate(taskPublishState, "Publish State Task", 4096, NULL, 1, NULL);

  Serial.println("¡Tareas creadas! Sistema operativo iniciado.");
}


void loop() {
  // El loop principal no hace nada, todo se maneja con FreeRTOS
  vTaskDelete(NULL); // Borra la tarea del loop() para liberar recursos.
}

// =========================================================
// --- FUNCIONES DE UTILIDAD MQTT ---
// =========================================================

/**
 * Función de reconexión MQTT.
 * Se llama cuando el cliente se desconecta.
 */
void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    
    // ClientID único para los CAJONES
    char clientId[50];
    sprintf(clientId, "ESP32_Estacionamiento_Cajones-%ld", random(1000));
    
    if (client.connect(clientId, mqttUser, mqttPassword)) {
      Serial.println(" conectado!");
      // Este ESP32 solo publica, no necesita suscribirse a nada por ahora.
    } else {
      Serial.print(" falló, rc=");
      Serial.print(client.state());
      Serial.println(" -> Intentando de nuevo en 5 segundos");
      delay(5000); // Espera 5 segundos antes de reintentar
    }
  }
}

/**
 * Función de Callback de MQTT.
 * Se ejecuta si llega un mensaje de un tópico al que estemos suscritos.
 * En este código no nos suscribimos a nada, pero la función es necesaria.
 */
void callback(char* topic, byte* message, unsigned int length) {
  String stMessage;
  for (int i = 0; i < length; i++) {
    stMessage += (char)message[i];
  }
  Serial.printf("Mensaje recibido en [%s]: %s\n", topic, stMessage.c_str());
}