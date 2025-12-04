/**
 * ===================================================================================
 * Proyecto de Estacionamiento IoT - Módulo de SALIDA
 * Autores:
 * Luis Enrique Aguilera Fierro						                                     21060671
 * Victor Francisco Duarte Villalobos                                          21060670
 * Laura Liliana Morones Báez						                                       21060693
 * Fecha: Diciembre 2025
 * Instituto Tecnologico de Chihuahua
 *
 * Descripción:
 * Este código controla el sistema de SALIDA de un estacionamiento IoT usando un ESP32.
 * Utiliza FreeRTOS para manejar múltiples tareas concurrentemente:
 * 1. Leer RFID (precedido por detección de auto con ultrasonido).
 * 2. Controlar actuadores (LEDs, Servo de la pluma).
 * 3. Gestionar la conexión y comunicación MQTT (publicar eventos, recibir comandos).
 * 4. Leer y publicar datos de un sensor ambiental BME280 (Temp, Hum, Pres).
 *
 * Hardware:
 * - ESP32
 * - Lector RFID MFRC522 (SPI)
 * - Sensor Ultrasónico HC-SR04
 * - Servomotor (Pluma)
 * - Sensor Ambiental BME280 (I2C)
 * - LEDs (Acceso permitido/denegado)
 * ===================================================================================
 */

// =========================================================
// --- INCLUDES (LIBRERÍAS REQUERIDAS) ---
// =========================================================
#include <Arduino.h>      // Framework principal de Arduino
#include <SPI.h>          // Protocolo SPI (para el lector RFID)
#include <MFRC522.h>      // Librería para el lector RFID MFRC522
#include <ESP32Servo.h>   // Librería de Servo para ESP32
#include <WiFi.h>         // Para conectividad WiFi
#include <time.h>         // Para obtener la hora de internet (NTP)
#include "PubSubClient.h" // Para el protocolo MQTT
#include <Wire.h>         // Protocolo I2C (para el BME280)
#include <Adafruit_Sensor.h> // Librería base de sensores Adafruit
#include <Adafruit_BME280.h> // Librería para el sensor BME280

// =========================================================
// --- CONFIGURACIÓN GLOBAL (DATOS SENSIBLES Y DE RED) ---
// =========================================================
// --- Credenciales WiFi ---
const char* ssid = "EnlaceFTTH_CASA_2.4G";
const char* password = "6481211001";
// --- Configuración del Broker MQTT ---
const char* mqttServer = "192.168.1.241"; // IP del broker (ej. Raspberry Pi)
const int   mqttPort = 1883;
const char* mqttUser = "luisenrique";
const char* mqttPassword = "enrique02";

// --- Configuración de Servidores de Tiempo (NTP) ---
// Se añaden dos servidores: uno principal y uno de respaldo por si falla el primero.
const char* ntpServer1 = "pool.ntp.org";    // Servidor Principal
const char* ntpServer2 = "time.google.com"; // Servidor de Respaldo (Backup)

const long  gmtOffset_sec = -6 * 3600; // Offset GMT (Chihuahua es GMT-6)
const int   daylightOffset_sec = 0;   // Offset de horario de verano (0 = desactivado)

// =========================================================
// --- DEFINICIÓN DE PINES Y OBJETOS GLOBALES ---
// =========================================================
// --- Pines del Lector RFID MFRC522 (SPI) ---
#define SS_PIN    5   // Slave Select (SDA/SS)
#define RST_PIN   4   // Reset
// --- Pines de Actuadores y LEDs ---
#define LED_ACCESS_GRANTED_PIN  13  // LED Verde (Acceso permitido)
#define LED_ACCESS_DENIED_PIN   12  // LED Rojo (Acceso denegado)
#define SERVO_PIN               2   // Pin de señal del Servomotor
// --- Pines para el Sensor Ultrasónico HC-SR04 ---
#define TRIG_PIN 26 // Pin de Disparo (Trigger)
#define ECHO_PIN 25 // Pin de Eco (Echo)
// --- Lógica del Sensor Ultrasónico ---
#define UMBRAL_DISTANCIA 15 // Distancia (cm) para detectar un auto. Si es menor, hay auto.

// --- OBJETOS DE SENSORES Y ACTUADORES ---
MFRC522 rfid(SS_PIN, RST_PIN);  // Instancia del lector RFID
Servo plumaServo;               // Instancia del servomotor
Adafruit_BME280 bme;            // Instancia del sensor BME280 (modo I2C)
#define BME_ADDRESS 0x76        // Dirección I2C del BME280 (confirmada)

// --- OBJETOS DE RED ---
WiFiClient espClient;                 // Cliente WiFi base
PubSubClient client(espClient);       // Cliente MQTT sobre el cliente WiFi

// =========================================================
// --- USUARIOS AUTORIZADOS Y COLA DE EVENTOS ---
// =========================================================
// Estructura para almacenar la info de un usuario
struct AuthorizedUser {
  byte uid[4];        // El ID único de 4 bytes de la tarjeta
  const char* name;   // Nombre del usuario (para MQTT)
};

// Lista (array) de usuarios autorizados
const int numTarjetas = 4;
AuthorizedUser tarjetasAutorizadas[numTarjetas] = {
  {{0x83, 0xE8, 0xED, 0x2C}, "Usuario 1 (Acm)"},
  {{0x93, 0xB8, 0xB7, 0x14}, "Usuario 2 (asm)"},
  {{0xC3, 0x28, 0x1D, 0x2A}, "Usuario 3 (tbsm)"},
  {{0xF3, 0xB9, 0x7C, 0x29}, "Usuario 4 (tbcm)"}
};

/**
 * Cola de FreeRTOS.
 * Se usa para comunicar la Tarea 1 (RFID) con la Tarea 2 (Actuadores).
 * Tarea 1 escribe en la cola, Tarea 2 lee de la cola.
 * Almacena el índice del usuario (0-3 si es válido, -1 si es denegado, -2 si es remoto).
 */
QueueHandle_t userIndexQueue;

// Valor especial para la cola que indica una apertura remota (desde Node-RED)
const int REMOTE_OPEN_COMMAND = -2;

// =========================================================
// --- PROTOTIPOS DE FUNCIONES (DECLARACIONES) ---
// =========================================================
// Prototipos de las tareas de FreeRTOS
void taskReadRFID(void *parameter);
void taskControlActuators(void *parameter);
void taskMqttManager(void *parameter);
void taskReadBME280(void *parameter); // Tarea para el sensor ambiental
// Prototipos de funciones de utilidad MQTT
void mqttReconnect();
void callback(char* topic, byte* message, unsigned int length);
void publishAccessEvent(int userIndex);
// Prototipos de funciones de sensores y actuadores
long getHcsr04Distance();
void openBarrierWithSensorLogic();
void openBarrierRemote();

// =========================================================
// --- FUNCIÓN DE CONFIGURACIÓN PRINCIPAL (SETUP) ---
// =========================================================
void setup() {
  // --- 1. INICIALIZACIÓN BÁSICA ---
  Serial.begin(115200); // Inicia comunicación serial para debugging

  // --- 2. CONEXIÓN WIFI ---
  Serial.print("Conectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado! IP: ");
  Serial.println(WiFi.localIP());

  // --- 3. CONFIGURACIÓN DE TIEMPO (NTP) ---
  // Se usa para poner la fecha/hora correcta en los eventos MQTT.
  // Se configuran el servidor principal y el de respaldo.
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  // --- 4. CONFIGURACIÓN MQTT ---
  client.setServer(mqttServer, mqttPort); // Apunta el cliente al broker
  client.setCallback(callback);           // Asigna la función que maneja mensajes entrantes

  // --- 5. INICIALIZACIÓN DE PERIFÉRICOS ---
  SPI.begin();       // Inicia el bus SPI (para RFID)
  Wire.begin();      // Inicia el bus I2C (para BME280, pines 21=SDA, 22=SCL)
  rfid.PCD_Init(); // Inicia el lector RFID

  // Iniciar BME280
  if (!bme.begin(BME_ADDRESS)) {
    Serial.println("¡Error! No se pudo encontrar el sensor BME280.");
    // No detenemos el programa, solo avisamos
  } else {
    Serial.println("Sensor BME280 encontrado. ¡Listo!");
  }

  // Configuración de actuadores y pines
  plumaServo.attach(SERVO_PIN); // Asocia el servo al pin
  plumaServo.write(0);          // Asegura que la pluma inicie cerrada
  pinMode(LED_ACCESS_GRANTED_PIN, OUTPUT);
  pinMode(LED_ACCESS_DENIED_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);  // Pin de trigger del ultrasonido
  pinMode(ECHO_PIN, INPUT);   // Pin de echo del ultrasonido

  // Estado inicial de los LEDs
  digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);  // LED Verde apagado
  digitalWrite(LED_ACCESS_DENIED_PIN, HIGH); // LED Rojo encendido (listo/esperando)

  // --- 6. CREACIÓN DE COLA (QUEUE) ---
  // Crea una cola de 5 elementos, donde cada elemento es un 'int'
  userIndexQueue = xQueueCreate(5, sizeof(int));

  // --- 7. INICIO DE TAREAS (FreeRTOS) ---
  Serial.println(F("\nSistema de estacionamiento [SALIDA] listo."));

  // (Nombre Tarea, Función, Stack, Parámetros, Prioridad, Handle)
  xTaskCreate(taskReadRFID, "Read RFID Task", 4096, NULL, 1, NULL);
  xTaskCreate(taskControlActuators, "Control Actuators Task", 4096, NULL, 1, NULL);
  xTaskCreate(taskMqttManager, "MQTT Manager Task", 4096, NULL, 1, NULL);
  xTaskCreate(taskReadBME280, "BME280 Task", 4096, NULL, 1, NULL);
}

// =========================================================
// --- TAREA 1: Leer RFID (Con estabilización) ---
// =========================================================
/**
 * Tarea que gestiona la lógica de detección de auto Y lectura de RFID.
 * Esta tarea funciona como una máquina de estados simple:
 * 1. Espera a que un auto se detecte (distancia < umbral).
 * 2. Una vez detectado, espera a que se presente una tarjeta RFID.
 * 3. Valida la tarjeta y envía el resultado (índice o -1) a la Tarea 2.
 * 4. Espera a que el auto se vaya antes de repetir el ciclo.
 */
void taskReadRFID(void *parameter) {
  // Flag que actúa como máquina de estados (true = auto presente, false = esperando auto)
  bool isCarStablePresent = false;
  // Tiempo (ms) para estabilizar la lectura del sensor y evitar rebotes
  const int confirmationTime = 300; 

  for (;;) { // Bucle infinito de la tarea
    if (!isCarStablePresent) {
      // --- ESTADO 1: NO HAY AUTO ---
      // Si el sensor detecta algo cerca...
      if (getHcsr04Distance() < UMBRAL_DISTANCIA) {
        vTaskDelay(pdMS_TO_TICKS(confirmationTime)); // Espera de estabilización
        // Vuelve a checar si el auto sigue ahí
        if (getHcsr04Distance() < UMBRAL_DISTANCIA) {
          Serial.println("Auto detectado. Por favor, presente su tarjeta RFID.");
          isCarStablePresent = true; // Cambia al estado "HAY AUTO"
        }
      } else {
        // Si no hay auto, duerme la tarea brevemente para ceder CPU
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    } else {
      // --- ESTADO 2: HAY UN AUTO ---
      // Si hay un auto Y se presenta una nueva tarjeta RFID...
      if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
        Serial.println("Leyendo tarjeta...");
        int userIndex = -1; // -1 = Denegado (por defecto)

        // Compara el UID leído con la lista de autorizados
        for (int i = 0; i < numTarjetas; i++) {
          bool match = true;
          for (int j = 0; j < 4; j++) {
            if (rfid.uid.uidByte[j] != tarjetasAutorizadas[i].uid[j]) {
              match = false;
              break; // Si un byte no coincide, no es esta tarjeta
            }
          }
          if (match) {
            userIndex = i; // Guarda el índice del usuario autorizado
            break; // Deja de buscar
          }
        }

        // Envía el resultado (índice 0-3 o -1) a la Tarea 2 (Actuadores)
        // portMAX_DELAY = bloquea si la cola está llena (nunca debería pasar)
        xQueueSend(userIndexQueue, &userIndex, portMAX_DELAY);
        
        // Detiene la lectura de la tarjeta actual
        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();

        Serial.println("Tarjeta procesada. Esperando a que el auto pase...");
        isCarStablePresent = false; // Vuelve al estado "NO HAY AUTO"
        // Da un tiempo (5s) para que la Tarea 2 tome control y abra la pluma
        // antes de que esta tarea vuelva a intentar leer la distancia.
        vTaskDelay(pdMS_TO_TICKS(5000)); 

      }
      // Si el auto se va *antes* de presentar la tarjeta...
      else if (getHcsr04Distance() >= UMBRAL_DISTANCIA) {
        vTaskDelay(pdMS_TO_TICKS(confirmationTime)); // Espera de estabilización
        // Vuelve a checar si el auto de verdad se fue
        if (getHcsr04Distance() >= UMBRAL_DISTANCIA) {
           Serial.println("Auto ya no está presente. Esperando nuevo auto...");
           isCarStablePresent = false; // Vuelve al estado "NO HAY AUTO"
        }
      }
      else {
        // Si el auto sigue ahí pero no hay tarjeta, duerme 50ms
         vTaskDelay(pdMS_TO_TICKS(50));
      }
    }
  }
}

// =========================================================
// --- TAREA 2: Controlar actuadores (Servo y LEDs) ---
// =========================================================
/**
 * Tarea que espera eventos en la cola (userIndexQueue).
 * Esta tarea permanece bloqueada hasta que la Tarea 1 (RFID) o la Tarea 3 (MQTT)
 * envían un comando a la cola.
 */
void taskControlActuators(void *parameter) {
  int receivedUserIndex; // Variable para almacenar el índice recibido
  
  for (;;) { // Bucle infinito de la tarea
    // Espera (bloquea) hasta que un dato llegue a la cola
    if (xQueueReceive(userIndexQueue, &receivedUserIndex, portMAX_DELAY) == pdPASS) {
      
      // 1. Publica el evento en MQTT (sea cual sea el resultado)
      publishAccessEvent(receivedUserIndex);

      // 2. Decide qué acción tomar
      if (receivedUserIndex >= 0) {
        // --- ACCESO PERMITIDO (RFID) ---
        // Llama a la función que abre la pluma con lógica de sensor
        openBarrierWithSensorLogic();
        
      } else if (receivedUserIndex == REMOTE_OPEN_COMMAND) {
        // --- ACCESO PERMITIDO (REMOTO) ---
        // Llama a la función simple de apertura remota
        openBarrierRemote();
        
      } else {
        // --- ACCESO DENEGADO (userIndex == -1) ---
        // Parpadea el LED rojo 3 veces
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

// =========================================================
// --- TAREAS 3: Gestionar conexión y bucle MQTT ---
// =========================================================
/**
 * Tarea dedicada a mantener la conexión MQTT viva.
 * Se asegura de reconectar si se pierde la conexión y llama a
 * client.loop() para procesar mensajes entrantes y mantener el 'heartbeat'.
 */
void taskMqttManager(void *parameter) {
  for (;;) { // Bucle infinito de la tarea
    if (!client.connected()) {
      mqttReconnect(); // Intenta reconectar si está desconectado
    }
    client.loop(); // IMPORTANTE: Mantiene la conexión MQTT
    
    // Duerme la tarea 50ms. client.loop() no debe llamarse muy rápido.
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

// =========================================================
// --- TAREA 4: LEER SENSOR BME280 Y PUBLICAR ---
// =========================================================
/**
 * Tarea dedicada a leer el sensor ambiental BME280.
 * Lee los datos de temperatura, presión y humedad cada 60 segundos
 * y los publica en sus respectivos tópicos MQTT.
 */
void taskReadBME280(void *parameter) {
  // Delay inicial: Espera 15 segundos la primera vez
  // para dar tiempo a que WiFi y MQTT se conecten antes de publicar.
  vTaskDelay(pdMS_TO_TICKS(15000)); 
  
  for (;;) { // Bucle infinito de la tarea
    // Leer los datos del sensor
    float temperatura = bme.readTemperature();       // en Celsius
    float presion = bme.readPressure() / 100.0F; // en hPa
    float humedad = bme.readHumidity();          // en %

    // Imprimir en serial (para debugging)
    Serial.printf("[BME280] Temp: %.2f C, Pres: %.2f hPa, Hum: %.2f %%\n", temperatura, presion, humedad);

    // Publicar en MQTT (solo si el cliente está conectado)
    if (client.connected()) {
      // Creamos buffers (arrays de char) para convertir float a string
      char tempStr[8];
      char presStr[8];
      char humStr[8];

      // dtostrf(variable_float, ancho_total, decimales, buffer_destino)
      dtostrf(temperatura, 4, 2, tempStr); 
      dtostrf(presion, 6, 2, presStr);
      dtostrf(humedad, 4, 2, humStr);

      // Publica en los tópicos de "ambiente"
      client.publish("estacionamiento/ambiente/temperatura", tempStr);
      client.publish("estacionamiento/ambiente/humedad", humStr);
      client.publish("estacionamiento/ambiente/presion", presStr);
    }

    // Espera 1 minuto (60,000 ms) para la siguiente lectura
    vTaskDelay(pdMS_TO_TICKS(60000)); 
  }
}


// =========================================================
// --- FUNCIÓN: LEER SENSOR HC-SR04 ---
// =========================================================
/**
 * Función de utilidad para leer el sensor ultrasónico.
 * Ejecuta el pulso de trigger y mide el pulso de echo.
 * Retorna: La distancia en centímetros (cm).
 */
long getHcsr04Distance() {
  // --- Secuencia de Trigger ---
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); // Envía un pulso de 10us
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // --- Lectura del Eco ---
  // Mide el tiempo (us) que tarda en regresar el eco
  // Se añade un timeout (30000us = 30ms) para evitar que se bloquee
  // si el eco nunca regresa.
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); 

  // --- Cálculo de Distancia ---
  // Fórmula: (tiempo * velocidad_del_sonido) / 2
  // (duration * 0.0343 cm/us) / 2
  long distance = (duration * 0.0343) / 2;

  // Manejo de error (si pulseIn falla por timeout, retorna 0)
  if (distance == 0) {
    return 999; // Retorna un valor alto (lejos) si el pulso falló
  }
  return distance;
}


// =========================================================
// --- FUNCIONES AUXILIARES DE APERTURA DE PLUMA ---
// =========================================================

/**
 * Apertura para MODO REMOTO (Node-RED).
 * Secuencia simple: Abre, espera 5 segundos, cierra.
 * No utiliza el sensor ultrasónico.
 */
void openBarrierRemote() {
  // 1. Abrir
  digitalWrite(LED_ACCESS_GRANTED_PIN, HIGH);
  digitalWrite(LED_ACCESS_DENIED_PIN, LOW);
  plumaServo.write(90);
  Serial.println("Pluma abierta (Remoto).");

  // 2. Esperar
  vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 5 segundos fijos

  // 3. Cerrar
  Serial.println("Cerrando pluma (Remoto).");
  plumaServo.write(0);
  digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
  digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
}


/**
 * Apertura para RFID (CON LÓGICA DE SENSOR Y TIMEOUT).
 * Secuencia compleja:
 * 1. Abre la pluma.
 * 2. Espera a que el sensor detecte el auto DEBAJO (dist < umbral).
 * 3. Una vez que el auto está debajo, espera a que DEJE de estarlo (dist >= umbral).
 * 4. Cierra la pluma.
 * 5. Incluye un TIMEOUT de 15s por si el auto nunca cruza.
 */
void openBarrierWithSensorLogic() {
  // 1. Abrir
  digitalWrite(LED_ACCESS_GRANTED_PIN, HIGH);
  digitalWrite(LED_ACCESS_DENIED_PIN, LOW);
  plumaServo.write(90);
  Serial.println("Pluma abierta (RFID).");

  // Pequeña pausa antes de empezar a sensar
  vTaskDelay(pdMS_TO_TICKS(1500));
  Serial.println("Esperando a que el auto pase...");

  long distancia = 0;
  bool autoDetectado = false; // Flag para saber si el auto pasó POR DEBAJO

  // --- Lógica de Timeout (Seguridad) ---
  uint32_t startTime = millis(); // Guarda el tiempo de inicio
  const uint32_t TIMEOUT_MS = 15000; // Timeout de 15 segundos
  // --- Fin Lógica de Timeout ---

  // Bucle principal que monitorea el paso del auto
  do {
    distancia = getHcsr04Distance(); // Lee la distancia
    Serial.printf("Distancia medida: %ld cm\n", distancia);

    // --- Lógica de detección ---
    if (distancia < UMBRAL_DISTANCIA) {
      // ESTADO: Auto está DEBAJO de la pluma
      if (!autoDetectado) {
        Serial.println("¡Auto detectado! Esperando a que pase...");
        autoDetectado = true; // El auto está pasando por debajo
      }
      // Parpadea el LED verde mientras el auto está debajo
      digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(200));
      digitalWrite(LED_ACCESS_GRANTED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(200));

    } else if (autoDetectado) {
      // ESTADO: El auto ESTABA debajo (autoDetectado=true) 
      // y AHORA ya no está (distancia >= umbral)
      Serial.println("Auto parece haber pasado. Dando 1 segundo de gracia...");
      vTaskDelay(pdMS_TO_TICKS(1000)); // Espera 1s por seguridad
      break; // <-- Salida normal del bucle. El auto cruzó.

    } else {
      // ESTADO: El auto NO está debajo (dist >= umbral) 
      // y NUNCA se detectó (autoDetectado=false)
      // (Esperando a que el auto avance desde el lector hasta la pluma)
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // --- Chequeo de Timeout ---
    // Si han pasado más de 15 segundos Y NUNCA detectamos al auto pasar...
    if (!autoDetectado && (millis() - startTime > TIMEOUT_MS)) {
      Serial.println("¡Timeout! El auto no cruzó (o pasó muy rápido). Cerrando pluma.");
      break; // <-- Salida de emergencia del bucle.
    }
    // --- Fin Chequeo de Timeout ---
    
  } while (distancia < UMBRAL_DISTANCIA || !autoDetectado); // Condición de salida:
                                                           // (distancia >= umbral) Y (autoDetectado == true)
  
  // 3. Cerrar
  Serial.println("Camino libre. Cerrando pluma.");
  plumaServo.write(0);
  digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
  digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
}


// =========================================================
// --- FUNCIONES DE COMUNICACIÓN MQTT ---
// =========================================================

/**
 * Publica el evento de acceso (permitido/denegado) en MQTT.
 * Se llama desde la Tarea 2 cada vez que se procesa un evento de la cola.
 * Parámetro 'userIndex': El índice del usuario (o -1 denegado, -2 remoto).
 */
void publishAccessEvent(int userIndex) {
    if (!client.connected()) return; // No hacer nada si MQTT está desconectado
    
    // --- Obtener la hora actual ---
    char timeBuffer[20];
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Error al obtener la hora");
      strcpy(timeBuffer, "1970-01-01 00:00:00"); // Hora por defecto si falla NTP
    } else {
      strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    }

    // --- Definir los mensajes de status y usuario ---
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

    // --- Publicar en los tópicos de "salida" ---
    client.publish("estacionamiento/salida/usuario", userName);
    client.publish("estacionamiento/salida/tiempo", timeBuffer);
    client.publish("estacionamiento/salida/status", status);
    
    Serial.printf("--- Evento publicado [SALIDA]: Usuario: %s, Status: %s ---\n", userName, status);
}

/**
 * Función de reconexión MQTT.
 * Se llama cuando el cliente se desconecta.
 */
void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    
    // ClientID único para la SALIDA (evita colisiones con la ENTRADA)
    char clientId[50];
    sprintf(clientId, "ESP32_Estacionamiento_Salida-%ld", random(1000));
    
    // Intenta conectar
    if (client.connect(clientId, mqttUser, mqttPassword)) {
      Serial.println(" conectado!");
      
      // --- Suscripción a Tópicos ---
      // Se suscribe al tópico de comando específico para la "salida"
      client.subscribe("estacionamiento/salida/comando");
      Serial.println("Suscrito a 'estacionamiento/salida/comando'");
      
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
 * Se ejecuta CADA VEZ que llega un mensaje de un tópico al que estamos suscritos.
 * Parámetros:
 * - topic: Tópico del mensaje.
 * - message: Contenido del mensaje.
 * - length: Longitud del mensaje.
 */
void callback(char* topic, byte* message, unsigned int length) {
  // Convertir el mensaje (byte*) a un string 
  String stMessage;
  for (int i = 0; i < length; i++) {
    stMessage += (char)message[i];
  }
  Serial.printf("Mensaje recibido en [%s]: %s\n", topic, stMessage.c_str());

  // --- Lógica de Comandos ---
  // Revisa si el tópico es el de comando de "salida"
  if (String(topic) == "estacionamiento/salida/comando") {
    // Revisa si el mensaje es "abrir"
    if (stMessage == "abrir") {
      Serial.println("Comando de apertura remota recibido!");
      // Envía el comando especial (-2) a la cola para que la Tarea 2 lo procese
      int command = REMOTE_OPEN_COMMAND;
      xQueueSend(userIndexQueue, &command, portMAX_DELAY);
    }
  }
}

// =========================================================
// --- LOOP PRINCIPAL (VACÍO) ---
// =========================================================
/**
 * El loop principal de Arduino.
 * En un proyecto con FreeRTOS, este loop no se usa.
 * Las tareas se encargan de toda la ejecución.
 * Se llama a vTaskDelete(NULL) para borrar esta tarea (la del loop)
 * y liberar sus recursos.
 */
void loop() {
  vTaskDelete(NULL);
}