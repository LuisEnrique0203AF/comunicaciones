// =========================================================
// --- INCLUDES (CON BME280 AÑADIDO) ---
// =========================================================
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <time.h>
#include "PubSubClient.h"
#include <Wire.h>                 // <-- LIBRERÍA NUEVA
#include <Adafruit_Sensor.h>      // <-- LIBRERÍA NUEVA
#include <Adafruit_BME280.h>    // <-- LIBRERÍA NUEVA

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

// --- OBJETOS DE SENSORES Y ACTUADORES ---
MFRC522 rfid(SS_PIN, RST_PIN);
Servo plumaServo;
Adafruit_BME280 bme; // Objeto para BME280 (I2C) <-- NUEVO
#define BME_ADDRESS 0x76 // Dirección I2C confirmada <-- NUEVO

// --- OBJETOS DE RED ---
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
void taskReadBME280(void *parameter); // <-- PROTOTIPO NUEVO
void mqttReconnect();
void callback(char* topic, byte* message, unsigned int length);
void publishAccessEvent(int userIndex);
long getHcsr04Distance();
void openBarrierWithSensorLogic();
void openBarrierRemote();

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
  
  // --- INICIALIZACIÓN DE PERIFÉRICOS ---
  SPI.begin();       // Para el RFID
  Wire.begin();      // Para el BME280 (Pines 21=SDA, 22=SCL) <-- NUEVO
  rfid.PCD_Init();
  
  // Iniciar BME280 <-- NUEVO
  if (!bme.begin(BME_ADDRESS)) {
    Serial.println("¡Error! No se pudo encontrar el sensor BME280.");
    // No detenemos el programa, solo avisamos
  } else {
    Serial.println("Sensor BME280 encontrado. ¡Listo!");
  }
  
  plumaServo.attach(SERVO_PIN);
  plumaServo.write(0);
  pinMode(LED_ACCESS_GRANTED_PIN, OUTPUT);
  pinMode(LED_ACCESS_DENIED_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
  digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
  
  userIndexQueue = xQueueCreate(5, sizeof(int));
  
  // --- INICIO DE TAREAS ---
  Serial.println(F("\nSistema de estacionamiento [SALIDA] listo."));
  
  xTaskCreate(taskReadRFID, "Read RFID Task", 4096, NULL, 1, NULL);
  xTaskCreate(taskControlActuators, "Control Actuators Task", 4096, NULL, 1, NULL);
  xTaskCreate(taskMqttManager, "MQTT Manager Task", 4096, NULL, 1, NULL);
  xTaskCreate(taskReadBME280, "BME280 Task", 4096, NULL, 1, NULL); // <-- ARRANQUE DE TAREA NUEVA
}

// =========================================================
// --- TAREA 1: Leer RFID (Con estabilización) ---
// =========================================================
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
      else if (getHcsr04Distance() >= UMBRAL_DISTANCIA) {
        vTaskDelay(pdMS_TO_TICKS(confirmationTime));
        if (getHcsr04Distance() >= UMBRAL_DISTANCIA) {
           Serial.println("Auto ya no está presente. Esperando nuevo auto...");
           isCarStablePresent = false;
        }
      }
      else {
         vTaskDelay(pdMS_TO_TICKS(50));
      }
    }
  }
}

// =========================================================
// --- TAREA 2: Controlar actuadores ---
// =========================================================
void taskControlActuators(void *parameter) {
  int receivedUserIndex;
  for (;;) {
    if (xQueueReceive(userIndexQueue, &receivedUserIndex, portMAX_DELAY) == pdPASS) {
      publishAccessEvent(receivedUserIndex);

      if (receivedUserIndex >= 0) {
        openBarrierWithSensorLogic();
      } else if (receivedUserIndex == REMOTE_OPEN_COMMAND) {
        openBarrierRemote();
      } else {
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
// --- TAREA 3: Gestionar MQTT ---
// =========================================================
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
// --- TAREA 4: LEER SENSOR BME280 Y PUBLICAR (NUEVA) ---
// =========================================================
void taskReadBME280(void *parameter) {
  // Espera 15 segundos la primera vez para que WiFi y MQTT se conecten
  vTaskDelay(pdMS_TO_TICKS(15000)); 
  
  for (;;) {
    // Leer los datos del sensor
    float temperatura = bme.readTemperature();
    float presion = bme.readPressure() / 100.0F;
    float humedad = bme.readHumidity();

    // Imprimir en serial (opcional)
    Serial.printf("[BME280] Temp: %.2f C, Pres: %.2f hPa, Hum: %.2f %%\n", temperatura, presion, humedad);

    // Publicar en MQTT (solo si está conectado)
    if (client.connected()) {
      // Creamos buffers para convertir float a string
      char tempStr[8];
      char presStr[8];
      char humStr[8];

      // dtostrf(variable, ancho_total, decimales, buffer_destino)
      dtostrf(temperatura, 4, 2, tempStr); 
      dtostrf(presion, 6, 2, presStr);
      dtostrf(humedad, 4, 2, humStr);

      // Publica en los tópicos de "salida"
      client.publish("estacionamiento/salida/temperatura", tempStr);
      client.publish("estacionamiento/salida/humedad", humStr);
      client.publish("estacionamiento/salida/presion", presStr);
    }

    // Espera 1 minuto (60,000 ms) para la siguiente lectura
    vTaskDelay(pdMS_TO_TICKS(60000)); 
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
 * Apertura para MODO REMOTO (Node-RED)
 */
void openBarrierRemote() {
  digitalWrite(LED_ACCESS_GRANTED_PIN, HIGH);
  digitalWrite(LED_ACCESS_DENIED_PIN, LOW);
  plumaServo.write(90);
  Serial.println("Pluma abierta (Remoto).");
  vTaskDelay(pdMS_TO_TICKS(5000));
  Serial.println("Cerrando pluma (Remoto).");
  plumaServo.write(0);
  digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
  digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
}


/**
 * Apertura para RFID (CON LÓGICA DE SENSOR)
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
  
  Serial.println("Camino libre. Cerrando pluma.");
  plumaServo.write(0);
  digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
  digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
}


// =========================================================
// --- FUNCIONES MQTT (MODIFICADAS PARA LA SALIDA) ---
// =========================================================

// Función de publicación MQTT
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
        status = "Permititdo";
        userName = tarjetasAutorizadas[userIndex].name;
    } else {
        status = "Denegado";
        userName = "Desconocido";
    }

    // Publica en los tópicos de "salida"
    client.publish("estacionamiento/salida/usuario", userName);
    client.publish("estacionamiento/salida/tiempo", timeBuffer);
    client.publish("estacionamiento/salida/status", status);
    
    Serial.printf("--- Evento publicado [SALIDA]: Usuario: %s, Status: %s ---\n", userName, status);
}

// Función de reconexión MQTT
void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    
    // ClientID único para la SALIDA
    char clientId[50];
    sprintf(clientId, "ESP32_Estacionamiento_Salida-%ld", random(1000));
    
    if (client.connect(clientId, mqttUser, mqttPassword)) {
      Serial.println(" conectado!");
      
      // Se suscribe al tópico de comando de "salida"
      client.subscribe("estacionamiento/salida/comando");
      Serial.println("Suscrito a 'estacionamiento/salida/comando'");
      
    } else {
      Serial.print(" falló, rc=");
      Serial.print(client.state());
      Serial.println(" -> Intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

// Función de Callback
void callback(char* topic, byte* message, unsigned int length) {
  String stMessage;
  for (int i = 0; i < length; i++) {
    stMessage += (char)message[i];
  }
  Serial.printf("Mensaje recibido en [%s]: %s\n", topic, stMessage.c_str());

  // Revisa el tópico de comando de "salida"
  if (String(topic) == "estacionamiento/salida/comando") {
    if (stMessage == "abrir") {
      Serial.println("Comando de apertura remota recibido!");
      int command = REMOTE_OPEN_COMMAND;
      xQueueSend(userIndexQueue, &command, portMAX_DELAY);
    }
  }
}

// =========================================================
// --- LOOP (AHORA VACÍO) ---
// =========================================================
void loop() {
  // El loop principal está vacío porque todo se maneja con FreeRTOS
  vTaskDelete(NULL);
}