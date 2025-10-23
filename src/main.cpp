/*
 * Proyecto de Estacionamiento con 3 Sensores VL53L0X y FreeRTOS
 * - Usa FreeRTOS para crear una tarea independiente por sensor.
 * - Implementa lógica de ocupación:
 *   - Si < umbral_cm por 2 segundos -> OCUPADO
 *   - Si ya no está < umbral_cm -> VACANTE (inmediato)
 * - Usa un Mutex para proteger el bus I2C.
 */

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// --- Pines XSHUT para cada sensor ---
#define XSHUT_1 25
#define XSHUT_2 26
#define XSHUT_3 27

// --- Parámetros de lógica ---
#define DISTANCIA_UMBRAL_CM_DEF 6     // Umbral por defecto (en cm)
#define TIEMPO_OCUPADO_MS 2000        // Tiempo para confirmar ocupación
#define POLLING_RATE_MS 200           // Frecuencia de muestreo

// --- Objetos de sensor ---
VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

// --- RTOS ---
SemaphoreHandle_t i2cMutex; // Mutex para proteger el bus I2C

// --- Estados del sensor ---
enum State { VACANTE, PENDIENTE, OCUPADO };

// --- Estructura de datos para cada sensor ---
struct SensorData {
  VL53L0X* sensor;
  const char* name;
  State state;
  unsigned long timer;
  uint8_t umbral_cm; // Umbral específico (cm)
};

// --- Datos por sensor ---
SensorData dataS1 = {&sensor1, "Sensor 1", VACANTE, 0, DISTANCIA_UMBRAL_CM_DEF};
SensorData dataS2 = {&sensor2, "Sensor 2", VACANTE, 0, DISTANCIA_UMBRAL_CM_DEF};
SensorData dataS3 = {&sensor3, "Sensor 3", VACANTE, 0, 9}; // ← Umbral especial de 9 cm

// --- Función de tarea para cada sensor ---
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
          }
        } else {
          data->state = VACANTE;
        }
        break;

      case OCUPADO:
        if (!objetoPresente) {
          data->state = VACANTE;
          Serial.printf("--- %s: CAJÓN DESOCUPADO ---\n", data->name);
        }
        break;
    }

    vTaskDelay(POLLING_RATE_MS / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  delay(100);

  // --- Crear el Mutex ---
  i2cMutex = xSemaphoreCreateMutex();
  if (i2cMutex == NULL) {
    Serial.println("Error: No se pudo crear el Mutex I2C");
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

  // ===== Sensor 1 (opcional) =====
   Serial.println("Activando sensor #1 (dirección 0x30)...");
   digitalWrite(XSHUT_1, HIGH);
   delay(10);
   if (!sensor1.init(true)) {
     Serial.println("Error: no se detecta el sensor #1");
     while (1);
   }
   sensor1.setAddress(0x30);
 sensor1.startContinuous();

  // ===== Sensor 2 =====
  Serial.println("Activando sensor #2 (dirección 0x31)...");
  digitalWrite(XSHUT_2, HIGH);
  delay(10);
  if (!sensor2.init(true)) {
    Serial.println("Error: no se detecta el sensor #2");
    while (1);
  }
  sensor2.setAddress(0x31);
  sensor2.startContinuous();

  // ===== Sensor 3 =====
  Serial.println("Activando sensor #3 (dirección 0x32)...");
  digitalWrite(XSHUT_3, HIGH);
  delay(10);
  if (!sensor3.init(true)) {
    Serial.println("Error: no se detecta el sensor #3");
    while (1);
  }
  sensor3.setAddress(0x32);
  sensor3.startContinuous();

  Serial.println("¡Sensores inicializados!");
  Serial.println("--- Creando Tareas de RTOS ---");

  // --- Crear tareas ---
  xTaskCreate(sensorTask, "Sensor 1 Task", 2048, (void*)&dataS1, 1, NULL); // ← Sensor 1 comentado
  xTaskCreate(sensorTask, "Sensor 2 Task", 2048, (void*)&dataS2, 1, NULL);
  xTaskCreate(sensorTask, "Sensor 3 Task", 2048, (void*)&dataS3, 1, NULL);

  Serial.println("¡Tareas creadas! Sistema operativo iniciado.");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
