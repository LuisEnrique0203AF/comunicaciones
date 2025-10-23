#include <Wire.h>
#include <VL53L0X.h>

// Pines XSHUT para cada sensor
#define XSHUT_1 25
#define XSHUT_2 26
#define XSHUT_3 27

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL
  delay(100);

  // Configurar pines XSHUT
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
  //Serial.println("Activando sensor #1 (dirección 0x30)...");
 // digitalWrite(XSHUT_1, HIGH);
 // delay(10);
 // if (!sensor1.init(true)) {
    ///Serial.println("Error: no se detecta el sensor #1");
   // while (1);
 // }
  //sensor1.setAddress(0x30);
//  sensor1.startContinuous();
 
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

  Serial.println("¡Sensores inicializados correctamente!");
}

void loop() {
  uint16_t dist1 = sensor1.readRangeContinuousMillimeters();
  uint16_t dist2 = sensor2.readRangeContinuousMillimeters();
  uint16_t dist3 = sensor3.readRangeContinuousMillimeters();

  // Convertir a centímetros (mm / 10.0)
  float cm1 = dist1 / 10.0;
  float cm2 = dist2 / 10.0;
  float cm3 = dist3 / 10.0;

  // --- Mostrar distancias con filtro ---
  Serial.print("S1: ");
  if (dist1 < 8000)
    Serial.printf("%.1f cm\t", cm1);
  else
    Serial.print("Fuera de rango\t");

  Serial.print("S2: ");
  if (dist2 < 8000)
    Serial.printf("%.1f cm\t", cm2);
  else
    Serial.print("Fuera de rango\t");

  Serial.print("S3: ");
  if (dist3 < 8000)
    Serial.printf("%.1f cm", cm3);
  else
    Serial.print("Fuera de rango");

  Serial.println();

  delay(200);
}
