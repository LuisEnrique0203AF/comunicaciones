#include <Wire.h>
#include <BH1750.h>

BH1750 lightMeter;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Iniciando prueba de sensor GY-302 (BH1750)");

  Wire.begin();

  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("Sensor BH1750 encontrado. ¡Listo!");
  } else {
    Serial.println("¡Error! No se pudo encontrar el sensor BH1750.");
    while (1);
  }
}

void loop() {
  float lux = lightMeter.readLightLevel();
  Serial.print("Nivel de Luz: ");
  Serial.print(lux);
  Serial.println(" lx");
  delay(1000);
}
