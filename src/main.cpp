#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h> // <-- LIBRERÍA CORRECTA PARA ESP32

// --- PINES ---
#define SS_PIN    5
#define RST_PIN   4
#define LED_ACCESS_GRANTED_PIN  13
#define LED_ACCESS_DENIED_PIN   12
#define SERVO_PIN               2

// --- OBJETOS ---
MFRC522 rfid(SS_PIN, RST_PIN);
Servo plumaServo;

// --- UIDs AUTORIZADOS ---
const int numTarjetas = 4;
byte tarjetasAutorizadas[numTarjetas][4] = {
  {0x83, 0xE8, 0xED, 0x2C}, // Azul con marca
  {0x93, 0xB8, 0xB7, 0x14}, // Azul sin marca
  {0xC3, 0x28, 0x1D, 0x2A}, // Tarjeta blanca
  {0xF3, 0xB9, 0x7C, 0x29}  // Tarjeta blanca con marca
};

// =========================================================
// === PROTOTIPOS DE FUNCIONES (AQUÍ ESTÁ LA CORRECCIÓN) ===
// =========================================================
void abrirPluma();
bool compareUID(byte *uidLeido, byte *uidAutorizado);
// =========================================================


void setup() { 
  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();

  plumaServo.attach(SERVO_PIN);
  plumaServo.write(0);
  
  pinMode(LED_ACCESS_GRANTED_PIN, OUTPUT);
  pinMode(LED_ACCESS_DENIED_PIN, OUTPUT);
  digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
  digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);

  Serial.println(F("Sistema de estacionamiento listo. Acerque su tarjeta."));
}
 
void loop() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    delay(50);
    return;
  }

  bool tarjetaValida = false;
  for (int i = 0; i < numTarjetas; i++) {
    if (compareUID(rfid.uid.uidByte, tarjetasAutorizadas[i])) {
      tarjetaValida = true;
      break;
    }
  }

  if (tarjetaValida) {
    Serial.println(F("ACCESO PERMITIDO"));
    abrirPluma(); 
  } else {
    Serial.println(F("ACCESO DENEGADO - Tarjeta incorrecta"));
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_ACCESS_DENIED_PIN, LOW);
      delay(150);
      digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
      delay(150);
    }
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

/**
 * Función que simula la apertura y cierre de la pluma.
 */
void abrirPluma() {
  digitalWrite(LED_ACCESS_GRANTED_PIN, HIGH);
  digitalWrite(LED_ACCESS_DENIED_PIN, LOW);

  Serial.println(F("...Pluma abriendo..."));
  plumaServo.write(90);

  delay(5000);

  Serial.println(F("...Pluma cerrando..."));
  plumaServo.write(0);

  digitalWrite(LED_ACCESS_GRANTED_PIN, LOW);
  digitalWrite(LED_ACCESS_DENIED_PIN, HIGH);
}

/**
 * Función para comparar UIDs.
 */
bool compareUID(byte *uidLeido, byte *uidAutorizado) {
  for (byte i = 0; i < 4; i++) {
    if (uidLeido[i] != uidAutorizado[i]) {
      return false;
    }
  }
  return true;
}