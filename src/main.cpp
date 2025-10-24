#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>

// Definición de pines
#define LED1 15
#define LED2 14
#define DE_RE 16
#define DI 12
#define RO 13

// Pines I2C por defecto ESP8266: SDA=GPIO4(D2), SCL=GPIO5(D1)

// Variables
Adafruit_AHTX0 aht;
unsigned long previousMillis = 0;
const long interval = 100;
bool ledState = false;

void setup()
{
  // Inicializar Serial
  Serial.begin(9600);
  delay(100);
  Serial.println("\n\n=== ESP8266 Iniciando ===");

  // Configurar pines de LEDs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  Serial.println("LEDs configurados en GPIO15 y GPIO14");

  // Configurar pines del MAX485
  pinMode(DE_RE, OUTPUT);
  pinMode(DI, OUTPUT);
  pinMode(RO, INPUT);
  digitalWrite(DE_RE, LOW); // Modo recepción por defecto
  Serial.println("MAX485 configurado - DE/RE: GPIO16, DI: GPIO12, RO: GPIO13");

  // Inicializar I2C
  Wire.begin(); // SDA=GPIO4, SCL=GPIO5
  Serial.println("I2C inicializado");

  // Inicializar sensor AHT10
  if (aht.begin())
  {
    Serial.println("Sensor AHT10 inicializado correctamente");
  }
  else
  {
    Serial.println("ERROR: No se pudo inicializar el sensor AHT10");
    Serial.println("Verifica las conexiones I2C");
  }

  Serial.println("=== Setup completado ===\n");
}

void loop()
{
  ledState = !ledState;
  digitalWrite(LED1, ledState);
  digitalWrite(LED2, !ledState);
  delay(100);
}