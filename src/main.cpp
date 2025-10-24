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
unsigned long previousMillisLED = 0;
unsigned long previousMillisSensor = 0;
const long intervalLED = 100;
const long intervalSensor = 2000; // Leer sensor cada 2 segundos
bool ledState = false;
bool sensorOK = false;

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

  // Inicializar sensor AHT10 en dirección 0x38
  if (aht.begin())
  {
    Serial.println("Sensor AHT10 inicializado correctamente en 0x38");
    sensorOK = true;
  }
  else
  {
    Serial.println("ERROR: No se pudo inicializar el sensor AHT10");
    Serial.println("Verifica las conexiones I2C (SDA=GPIO4, SCL=GPIO5)");
    sensorOK = false;
  }

  Serial.println("=== Setup completado ===\n");
}

void loop()
{
  unsigned long currentMillis = millis();

  // Parpadeo de LEDs cada 100ms (no bloqueante)
  if (currentMillis - previousMillisLED >= intervalLED)
  {
    previousMillisLED = currentMillis;
    ledState = !ledState;
    digitalWrite(LED1, ledState);
    digitalWrite(LED2, !ledState); // LED2 en oposición
  }

  // Lectura del sensor cada 2 segundos
  if (currentMillis - previousMillisSensor >= intervalSensor)
  {
    previousMillisSensor = currentMillis;

    if (sensorOK)
    {
      // Crear objeto para almacenar los datos del sensor
      sensors_event_t humidity, temp;

      // MÉTODO PRINCIPAL: aht.getEvent(&humidity, &temp)
      // Lee temperatura y humedad del sensor y las guarda en los objetos pasados por referencia
      // Retorna true si la lectura fue exitosa, false si falló
      aht.getEvent(&humidity, &temp);

      // Acceder a los valores leídos
      float temperatura = temp.temperature;       // Temperatura en °C
      float humedad = humidity.relative_humidity; // Humedad relativa en %

      // Publicar por serial
      Serial.println("=================================");
      Serial.print("Temperatura: ");
      Serial.print(temperatura);
      Serial.println(" °C");

      Serial.print("Humedad: ");
      Serial.print(humedad);
      Serial.println(" %");
      Serial.println("=================================\n");

      /*
       * MÉTODOS DISPONIBLES DE LA LIBRERÍA Adafruit_AHTX0:
       *
       * 1. aht.begin(uint8_t i2c_addr = AHTX0_I2CADDR_DEFAULT, TwoWire *wire = &Wire)
       *    - Inicializa el sensor AHT10/AHT20
       *    - Por defecto usa dirección 0x38
       *    - Retorna: true si inicialización exitosa, false si falla
       *    - Ejemplo: aht.begin() o aht.begin(0x38)
       *
       * 2. aht.getEvent(&humidity, &temp)
       *    - Lee temperatura y humedad del sensor
       *    - Requiere dos objetos sensors_event_t pasados por referencia
       *    - Retorna: true si lectura exitosa, false si falla
       *    - Ejemplo: sensors_event_t hum, tmp; aht.getEvent(&hum, &tmp);
       *
       * 3. aht.getTemperatureSensor()
       *    - Retorna un puntero al objeto sensor de temperatura
       *    - Útil para usar con sistemas de sensores unificados de Adafruit
       *    - Retorna: Adafruit_Sensor*
       *
       * 4. aht.getHumiditySensor()
       *    - Retorna un puntero al objeto sensor de humedad
       *    - Útil para usar con sistemas de sensores unificados de Adafruit
       *    - Retorna: Adafruit_Sensor*
       *
       * 5. aht.reset()
       *    - Realiza un reset por software del sensor
       *    - Útil si el sensor se cuelga o no responde correctamente
       *    - No retorna valor
       *    - Ejemplo: aht.reset();
       *
       * ESTRUCTURA sensors_event_t (datos leídos):
       * - temp.temperature: Temperatura en grados Celsius (float)
       * - humidity.relative_humidity: Humedad relativa en porcentaje (float)
       * - temp.timestamp: Marca de tiempo de la lectura (uint32_t)
       * - humidity.timestamp: Marca de tiempo de la lectura (uint32_t)
       *
       * EJEMPLO DE USO DE reset():
       * if (!aht.getEvent(&humidity, &temp)) {
       *   Serial.println("Error en lectura, reseteando sensor...");
       *   aht.reset();
       *   delay(100);
       * }
       *
       * NOTAS IMPORTANTES:
       * - El AHT10 tiene precisión de ±0.3°C para temperatura
       * - Precisión de ±2% para humedad relativa
       * - Tiempo de respuesta típico: 5 segundos para humedad
       * - Rango de medición temperatura: -40°C a +85°C
       * - Rango de medición humedad: 0% a 100% RH
       * - Dirección I2C fija: 0x38 (no se puede cambiar)
       */
    }
    else
    {
      Serial.println("Sensor no disponible - No se pueden leer datos");
    }
  }
}