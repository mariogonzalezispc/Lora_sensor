#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <SoftwareSerial.h>

// === Definición de pines según tu PDF ===
#define LED_VERDE 14 // GPIO15
#define LED_ROJO 15  // GPIO14
#define DE_RE 16     // GPIO16 (control RS485)
#define DI 12        // GPIO12 -> TX hacia MAX485
#define RO 13        // GPIO13 -> RX desde MAX485

Adafruit_AHTX0 aht;
SoftwareSerial rs485(RO, DI); // RX, TX
bool sensorOK = false;

void setup()
{
  Serial.begin(9600);
  delay(100);
  WiFi.mode(WIFI_OFF);    // Desactivar WiFi completamente
  WiFi.forceSleepBegin(); // Pone el módulo Wi-Fi en modo de bajo consumo total
  delay(1);
  pinMode(LED_ROJO, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);
  digitalWrite(LED_ROJO, LOW);
  digitalWrite(LED_VERDE, LOW);
  // Configuración MAX485
  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW); // Modo recepción
  rs485.begin(9600);
  // Inicializar I2C y sensor
  Wire.begin();
  if (aht.begin())
  {
    Serial.println("Sensor AHT10 inicializado correctamente.");
    sensorOK = true;
  }
  else
  {
    Serial.println("ERROR: No se pudo inicializar el sensor AHT10.");
  }
  Serial.println("Esperando mensajes RS485...");
}

void loop()
{
  if (rs485.available()) // Si hay datos disponibles
  {
    String mensaje = rs485.readStringUntil('\n');
    mensaje.trim();
    Serial.print("Mensaje recibido: ");
    Serial.println(mensaje);
    if (mensaje == "ID_01") // Si el mensaje coincide con el ID_XX, procesa el pedido
    {
      digitalWrite(LED_ROJO, HIGH);
      if (!aht.begin()) // Verifica si el sensor esta presente y responde
      {
        sensorOK = false;
        Serial.println("ERROR: Sensor AHT10 no responde.");
      }
      else
      {
        sensorOK = true;
      }

      if (sensorOK) // Si el sensor está OK, lee y envía respuesta
      {
        digitalWrite(LED_ROJO, LOW);
        sensors_event_t hum, temp;
        aht.getEvent(&hum, &temp);

        // Crear mensaje de respuesta
        char respuesta[64];
        snprintf(respuesta, sizeof(respuesta),
                 "$,ID_01,T:%.2f,H:%.2f%%,&",
                 temp.temperature, hum.relative_humidity);
        // Mostrar por Serial lo leído
        // Serial.print("Lectura AHT10 -> Temp: ");
        // Serial.print(temp.temperature, 2);
        // Serial.print(" °C | Humedad: ");
        // Serial.print(hum.relative_humidity, 2);
        // Serial.println(" %");
        Serial.print("Enviando respuesta RS485: "); //
        Serial.println(respuesta);
        delay(100);
        // Enviar por RS485
        digitalWrite(DE_RE, HIGH); // Modo transmisión
        digitalWrite(LED_VERDE, HIGH);
        delay(20);
        rs485.print(respuesta);
        rs485.flush();
        delay(20);
        digitalWrite(DE_RE, LOW); // Vuelve a recepción
        digitalWrite(LED_VERDE, LOW);
      }
      else // Si el sensor no está OK, no envía respuesta
      {
        Serial.println("Sensor AHT10 no disponible, no se envía respuesta.");
      }
      sensorOK = false; // reseteo el flag para forzar nueva inicialización en el próximo ciclo
    }
  }
}
