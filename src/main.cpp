#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"
#include "Utilities.h"
#include "WiFiConnection.h"

// Configuración del LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Constantes para MQTT
const char *UPDATE_TOPIC = "$aws/things/exercise_band/shadow/update";
const char *UPDATE_DELTA_TOPIC = "$aws/things/exercise_band/shadow/update/delta";

// Variables para los umbrales y mensajes
unsigned int min_pulse_alert;
unsigned int max_pulse_alert;
const char *message;
const String THING_NAME = "thing1";

// Objetos para WiFi, MQTT y JSON
WiFiClientSecure wiFiClient;
PubSubClient client(wiFiClient);
StaticJsonDocument<JSON_OBJECT_SIZE(512)> inputDoc;
StaticJsonDocument<JSON_OBJECT_SIZE(64)> outputDoc;
char outputBuffer[128];

// Variables para el control de tiempo
unsigned long previousMillis = 0;
const long interval = 1000; // 1 segundo

// Función para leer el sensor de pulso
int readPulseSensor()
{
  return random(60, 100); // Simula un valor aleatorio entre 60 y 100
}

int determinePulseState(int pulse)
{
  if (pulse < 60)
    return 0;
  if (pulse <= 200)
    return 1;
  return 2;
}

// Función para imprimir mensajes en el LCD
void printLCDMessage(const char *message, unsigned int row, unsigned int col)
{
  lcd.clear();
  lcd.setCursor(col, row);
  lcd.print(message);
}

// Función para actualizar el shadow con el valor de pulso
void updatePulseInShadow(int pulse)
{
  outputDoc.clear();
  outputDoc["state"]["reported"]["devices"][THING_NAME]["heart_rate"] = pulse;
  serializeJson(outputDoc, outputBuffer);
  client.publish(UPDATE_TOPIC, outputBuffer);
}

// Función para manejar la solicitud de actualización del pulso
void processPulseRequest(const JsonDocument &doc)
{
  int pulse_requested = doc["state"]["devices"][THING_NAME]["pulse_requested"];
  if (pulse_requested == 1)
  {
    int currentPulse = readPulseSensor();
    updatePulseInShadow(currentPulse);
  }
}

void publishMinPulseParameter()
{
  outputDoc.clear();
  outputDoc["state"]["reported"]["devices"][THING_NAME]["min_pulse_alert"] = min_pulse_alert;
  serializeJson(outputDoc, outputBuffer);
  client.publish(UPDATE_TOPIC, outputBuffer);
}

void publishMaxPulseParameter()
{
  outputDoc.clear();
  outputDoc["state"]["reported"]["devices"][THING_NAME]["max_pulse_alert"] = max_pulse_alert;
  serializeJson(outputDoc, outputBuffer);
  client.publish(UPDATE_TOPIC, outputBuffer);
}

void processMinPulseParameter(const JsonDocument &doc)
{
  min_pulse_alert = doc["state"]["devices"][THING_NAME]["min_pulse_alert"];
  publishMinPulseParameter();
}

void processMaxPulseParameter(const JsonDocument &doc)
{
  max_pulse_alert = doc["state"]["devices"][THING_NAME]["max_pulse_alert"];
  publishMaxPulseParameter();
}

// Callback para manejar mensajes recibidos
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.println("Llamada a callback");
  String message;
  for (int i = 0; i < length; i++)
    message += String((char)payload[i]);
  Serial.println("Message from topic " + String(topic) + ":" + message);
  DeserializationError err = deserializeJson(inputDoc, payload);

  if (!err)
  {
    if (String(topic) == UPDATE_DELTA_TOPIC)
    {
      if (inputDoc["state"]["devices"][THING_NAME]["pulse_requested"])
      {
        processPulseRequest(inputDoc);
      }
      if (inputDoc["state"]["devices"][THING_NAME]["min_pulse_alert"])
      {
        processMinPulseParameter(inputDoc);
      }
      if (inputDoc["state"]["devices"][THING_NAME]["max_pulse_alert"])
      {
        processMaxPulseParameter(inputDoc);
      }
    }
  }
}

// Función para reconectar con MQTT
void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect(CLIENT_ID))
    {
      Serial.println("conectado");
      if (client.subscribe(UPDATE_DELTA_TOPIC))
        Serial.println("Suscrito a " + String(UPDATE_DELTA_TOPIC));
    }
    else
    {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

// Función para mostrar el pulso en el LCD
void displayPulseOnLCD()
{
  int pulse = readPulseSensor();
  String message = "Pulso: " + String(pulse);
  printLCDMessage(message.c_str(), 0, 0);
}


WiFiConnection wifi(WIFI_SSID, WIFI_PASS);

void setup()
{
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  wifi.connect();

  wiFiClient.setCACert(AMAZON_ROOT_CA1);
  wiFiClient.setCertificate(CERTIFICATE);
  wiFiClient.setPrivateKey(PRIVATE_KEY);

  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setCallback(callback);
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  Utilities::nonBlockingDelay(1000, []()
                              { displayPulseOnLCD(); });
}
