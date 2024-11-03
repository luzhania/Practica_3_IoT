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
LiquidCrystal_I2C lcd(0x27, 20, 4); // Cambia 0x27 si tu dirección I2C es diferente

// Reemplaza con los detalles de tu shadow
const char* UPDATE_TOPIC = "$aws/things/exercise_band/shadow/update";              // publicar
const char* UPDATE_DELTA_TOPIC = "$aws/things/exercise_band/shadow/update/delta";  // suscribirse

WiFiClientSecure wiFiClient;
PubSubClient client(wiFiClient);

StaticJsonDocument<JSON_OBJECT_SIZE(64)> inputDoc;
StaticJsonDocument<JSON_OBJECT_SIZE(4)> outputDoc;
char outputBuffer[128];

// Variables para la medición periódica
unsigned long previousMillis = 0;
const long interval = 1000; // 1 segundo

void printLCDMessage(const char* message, unsigned int row, unsigned int col) {
  lcd.clear();
  lcd.setCursor(col, row); // Posicionar en la primera fila
  lcd.print(message);
}

int readPulseSensor() {
  // int pulse = analogRead(A0); // Asume que el XD-58C está conectado a A0
  int pulse = random(60, 100); // Simula un valor aleatorio entre 60 y 100
  return pulse;
}

string getMessage(unsigned int pulse) {
  string message;
  if (pulse < 60) {
    message = "Pulso demasiado bajo:";
  } else if (pulse >= 60 && pulse < 100) {
    message = "Pulso actual:";
  } else {
    message = "Pulso demasiado alto:";
  }
  return message + to_string(pulse);
}

void displayPulseOnLCD() {
  int pulse = readPulseSensor();
  string message = getMessage(pulse);
  printLCDMessage(message.c_str(), 0, 0);
}

void updateShadowPulse(int pulse) {
  outputDoc["state"]["reported"]["devices"][THING_NAME]["pulse"] = pulse; // Actualiza el pulso en el shadow
  serializeJson(outputDoc, outputBuffer);
  client.publish(UPDATE_TOPIC, outputBuffer);
}

// Callback para manejar mensajes recibidos
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) message += String((char) payload[i]);
  Serial.println("Mensaje desde el tema " + String(topic) + ": " + message);
  
  DeserializationError err = deserializeJson(inputDoc, payload);
  if (!err) {
    if (String(topic) == UPDATE_DELTA_TOPIC) {
      int pulse_requested = inputDoc["state"]["devices"][THING_NAME]["pulse_requested"].as<int>();
      if (pulse_requested == 1) {
        int currentPulse = readPulseSensor();
        updateShadowPulse(currentPulse); // Actualiza el pulso en el shadow
      }
    }
  }
}

WiFiConnection wifi(WIFI_SSID, WIFI_PASS);
const string THING_NAME = "thing1";

void setup() {
  Serial.begin(115200);
  lcd.init(); // Inicializa el LCD
  lcd.backlight(); // Enciende la luz de fondo
  wifi.connect();

  wiFiClient.setCACert(AMAZON_ROOT_CA1);
  wiFiClient.setCertificate(CERTIFICATE);
  wiFiClient.setPrivateKey(PRIVATE_KEY);

  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setCallback(callback);
}

void reconnect() {
  // Bucle hasta que estemos reconectados
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect(CLIENT_ID)) {
      Serial.println("conectado");
      client.subscribe(UPDATE_DELTA_TOPIC);
      Serial.println("Suscrito a " + String(UPDATE_DELTA_TOPIC));
      delay(100);
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  Utilities::nonBlockingDelay(1000, []()
                              { displayPulseOnLCD(); });
}
