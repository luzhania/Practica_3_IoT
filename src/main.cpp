#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"

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

void displayPulseOnLCD(int pulse) {
  // Mostrar el pulso en la pantalla LCD
  lcd.clear();
  lcd.setCursor(0, 0); // Posicionar en la primera fila
  lcd.print("Pulso actual:");
  lcd.setCursor(0, 1); // Posicionar en la segunda fila
  lcd.print(pulse);
}

void updateShadowPulse(int pulse) {
  outputDoc["state"]["reported"]["devices"]["thing1"]["pulse"] = pulse; // Actualiza el pulso en el shadow
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
      int pulse_requested = inputDoc["state"]["devices"]["thing1"]["pulse_requested"].as<int>();
      if (pulse_requested == 1) {
        int currentPulse = inputDoc["state"]["reported"]["devices"]["thing1"]["pulse"].as<int>();
        displayPulseOnLCD(currentPulse);
        updateShadowPulse(currentPulse); // Actualiza el pulso en el shadow
      }
    }
  }
}

void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.print("Conectado a WiFi. Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  lcd.init(); // Inicializa el LCD
  lcd.backlight(); // Enciende la luz de fondo
  setupWiFi();

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
}
