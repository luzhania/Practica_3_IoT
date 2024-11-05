#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "Utilities.h"
#include "config.h"
#include "WiFiConnection.h"
#include "PulseSensorPlayground.h"

class LCDDisplay {
private:
    LiquidCrystal_I2C lcd;

public:
    LCDDisplay() : lcd(0x27, 20, 4) {}

    void init() {
        lcd.init();
        lcd.backlight();
    }

    void printMessage(unsigned int pulse, unsigned int row, unsigned int col) {
        lcd.clear();
        lcd.setCursor(col, row);
        lcd.print(pulse);
    }
};

class HeartRateMonitor {
private:
    const int pulsePin = 35;
    const int ledPin = 2;
    const int threshold = 2500;
    PulseSensorPlayground pulseSensor;

public:
    HeartRateMonitor() {
        pulseSensor.analogInput(pulsePin);
        pulseSensor.blinkOnPulse(ledPin);
        pulseSensor.setThreshold(threshold);
    }

    void begin() {
        if (pulseSensor.begin()) {
            Serial.println("PulseSensor initialized.");
        }
    }

    int readPulseSensor() {
        if (pulseSensor.sawStartOfBeat()) {
            return pulseSensor.getBeatsPerMinute();
        }
        return -1;
    }
};

class MQTTHandler {
private:
    WiFiClientSecure wiFiClient;
    PubSubClient client;
    StaticJsonDocument<JSON_OBJECT_SIZE(64)> outputDoc;
    char outputBuffer[128];
    const char *UPDATE_TOPIC = "$aws/things/exercise_band/shadow/update";
    const char *UPDATE_DELTA_TOPIC = "$aws/things/exercise_band/shadow/update/delta";
    const String THING_NAME = "thing2";

    void callback(char *topic, byte *payload, unsigned int length) {
        String message;
        for (int i = 0; i < length; i++) {
            message += String((char)payload[i]);
        }
        Serial.println("Message from topic " + String(topic) + ":" + message);
    }

public:
    MQTTHandler(const char *broker, int port) : client(wiFiClient) {
        client.setServer(broker, port);
        client.setCallback([this](char *topic, byte *payload, unsigned int length) { this->callback(topic, payload, length); });
    }

    void setCertificates(const char *rootCA, const char *cert, const char *key) {
        wiFiClient.setCACert(rootCA);
        wiFiClient.setCertificate(cert);
        wiFiClient.setPrivateKey(key);
    }

    bool isConnected() {
        return client.connected();
    }

    void connectMQTT() {
        while (!client.connected()) {
            Serial.print("Intentando conexión MQTT...");
            if (client.connect("CLIENT_ID")) {
                Serial.println("conectado");
                client.subscribe(UPDATE_DELTA_TOPIC);
            } else {
                Serial.print("falló, rc=");
                Serial.print(client.state());
                Serial.println(" intentando de nuevo en 5 segundos");
                delay(5000);
            }
        }
    }

    void loop() {
        client.loop();
    }

    void updatePulseInShadow(int pulse) {
        outputDoc.clear();
        outputDoc["state"]["reported"]["devices"][THING_NAME]["heart_rate"] = pulse;
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    void publishState(int currentState) {
        outputDoc.clear();
        outputDoc["state"]["reported"]["devices"][THING_NAME]["heart_rate_state"] = currentState;
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }
};

class PulseProcessor {
private:
    int currentState = -1;

public:
    int determinePulseState(int pulse) {
        if (pulse < 60) return 0;
        if (pulse <= 200) return 1;
        return 2;
    }

    void publishStateIfChanged(MQTTHandler &mqttHandler, int newState) {
        if (newState != currentState) {
            currentState = newState;
            mqttHandler.publishState(currentState);
            Serial.print("Publicado nuevo estado: ");
            Serial.println(currentState);
        }
    }
};

LCDDisplay lcd;
HeartRateMonitor heartRateMonitor;
MQTTHandler mqttHandler(MQTT_BROKER, MQTT_PORT);
PulseProcessor pulseProcessor;
WiFiConnection wifi(WIFI_SSID, WIFI_PASS);

void setup() {
    Serial.begin(115200);
    lcd.init();
    wifi.connect();
    heartRateMonitor.begin();

    mqttHandler.setCertificates(AMAZON_ROOT_CA1, CERTIFICATE, PRIVATE_KEY);
    mqttHandler.connectMQTT();
}

void loop() {
    if (!mqttHandler.isConnected()) {
        mqttHandler.connectMQTT();
    }
    mqttHandler.loop();

    Utilities::nonBlockingDelay(200, []() {
        int pulse = heartRateMonitor.readPulseSensor();
        if (pulse != -1) {
            int newState = pulseProcessor.determinePulseState(pulse);
            pulseProcessor.publishStateIfChanged(mqttHandler, newState);
            lcd.printMessage(pulse, 0, 0);
        }
    });
}
