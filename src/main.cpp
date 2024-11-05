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

PulseSensorPlayground pulseSensor;

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

class MQTTHandler {
private:
    WiFiClientSecure wiFiClient;
    PubSubClient client;
    StaticJsonDocument<JSON_OBJECT_SIZE(64)> outputDoc;
    char outputBuffer[128];
    const char *UPDATE_TOPIC = "$aws/things/exercise_band/shadow/update";
    const char *UPDATE_DELTA_TOPIC = "$aws/things/exercise_band/shadow/update/delta";
    const String THING_NAME = "thing2";

    unsigned int min_pulse_alert = 0;
    unsigned int max_pulse_alert = 0;

    void callback(char *topic, byte *payload, unsigned int length) {
        String message;
        for (int i = 0; i < length; i++) {
            message += String((char)payload[i]);
        }
        Serial.println("Message from topic " + String(topic) + ": " + message);
        
        StaticJsonDocument<200> inputDoc;
        DeserializationError error = deserializeJson(inputDoc, payload, length);
        if (!error) {
            if (String(topic) == UPDATE_DELTA_TOPIC) {
                if (inputDoc["state"]["devices"][THING_NAME]["pulse_requested"] == 1) {
                    unsigned int pulse = pulseSensor.getBeatsPerMinute();
                    updatePulseInShadow(pulse);
                    publishPulseRequestAttended();
                }
                if (inputDoc["state"]["devices"][THING_NAME]["min_pulse_alert"] > 0) {
                    min_pulse_alert = inputDoc["state"]["devices"][THING_NAME]["min_pulse_alert"];
                    publishMinPulseParameter();
                }
                if (inputDoc["state"]["devices"][THING_NAME]["max_pulse_alert"] > 0) {
                    max_pulse_alert = inputDoc["state"]["devices"][THING_NAME]["max_pulse_alert"];
                    publishMaxPulseParameter();
                }
            }
        } else {
            Serial.println("Error deserializando el mensaje JSON");
        }
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

    void publishPulseRequestAttended() {
        outputDoc.clear();
        outputDoc["state"]["desired"]["devices"][THING_NAME]["pulse_requested"] = 0;
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    void publishMinPulseParameter() {
        outputDoc.clear();
        outputDoc["state"]["reported"]["devices"][THING_NAME]["min_pulse_alert"] = min_pulse_alert;
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    void publishMaxPulseParameter() {
        outputDoc.clear();
        outputDoc["state"]["reported"]["devices"][THING_NAME]["max_pulse_alert"] = max_pulse_alert;
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

// Clase para procesar el estado del pulso
class PulseProcessor {
private:
    unsigned int currentState = 0;

public:
    unsigned int determinePulseState(unsigned int pulse) {
        if (pulse < 60) return 0;
        if (pulse <= 200) return 1;
        return 2;
    }

    void publishStateIfChanged(MQTTHandler &mqttHandler, unsigned int newState) {
        if (newState != currentState) {
            currentState = newState;
            mqttHandler.publishState(currentState);
            Serial.print("Publicado nuevo estado: ");
            Serial.println(currentState);
        }
    }
};

LCDDisplay lcd;
MQTTHandler mqttHandler(MQTT_BROKER, MQTT_PORT);
PulseProcessor pulseProcessor;
WiFiConnection wifi(WIFI_SSID, WIFI_PASS);

void setup() {
    Serial.begin(115200);
    lcd.init();
    
    pulseSensor.analogInput(35);
    pulseSensor.blinkOnPulse(2);
    pulseSensor.setThreshold(2300);
    if (pulseSensor.begin()) {
        Serial.println("PulseSensor initialized.");
    }

    wifi.connect();

    mqttHandler.setCertificates(AMAZON_ROOT_CA1, CERTIFICATE, PRIVATE_KEY);
    mqttHandler.connectMQTT();
}

void loop() {
    if (!mqttHandler.isConnected()) {
        mqttHandler.connectMQTT();
    }
    mqttHandler.loop();

    Utilities::nonBlockingDelay(200, []() {
        if (pulseSensor.sawStartOfBeat()) {
            unsigned int pulse = pulseSensor.getBeatsPerMinute();
            Serial.print(pulse);
            unsigned int newState = pulseProcessor.determinePulseState(pulse);
            pulseProcessor.publishStateIfChanged(mqttHandler, newState);
            lcd.printMessage(pulse, 0, 0);
        }
    });
}
