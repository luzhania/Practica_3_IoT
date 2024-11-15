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

class LCDDisplay
{
private:
    LiquidCrystal_I2C lcd;

public:
    LCDDisplay() : lcd(0x27, 20, 4) {}

    void init()
    {
        lcd.init();
        lcd.backlight();
    }

    void printMessage(unsigned int pulse, unsigned int row, unsigned int col, string message)
    {
        lcd.clear();
        lcd.setCursor(col, row);
        lcd.print(pulse);
        lcd.setCursor(0, 1);
        lcd.print(message.c_str());
    }
};


class MQTTClientWrapper
{
private:
    WiFiClientSecure wiFiClient;
    PubSubClient client;
    const char *broker;
    int port;

public:
    MQTTClientWrapper(const char *broker, int port) : client(wiFiClient), broker(broker), port(port)
    {
        client.setServer(broker, port);
    }

    void setCallback(MQTT_CALLBACK_SIGNATURE)
    {
        client.setCallback(callback);
    }

    bool isConnected()
    {
        return client.connected();
    }

    void connect(const char *clientId, const char *updateDeltaTopic)
    {
        while (!client.connected())
        {
            Serial.print("Intentando conexión MQTT...");
            if (client.connect(clientId))
            {
                Serial.println("conectado");
                client.subscribe(updateDeltaTopic);
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

    void publish(const char *topic, const char *message)
    {
        client.publish(topic, message);
    }

    void loop()
    {
        client.loop();
    }

    PubSubClient &getClient()
    {
        return client;
    }

    void setCertificates(const char *rootCA, const char *cert, const char *key)
    {
        wiFiClient.setCACert(rootCA);
        wiFiClient.setCertificate(cert);
        wiFiClient.setPrivateKey(key);
    }
};

class PulseManager
{
private:
    unsigned int currentState = 0;
    unsigned int minPulseAlert;
    unsigned int maxPulseAlert;
    string message;

public:
    PulseManager(unsigned int minAlert, unsigned int maxAlert) : minPulseAlert(minAlert), maxPulseAlert(maxAlert) {}

    unsigned int determinePulseState(unsigned int pulse)
    {
        if (pulse < minPulseAlert) return 0;
        if (pulse < maxPulseAlert) return 1;
        return 2;
    }

    void setMinPulseAlert(unsigned int minAlert)
    {
        minPulseAlert = minAlert;
    }

    unsigned int getMinPulseAlert() const
    {
        return minPulseAlert;
    }

    unsigned int getMaxPulseAlert() const
    {
        return maxPulseAlert;
    }

    void setMaxPulseAlert(unsigned int maxAlert)
    {
        maxPulseAlert = maxAlert;
    }

    unsigned int getCurrentState() const
    {
        return currentState;
    }

    void updateCurrentState(unsigned int newState)
    {
        currentState = newState;
    }

    void setMessage(const string &newMessage)
    {
        message = newMessage;
    }

    const string &getMessage() const
    {
        return message;
    }
};

class ExerciseBandMQTTHandler
{
private:
    const char *THING_NAME = "exerciseband";
    const char *UPDATE_TOPIC = "$aws/things/exerciseband/shadow/update";////////////////////string
    const char *UPDATE_DELTA_TOPIC = "$aws/things/exerciseband/shadow/update/delta";

    StaticJsonDocument<JSON_OBJECT_SIZE(64)> outputDoc;
    char outputBuffer[128];
    
    MQTTClientWrapper mqttClient;
    PulseManager pulseManager;

    void processRequest(const StaticJsonDocument<200> &inputDoc)
    {
        if (inputDoc["state"]["pulse_requested"] == 1)
        {
            unsigned int pulse = pulseSensor.getBeatsPerMinute();
            publishPulseRequestAttended();
            updatePulseInShadow(pulse);
        }
        if (inputDoc["state"]["min_pulse_alert"] > 0)
        {
            pulseManager.setMinPulseAlert(inputDoc["state"]["min_pulse_alert"]);
            reportMinPulseParameter();
        }
        if (inputDoc["state"]["max_pulse_alert"] > 0)
        {
            pulseManager.setMaxPulseAlert(inputDoc["state"]["max_pulse_alert"]);
            reportMaxPulseParameter();
        }
        if (inputDoc["state"]["message"])
        {
            string message = inputDoc["state"]["message"].as<string>();
            pulseManager.setMessage(message);
            reportMessage();
        }
    }

    void callback(char *topic, byte *payload, unsigned int length)
    {
        StaticJsonDocument<200> inputDoc;
        DeserializationError error = deserializeJson(inputDoc, payload, length);
        if (!error && String(topic) == UPDATE_DELTA_TOPIC)
        {
            processRequest(inputDoc);
        }
        else
        {
            Serial.println("Error deserializando el mensaje JSON");
        }
    }

public:
    ExerciseBandMQTTHandler(const char *broker, int port)
        : mqttClient(broker, port), pulseManager(60, 200) 
    {
        mqttClient.setCertificates(AMAZON_ROOT_CA1, CERTIFICATE, PRIVATE_KEY);
        mqttClient.setCallback([this](char *topic, byte *payload, unsigned int length)
                               { this->callback(topic, payload, length); });

    }

    void connect()
    {
        mqttClient.connect("CLIENT_ID", UPDATE_DELTA_TOPIC);
    }

    void updatePulseInShadow(unsigned int pulse)
    {
        outputDoc.clear();
        outputDoc["state"]["reported"]["heart_rate"] = pulse;
        serializeJson(outputDoc, outputBuffer);
        mqttClient.publish(UPDATE_TOPIC, outputBuffer);
    }

    void publishPulseRequestAttended()
    {
        outputDoc.clear();
        outputDoc["state"]["desired"]["pulse_requested"] = 0;
        serializeJson(outputDoc, outputBuffer);
        mqttClient.publish(UPDATE_TOPIC, outputBuffer);
    }

    void reportMinPulseParameter()
    {
        outputDoc.clear();
        outputDoc["state"]["reported"]["min_pulse_alert"] = pulseManager.getMinPulseAlert();
        serializeJson(outputDoc, outputBuffer);
        mqttClient.publish(UPDATE_TOPIC, outputBuffer);
    }

    void reportMaxPulseParameter()
    {
        outputDoc.clear();
        outputDoc["state"]["reported"]["max_pulse_alert"] = pulseManager.getMaxPulseAlert();
        serializeJson(outputDoc, outputBuffer);
        mqttClient.publish(UPDATE_TOPIC, outputBuffer);
    }

    void reportMessage()
    {
        outputDoc.clear();
        outputDoc["state"]["reported"]["message"] = pulseManager.getMessage().c_str();
        serializeJson(outputDoc, outputBuffer);
        mqttClient.publish(UPDATE_TOPIC, outputBuffer);
    }

    void loop()
    {
        mqttClient.loop();
    }

    MQTTClientWrapper getMqttClient()
    {
        return mqttClient;
    }

    PulseManager getPulseManager()
    {
        return pulseManager;
    }

    void publishStateIfChanged(unsigned int newState)
    {
        if (newState != pulseManager.getCurrentState())
        {
           pulseManager.updateCurrentState(newState);
            outputDoc.clear();
            outputDoc["state"]["reported"]["heart_rate_state"] = pulseManager.getCurrentState();
            serializeJson(outputDoc, outputBuffer);
            mqttClient.publish(UPDATE_TOPIC, outputBuffer);
            Serial.print("Publicado nuevo estado: ");
            Serial.println(pulseManager.getCurrentState());
        }
    }
};


LCDDisplay lcd;
WiFiConnection wifi(WIFI_SSID, WIFI_PASS);
ExerciseBandMQTTHandler exerciseBand(MQTT_BROKER, MQTT_PORT);

void setup()
{
    Serial.begin(115200);
    lcd.init();

    pulseSensor.analogInput(34);
    pulseSensor.blinkOnPulse(2);
    pulseSensor.setThreshold(2700);
    if (pulseSensor.begin())
    {
        Serial.println("PulseSensor initialized.");
    }

    wifi.connect();

    exerciseBand.connect();
}

void loop()
{
    if (!exerciseBand.getMqttClient().isConnected())
    {
        exerciseBand.connect();
    }
    exerciseBand.loop();
    // Utilities::nonBlockingDelay(200, []()
    //                             {
    Serial.print(analogRead(34));
    if (pulseSensor.sawStartOfBeat())
    {
        unsigned int pulse = pulseSensor.getBeatsPerMinute();
        Serial.print("bpm: ");
        Serial.println(pulse);
        //////////arreglar
        unsigned int newState = exerciseBand.getPulseManager().determinePulseState(pulse);
        exerciseBand.publishStateIfChanged(newState);
        /////////////
        lcd.printMessage(pulse, 0, 0, exerciseBand.getPulseManager().getMessage());
    }
    delay(50);
    // });
}
