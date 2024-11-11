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

    void printMessage(unsigned int pulse, unsigned int row, unsigned int col,  string message)
    {
        lcd.clear();
        lcd.setCursor(col, row);
        lcd.print(pulse);
        lcd.setCursor(0, 1);
        lcd.print(message.c_str());
    }
};

// Clase para procesar el estado del pulso
class MQTTHandler
{
private:
    const String THING_NAME = "exerciseband";
    WiFiClientSecure wiFiClient;
    PubSubClient client;
    StaticJsonDocument<JSON_OBJECT_SIZE(64)> outputDoc;
    char outputBuffer[128];

    String updateTopic = "$aws/things/" + THING_NAME + "/shadow/update";
    String updateDeltaTopic = "$aws/things/" + THING_NAME + "/shadow/update/delta";

    String messageTopic = "aws/things/message";
    const char *MESSAGE_TOPIC = messageTopic.c_str();


    const char *UPDATE_TOPIC = updateTopic.c_str();
    const char *UPDATE_DELTA_TOPIC = updateDeltaTopic.c_str();

    unsigned int currentState = 0;
    unsigned int minPulseAlert = 60;
    unsigned int maxPulseAlert = 200;

    string message = "";
    ///////////

    void callback(char *topic, byte *payload, unsigned int length)
    {
        String message;
        for (int i = 0; i < length; i++)
        {
            message += String((char)payload[i]);
        }
        Serial.println("Message from topic " + String(topic) + ": " + message);

        StaticJsonDocument<200> inputDoc;
        DeserializationError error = deserializeJson(inputDoc, payload, length);
        if (!error)
        {
            if (String(topic) == UPDATE_DELTA_TOPIC)
            {
                if (inputDoc["state"]["pulse_requested"] == 1)
                {
                    unsigned int pulse = pulseSensor.getBeatsPerMinute();
                    publishPulseRequestAttended();
                    updatePulseInShadow(pulse);
                }
                if (inputDoc["state"]["min_pulse_alert"] > 0)
                {
                    minPulseAlert = inputDoc["state"]["min_pulse_alert"];
                    reportMinPulseParameter();
                }
                if (inputDoc["state"]["max_pulse_alert"] > 0)
                {
                    maxPulseAlert = inputDoc["state"]["max_pulse_alert"];
                    reportMaxPulseParameter();
                }
                if (inputDoc["state"]["message"])
                {
                    string message = inputDoc["state"]["message"];
                    Serial.println(message.c_str());
                    this->message = message;
                    reportMessage(message);
                }
            }
            // else if (String(topic) == MESSAGE_TOPIC)
            // {
            //     if (inputDoc["state"]["message"])
            //     {
            //         string message = inputDoc["state"]["message"];
            //         Serial.println(message.c_str());
            //         this->message = message;
            //     }
            // }
        }
        else
        {
            Serial.println("Error deserializando el mensaje JSON");
        }
    }

public:
    MQTTHandler(const char *broker, int port) : client(wiFiClient)
    {
        client.setServer(broker, port);
        client.setCallback([this](char *topic, byte *payload, unsigned int length)
                           { this->callback(topic, payload, length); });
    }

    unsigned int determinePulseState(unsigned int pulse)
    {
        if (pulse < minPulseAlert)
            return 0;
        if (pulse < maxPulseAlert)
            return 1;
        return 2;
    }

    void publishStateIfChanged(unsigned int newState)
    {
        if (newState != currentState)
        {
            currentState = newState;
            outputDoc.clear();
            outputDoc["state"]["reported"]["heart_rate_state"] = currentState;
            serializeJson(outputDoc, outputBuffer);
            client.publish(UPDATE_TOPIC, outputBuffer);
            Serial.print("Publicado nuevo estado: ");
            Serial.println(currentState);
        }
    }

    void setCertificates(const char *rootCA, const char *cert, const char *key)
    {
        wiFiClient.setCACert(rootCA);
        wiFiClient.setCertificate(cert);
        wiFiClient.setPrivateKey(key);
    }

    bool isConnected()
    {
        return client.connected();
    }

    void connectMQTT()
    {
        while (!client.connected())
        {
            Serial.print("Intentando conexión MQTT...");
            if (client.connect("CLIENT_ID"))
            {
                Serial.println("conectado");
                client.subscribe(UPDATE_DELTA_TOPIC);
                client.subscribe(MESSAGE_TOPIC);
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

    void loop()
    {
        client.loop();
    }

    void updatePulseInShadow(int pulse)
    {
        outputDoc.clear();
        outputDoc["state"]["reported"]["heart_rate"] = pulse;
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    void publishPulseRequestAttended()
    {
        outputDoc.clear();
        outputDoc["state"]["desired"]["pulse_requested"] = 0;
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    void reportMinPulseParameter()
    {
        outputDoc.clear();
        outputDoc["state"]["reported"]["min_pulse_alert"] = minPulseAlert;
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    void reportMaxPulseParameter()
    {
        outputDoc.clear();
        outputDoc["state"]["reported"]["max_pulse_alert"] = maxPulseAlert;
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    void reportMessage(string message)
    {
        outputDoc.clear();
        outputDoc["state"]["reported"]["message"] = message;
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    string getMessage()
    {
        return message;
    }
};

LCDDisplay lcd;
MQTTHandler mqttHandler(MQTT_BROKER, MQTT_PORT);
WiFiConnection wifi(WIFI_SSID, WIFI_PASS);

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

    mqttHandler.setCertificates(AMAZON_ROOT_CA1, CERTIFICATE, PRIVATE_KEY);
    mqttHandler.connectMQTT();
}

void loop()
{
    if (!mqttHandler.isConnected())
    {
        mqttHandler.connectMQTT();
    }
    mqttHandler.loop();

    Utilities::nonBlockingDelay(200, []()
                                {
        if (pulseSensor.sawStartOfBeat()) {
            unsigned int pulse = pulseSensor.getBeatsPerMinute();
            Serial.print(pulse);
            unsigned int newState = mqttHandler.determinePulseState(pulse);
            mqttHandler.publishStateIfChanged(newState);
            lcd.printMessage(pulse, 0, 0, mqttHandler.getMessage());
        } });
}
