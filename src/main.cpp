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

    void printMessage(unsigned int pulse, unsigned int row, unsigned int col,  String message)
    {
        lcd.clear();
        lcd.setCursor(col, row);
        lcd.print(message.c_str());
        lcd.setCursor(0, 3);
        String pulseMessage = String(pulse) + " lpm";
        lcd.print(pulseMessage);
    }
};

class MQTTClient{
    protected:
    WiFiClientSecure wiFiClient;
    PubSubClient client;
    
void callback(char *topic, byte *payload, unsigned int length)
    {
        String message;
        for (unsigned int i = 0; i < length; i++)
        {
            message += (char)payload[i];
        }
        Serial.println("Message from topic " + String(topic) + ": " + message);
        StaticJsonDocument<200> inputDoc;
        DeserializationError error = deserializeJson(inputDoc, payload, length);
        if (!error)
        {
            onMessageReceived(topic, inputDoc);
        }
        else
        {
            Serial.println("Error deserializando el mensaje JSON");
        }
    }

    virtual void onMessageReceived(const String &topic, StaticJsonDocument<200> inputDoc) = 0;
    virtual void subscribeTopics() = 0;
public:
    MQTTClient(const char *broker, int port) : client(wiFiClient)
    {
        client.setServer(broker, port);
        client.setCallback([this](char *topic, byte *payload, unsigned int length)
                           { this->callback(topic, payload, length); });
    }

    void setCertificates(const char *rootCA, const char *cert, const char *key)
    {
        wiFiClient.setCACert(rootCA);
        wiFiClient.setCertificate(cert);
        wiFiClient.setPrivateKey(key);
    }

    void loop()
    {
        client.loop();
    }

    void connectMQTT()
    {
        while (!client.connected())
        {
            Serial.print("Trying MQTT conection...");
            if (client.connect("CLIENT_ID"))
            {
                Serial.println("Conected");
                subscribeTopics();
            }
            else
            {
                Serial.print("error, rc=");
                Serial.print(client.state());
                Serial.println(" trying again in 5 seconds");
                delay(5000);
            }
        }
    }
};

class ExerciseBand : public MQTTClient
{
    protected:
    const char *UPDATE_TOPIC = "$aws/things/exerciseband/shadow/update";
    const char *UPDATE_DELTA_TOPIC = "$aws/things/exerciseband/shadow/update/delta";

    unsigned int pulse = 0;
    unsigned int currentState = 0;
    unsigned int minPulseAlert = 60;
    unsigned int maxPulseAlert = 200;
    String message = "";

    StaticJsonDocument<JSON_OBJECT_SIZE(64)> outputDoc;
    char outputBuffer[128];

    public:

    ExerciseBand(const char *broker, int port) : MQTTClient(broker, port) {}

    void onMessageReceived(const String &topic, StaticJsonDocument<200> inputDoc) override
    {
        Serial.println("Message received from: " + topic);
            if (String(topic) == UPDATE_DELTA_TOPIC)
            {
                if (inputDoc["state"]["message"])
                {
                    message = inputDoc["state"]["message"].as<String>();
                    reportMessage();
                }
                if (inputDoc["state"]["pulse_requested"] == 1)
                {
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
            }
    }

    void subscribeTopics() override
    {
        client.subscribe(UPDATE_DELTA_TOPIC);
        Serial.println("Suscribed to topic: " + String(UPDATE_DELTA_TOPIC));
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

    void reportMessage()
    {
        outputDoc.clear();
        outputDoc["state"]["reported"]["message"] = message.c_str();
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    unsigned int getNewState()
    {
        if (pulse < minPulseAlert)
            return 0;
        if (pulse < maxPulseAlert)
            return 1;
        return 2;
    }

    void reportStateIfChanged(unsigned int pulse)
    {
        this->pulse = pulse;
        if (getNewState() != currentState)
        {
            currentState = getNewState();
            outputDoc.clear();
            outputDoc["state"]["reported"]["heart_rate_state"] = currentState;
            serializeJson(outputDoc, outputBuffer);
            client.publish(UPDATE_TOPIC, outputBuffer);
            Serial.print("Publicado nuevo estado: ");
            Serial.println(currentState);
        }
    }

    bool isConnected()
    {
        return client.connected();
    }

    String getMessage()
    {
        return message;
    }    
};


LCDDisplay lcd;
WiFiConnection wifi(WIFI_SSID, WIFI_PASS);
PulseSensorPlayground pulseSensor;
ExerciseBand exerciseBand(MQTT_BROKER, MQTT_PORT);

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

    exerciseBand.setCertificates(AMAZON_ROOT_CA1, CERTIFICATE, PRIVATE_KEY);
    exerciseBand.connectMQTT();
}

void loop()
{
    if (!exerciseBand.isConnected())
    {
        exerciseBand.connectMQTT();
    }
    exerciseBand.loop();

    Utilities::nonBlockingDelay(200, []()
                                {
        if (pulseSensor.sawStartOfBeat()) {
            unsigned int pulse = pulseSensor.getBeatsPerMinute();
            exerciseBand.reportStateIfChanged(pulse);
            lcd.printMessage(pulse, 0, 0, exerciseBand.getMessage());
        } });
}