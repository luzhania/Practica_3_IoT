#include "MQTTClient.h"

#pragma once

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
                updatePulseInShadow();
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

    void updatePulseInShadow()
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

    String getMessage()
    {
        return message;
    }
};