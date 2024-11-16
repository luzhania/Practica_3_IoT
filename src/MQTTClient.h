#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

#pragma once

class MQTTClient
{
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
            Serial.println("Error deserializing JSON message");
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

    void loop()
    {
        client.loop();
    }

    bool isConnected()
    {
        return client.connected();
    }
};
