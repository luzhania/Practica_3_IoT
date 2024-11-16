#include <Wire.h>
#include <WiFi.h>
#include "Utilities.h"
#include "config.h"
#include "WiFiConnection.h"
#include "PulseSensorPlayground.h"
#include "LCDDisplay.h"
#include "MQTTClient.h"
#include "ExerciseBand.h"

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