#include <Arduino.h>
#include <functional>
#include <iostream>
#pragma once

using namespace std;

class Utilities
{
public:

  static void nonBlockingDelay(unsigned long milliseconds, std::function<void()> callback)
  {
    static unsigned long lastMeasurement = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastMeasurement >= milliseconds)
    {
      callback();
      lastMeasurement = currentMillis;
    }
  }
};