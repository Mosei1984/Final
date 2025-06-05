#pragma once
// Debug helpers: define ENABLE_DEBUG to enable Serial output
#include <Arduino.h>

#ifdef ENABLE_DEBUG
  #define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif
