#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>

void parseKeyString(const String &keyStr, uint8_t *outArray, size_t expectedSize);

#endif // UTILITIES_H 