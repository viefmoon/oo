#include <Arduino.h>  // Agregar esta línea para definir la clase String

// Función auxiliar para convertir un string tipo 
// "EE,F1,30,98,6A,11,4E,69,D0,DE,8A,DC,D6,8D,28,A6"
// en un array de 16 bytes
void parseKeyString(const String &keyStr, uint8_t *outArray, size_t expectedSize) {
    int index = 0;
    int start = 0;
    while (index < expectedSize && start < keyStr.length()) {
        int comma = keyStr.indexOf(',', start);
        String byteStr;
        if (comma == -1) {
            byteStr = keyStr.substring(start);
            start = keyStr.length();
        } else {
            byteStr = keyStr.substring(start, comma);
            start = comma + 1;
        }
        outArray[index] = (uint8_t)strtol(byteStr.c_str(), NULL, 16);
        index++;
    }
} 