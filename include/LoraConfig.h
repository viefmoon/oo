#ifndef _RADIOLIB_EX_LORAWAN_CONFIG_H
#define _RADIOLIB_EX_LORAWAN_CONFIG_H

#include <RadioLib.h>

// how often to send an uplink - consider legal & FUP constraints - see notes
const uint32_t uplinkIntervalSeconds = 20UL;    // minutes x seconds

// #define RADIOLIB_LORAWAN_JOIN_EUI  0x0000000000000000
// #ifndef RADIOLIB_LORAWAN_DEV_EUI   
// #define RADIOLIB_LORAWAN_DEV_EUI   0x4a0b13d21648b66c
// #endif
// #ifndef RADIOLIB_LORAWAN_APP_KEY   // Replace with your App Key 
// #define RADIOLIB_LORAWAN_APP_KEY   0x9a, 0x6f, 0x20, 0x59, 0x2e, 0xf9, 0xde, 0x40, 0x94, 0xad, 0xa9, 0x54, 0x2d, 0x2f, 0x66, 0x7b
// #endif
// //7ec326d11919fb7aec532d954742d87a
// #ifndef RADIOLIB_LORAWAN_NWK_KEY   // Put your Nwk Key here
// #define RADIOLIB_LORAWAN_NWK_KEY   0x7e, 0xc3, 0x26, 0xd1, 0x19, 0x19, 0xfb, 0x7a, 0xec, 0x53, 0x2d, 0x95, 0x47, 0x42, 0xd8, 0x7a
// #endif
// //de9d5d3a7ed5b60be234a24f806e67ed

#ifndef RADIOLIB_LORAWAN_DEV_ADDR   
#define RADIOLIB_LORAWAN_DEV_ADDR   0x00bfe104
#endif
#ifndef RADIOLIB_LORAWAN_FNWKSINT_KEY   
#define RADIOLIB_LORAWAN_FNWKSINT_KEY   0x63, 0x67, 0x7a, 0x0e, 0xc1, 0x28, 0x7b, 0x82, 0x92, 0xc5, 0x0b, 0xa1, 0x3d, 0xd6, 0xd2, 0x25
#endif
#ifndef RADIOLIB_LORAWAN_SNWKSINT_KEY   
#define RADIOLIB_LORAWAN_SNWKSINT_KEY   0xdd, 0x14, 0x59, 0xab, 0x63, 0x6e, 0x50, 0x0d, 0x8a, 0x75, 0x77, 0xa8, 0x75, 0x28, 0xc9, 0x01
#endif
#ifndef RADIOLIB_LORAWAN_NWKSENC_KEY   
#define RADIOLIB_LORAWAN_NWKSENC_KEY   0x64, 0x13, 0xe3, 0x99, 0x91, 0xf0, 0x87, 0x56, 0x99, 0xc9, 0xbd, 0x8c, 0x72, 0x07, 0x8f, 0x9e
#endif
#ifndef RADIOLIB_LORAWAN_APPS_KEY   
#define RADIOLIB_LORAWAN_APPS_KEY   0xee, 0xf1, 0x30, 0x98, 0x6a, 0x11, 0x4e, 0x69, 0xd0, 0xde, 0x8a, 0xdc, 0xd6, 0x8d, 0x28, 0xa6
#endif

const LoRaWANBand_t Region = US915;
const uint8_t subBand = 2;  // For US915, change this to 2, otherwise leave on 0

// uint64_t joinEUI =   RADIOLIB_LORAWAN_JOIN_EUI;
// uint64_t devEUI  =   RADIOLIB_LORAWAN_DEV_EUI;
// uint8_t appKey[] = { RADIOLIB_LORAWAN_APP_KEY };
// uint8_t nwkKey[] = { RADIOLIB_LORAWAN_NWK_KEY };

uint32_t devAddr =        RADIOLIB_LORAWAN_DEV_ADDR;
uint8_t fNwkSIntKey[] = { RADIOLIB_LORAWAN_FNWKSINT_KEY };
uint8_t sNwkSIntKey[] = { RADIOLIB_LORAWAN_SNWKSINT_KEY };
uint8_t nwkSEncKey[] =  { RADIOLIB_LORAWAN_NWKSENC_KEY };
uint8_t appSKey[] =     { RADIOLIB_LORAWAN_APPS_KEY };

// result code to text ...
String stateDecode(const int16_t result) {
  switch (result) {
  case RADIOLIB_ERR_NONE:
    return "ERR_NONE";
  case RADIOLIB_ERR_CHIP_NOT_FOUND:
    return "ERR_CHIP_NOT_FOUND";
  case RADIOLIB_ERR_PACKET_TOO_LONG:
    return "ERR_PACKET_TOO_LONG";
  case RADIOLIB_ERR_RX_TIMEOUT:
    return "ERR_RX_TIMEOUT";
  case RADIOLIB_ERR_CRC_MISMATCH:
    return "ERR_CRC_MISMATCH";
  case RADIOLIB_ERR_INVALID_BANDWIDTH:
    return "ERR_INVALID_BANDWIDTH";
  case RADIOLIB_ERR_INVALID_SPREADING_FACTOR:
    return "ERR_INVALID_SPREADING_FACTOR";
  case RADIOLIB_ERR_INVALID_CODING_RATE:
    return "ERR_INVALID_CODING_RATE";
  case RADIOLIB_ERR_INVALID_FREQUENCY:
    return "ERR_INVALID_FREQUENCY";
  case RADIOLIB_ERR_INVALID_OUTPUT_POWER:
    return "ERR_INVALID_OUTPUT_POWER";
  case RADIOLIB_ERR_NETWORK_NOT_JOINED:
	  return "RADIOLIB_ERR_NETWORK_NOT_JOINED";

  case RADIOLIB_ERR_DOWNLINK_MALFORMED:
    return "RADIOLIB_ERR_DOWNLINK_MALFORMED";
  case RADIOLIB_ERR_INVALID_REVISION:
    return "RADIOLIB_ERR_INVALID_REVISION";
  case RADIOLIB_ERR_INVALID_PORT:
    return "RADIOLIB_ERR_INVALID_PORT";
  case RADIOLIB_ERR_NO_RX_WINDOW:
    return "RADIOLIB_ERR_NO_RX_WINDOW";
  case RADIOLIB_ERR_INVALID_CID:
    return "RADIOLIB_ERR_INVALID_CID";
  case RADIOLIB_ERR_UPLINK_UNAVAILABLE:
    return "RADIOLIB_ERR_UPLINK_UNAVAILABLE";
  case RADIOLIB_ERR_COMMAND_QUEUE_FULL:
    return "RADIOLIB_ERR_COMMAND_QUEUE_FULL";
  case RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND:
    return "RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND";
  case RADIOLIB_ERR_JOIN_NONCE_INVALID:
    return "RADIOLIB_ERR_JOIN_NONCE_INVALID";
  case RADIOLIB_ERR_N_FCNT_DOWN_INVALID:
    return "RADIOLIB_ERR_N_FCNT_DOWN_INVALID";
  case RADIOLIB_ERR_A_FCNT_DOWN_INVALID:
    return "RADIOLIB_ERR_A_FCNT_DOWN_INVALID";
  case RADIOLIB_ERR_DWELL_TIME_EXCEEDED:
    return "RADIOLIB_ERR_DWELL_TIME_EXCEEDED";
  case RADIOLIB_ERR_CHECKSUM_MISMATCH:
    return "RADIOLIB_ERR_CHECKSUM_MISMATCH";
  case RADIOLIB_LORAWAN_NO_DOWNLINK:
    return "RADIOLIB_LORAWAN_NO_DOWNLINK";
  case RADIOLIB_LORAWAN_SESSION_RESTORED:
    return "RADIOLIB_LORAWAN_SESSION_RESTORED";
  case RADIOLIB_LORAWAN_NEW_SESSION:
    return "RADIOLIB_LORAWAN_NEW_SESSION";
  case RADIOLIB_LORAWAN_NONCES_DISCARDED:
    return "RADIOLIB_LORAWAN_NONCES_DISCARDED";
  case RADIOLIB_LORAWAN_SESSION_DISCARDED:
    return "RADIOLIB_LORAWAN_SESSION_DISCARDED";
  }
  return "See TypeDef.h";
}

// helper function to display any issues
void debug(bool isFail, const __FlashStringHelper* message, int state, bool Freeze) {
  if (isFail) {
    Serial.print(message);
    Serial.print(" - ");
    Serial.print(stateDecode(state));
    Serial.print(" (");
    Serial.print(state);
    Serial.println(")");
    while (Freeze);
  }
}


// helper function to display a byte array
void arrayDump(uint8_t *buffer, uint16_t len) {
  for(uint16_t c = 0; c < len; c++) {
    char b = buffer[c];
    if(b < 0x10) { Serial.print('0'); }
    Serial.print(b, HEX);
  }
  Serial.println();
}

#endif
