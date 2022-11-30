#ifndef __RFID_H__
#define __RFID_H__

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include "state.h"

static const enum State RFID_STATE_MAP[] = {RFID_QUEEN, RFID_QUEEN, RFID_QUEEN, RFID_UNDER, RFID_UNDER, RFID_UNDER, RFID_COME, RFID_COME, RFID_COME, RFID_COME};

/**
 * Initialization function, performs self test and returns result, true is OK, false is not
 */
bool rfidInit(MFRC522 rfid);

/**
 * Helper routine to dump a byte array as hex values to Serial. 
 */
void printHex(byte *buffer, byte bufferSize);

/**
 * Helper routine to dump a byte array as dec values to Serial.
 */
void printDec(byte *buffer, byte bufferSize);

/**
 * Check for RFID card presence and read its UID, 
 * return true if the UID read is new, NUID are written to nuidPICC[],
 * return false if no card present or UID detected is not new
 */
bool rfidReadNuid(MFRC522 rfid, byte *nuidPICC, byte nuidSize);

void activateRfidReception(MFRC522 *rfid);

void clearInt(MFRC522 rfid);

uint8_t random(uint8_t min, uint8_t max);

#endif // __RFID_H__