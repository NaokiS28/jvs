#ifndef JVS_M_H
#define JVS_M_H

#include <Arduino.h>

#ifdef JVS_FULL_FRAME
struct JVS_Frame {
    uint8_t sync = 0xE0;
    uint8_t nodeID = 0;
    uint8_t numBytes = 0;          // Includes all data bytes and sync
    uint8_t statusCode = 1;
    union{
        uint8_t data [254] = {0};
        uint16_t data16 [127];
        char dataString [104];
    } ;
    uint8_t sum = 0;                // Checksum of ID, numbytes, data bytes
    uint8_t cmdCount = 1;           // Number of commands counted. Only counts standard frames
};
#else
// Half size frames
struct JVS_Frame {
    uint8_t sync = 0xE0;
    uint8_t nodeID = 0;
    uint8_t numBytes = 0;          // Includes all data bytes and sync
    uint8_t statusCode = 1;
    union{
        uint8_t data [104] = {0};
        uint16_t data16 [52];
        char dataString [104];
    } ;
    uint8_t sum = 0;                // Checksum of ID, numbytes, data bytes
    uint8_t cmdCount = 1;           // Number of commands counted. Only counts standard frames
};
#endif

typedef enum {
    endCode, switchInput, coinInput, analogInput, rotaryInput, keycodeInput,
    screenPosInput, miscInput, reserved1, reserved2, cardSlots, medalOutputs, gpOutput,
    analogOutput, characterOuput, backupSupport
} featureTypes;

typedef enum {
    unknown, asciiNumeric, asciiAlpha, asciiKatakana, asciiKanji
} characterOutputType;

//
struct JVS_Info {
    char ident[100] = "JVS IO Library;github.com/NaokiS28/jvs;VER:0.1 Beta";
    char mainID[100] = "Generic JVS Game";
    uint8_t cmdRev = 11;    // Some JVS hosts (Triforce) will report ver 0.0 if it's higher than 1.1
    uint8_t jvsRev = 30;
    uint8_t comRev = 10;
    uint8_t totalFeatures = 4;
    featureTypes featureSupport[16] = {
        switchInput, coinInput, analogInput, gpOutput
    };
    uint8_t featureParameters[16][3] = {
        {2, 12, 0},     // Is switchInput in generic construction
        {2, 0, 0},      // 2 Coin slots
        {6, 10, 0},     // 6 ADCs, 10-Bit resolution
        {8, 0, 0}       // 8 GPOs (general purpose output)
    };
};

#endif