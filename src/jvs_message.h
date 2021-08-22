#ifndef JVS_M_H
#define JVS_M_H

#include <Arduino.h>

struct JVS_Frame {
    uint8_t _sync = 0xE0;
    uint8_t _nodeID;
    uint8_t _numBytes;          // Includes all data bytes and sync
    union {
        uint32_t data32 [2]    ; // Caution: subject to endianness
        uint16_t data16 [4]    ; // Caution: subject to endianness
        uint8_t data8   [8]    ;
        uint8_t data    [2]    ; // Most messages will use 2 bytes
        uint8_t dataString [104] = {""} ;
    } ;
    uint8_t _sum;               // Checksum of ID, numbytes, data bytes
};

typedef enum {
    _endCode, _switchInput, _coinInput, _analogInput, _keycodeInput,
    _screenPosInput, _miscInput, _cardSlots, _medalOutputs, _gpOutput,
    _analogOutput, _characterOuput, _backupSupport
} featureTypes;

typedef enum {
    _unknown, _asciiNumeric, _asciiAlpha, _asciiKatakana, _asciiKanji
} characterOutputType;

// Generic constructor for JVS infomation.
// This  will also serve as a store for data.

struct JVS_Info {
    char _ident[102] = {"JVS I/O"};
    char _mainID[102] = {"Generic JVS Game"};
    uint8_t _cmdRev = 13;
    uint8_t _jvsRev = 30;
    uint8_t _comRev = 10;
    uint8_t _numOfFeatures = 16;    // Most amount of features supported
    
    featureTypes _featureSupport[16] = {
        _switchInput, _coinInput, _analogInput, _gpOutput
    };
    uint8_t _featureParameters[16][3] = {
        {2, 12, 0},     // Is _switchInput in generic construction
        {2, 0, 0},      // 2 Coin slots
        {6, 10, 0},     // 6 ADCs, 10-Bit resolution
        {8, 0, 0}       // 8 GPOs (general purpose output)
    };
};

#endif