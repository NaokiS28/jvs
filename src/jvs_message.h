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
class JVS_Frame {
    public:
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
    bool isHost = false;
    bool send = false;
    bool escaped = false;

    JVS_Frame() {};

    JVS_Frame(uint8_t *_data, uint16_t _s, bool _isHost = false, bool _send = false) {
        uint16_t c = 0;
        uint16_t a = 0;

        isHost = _isHost;
        send = _send;

        sync = _data[c++];
        nodeID = _data[c++];
        numBytes = _data[c++];
        if(nodeID == 0x00){
            statusCode = _data[c++];
        }
        while(c < (_s - 1)){
            data[a++] = _data[c++];
        }
        sum = _data[c];
        EscapePacket();
    }

    JVS_Frame(uint8_t _i, uint8_t _n, uint8_t *_data, uint8_t _sc = 1, bool _isHost = false, bool _send = false){
        isHost = _isHost;
        send = _send;

        sync = 0xE0;
        nodeID = _i;
        numBytes = _n;  // 
        statusCode = _sc;
    
        if(!isHost){
            numBytes += 2;
        } else {
            numBytes += 1;
        }

        uint8_t * _d = _data;

        memcpy(data, _d, _n);
        CalculateSum(_isHost, _send);
        EscapePacket();
    }

    ~JVS_Frame(){};

    void CalculateSum(bool isHost, bool send){
        uint32_t _s = 0;
        _s += nodeID;
        _s += numBytes;
        if((!isHost && send) || (isHost && !send)){
            _s += statusCode;
        }

        for(int s = 0; s < numBytes-1; s++){
            _s += dataString[s];
        }
        _s = (_s % 256);
        sum =  _s;
    }

    bool CompareSum(bool isHost, bool send){
        uint32_t _s = 0;
        _s += nodeID;
        _s += numBytes;
        if((!isHost && send) || (isHost && !send)){
            _s += statusCode;
        }

        for(int s = 0; s < numBytes-1; s++){
            _s += dataString[s];
        }
        _s = (_s % 256);

        if(sum == _s){ 
            return true;
        } else {
            return false;
        }
    }

    void EscapePacket(){
        if(!escaped){
            uint8_t _temp[104];
            int b = 3;
            if(!isHost) b = 4;

            for(int m = 0; m < (numBytes + b); m++){
                uint8_t d = data[m];
                if ((d == 0xE0 || d == 0xD0) && m != 0){
                    _temp[m++] = 0xD0;
                    _temp[m] = (d - 1);
                } else {
                    _temp[m] = d;
                }
            }
            memcpy(data, _temp, 104);
        }
        escaped = true;
    }

    uint8_t* ToArray(){
        uint8_t *_arr = new uint8_t[this->numBytes + 3];
        if(_arr == nullptr) { return nullptr; }
        int c = 0;
        int size = numBytes;
        size -= (isHost ? 1 : 2);

        _arr[c++] = sync;
        _arr[c++] = nodeID;
        _arr[c++] = numBytes;

        if(!isHost){
            _arr[c++] = statusCode;
        }
        memcpy(_arr + c, data, size);
        c += size;
        _arr[c++] = sum;
        
        Serial.println(F("JVS_Msg: JVS_Frame.toArray(): "));
        Serial.print(F("Size: "));
        Serial.println(c);
        for(int a = 0; a < c; a++){
            Serial.println(_arr[a], HEX);
        }

        return _arr;
    }

    int CopyToArray(uint8_t* _ta, int _s){
        uint8_t *_arr = new uint8_t[numBytes + 3];
        if(_arr == nullptr) { return 0; }
        int c = 0;
        int a = 0;
        int b = 3;
        if(!isHost) b = 4;

        _arr[c++] = sync;
        _arr[c++] = nodeID;
        _arr[c++] = numBytes;

        if(!isHost){
            _arr[c++] = statusCode;
        }
        while(c < (numBytes - 1)){
            _arr[c++] = data[a++];
        }
        _arr[c] = sum;
        memcpy(_ta, _arr, numBytes + b);

        return numBytes + 3;
    }
};
#endif

typedef enum {
    endCode, switchInput, coinInput, analogInput, rotaryInput, keycodeInput,
    screenPosInput, miscInput, reserved1, reserved2, cardSlots, medalOutputs, 
    gpOutput, analogOutput, characterOutput, backupSupport
} featureTypes;

typedef enum {
    unknown, asciiNumeric, asciiAlpha, asciiKatakana, asciiKanji
} characterOutputType;

//
struct JVS_Info {
    char ident[100] = "JVS ArduinIO;github.com/NaokiS28/jvs;VER:0.1 Beta";
    char mainID[100] = "ArduinIO Host;github.com/NaokiS28/jvs;VER:0.1 Beta";
    uint8_t cmdRev = 11;    // Some JVS hosts (Triforce) will report ver 0.0 if it's higher than 1.1
    uint8_t jvsRev = 20;
    uint8_t comRev = 10;
    uint8_t totalFeatures = 0;
    featureTypes featureSupport[16] = {
        
    };
    uint8_t featureParameters[16][3] = {
        //{2, 12, 0},     // Is switchInput in generic construction
        //{2, 0, 0},      // 2 Coin slots
        //{6, 10, 0},     // 6 ADCs, 10-Bit resolution
        //{8, 0, 0}       // 8 GPOs (general purpose output)
    };
};

#endif