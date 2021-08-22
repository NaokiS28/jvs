# jvs
### This is still WIP and may not work correctly!
This is a WIP library for communicating over the JVS protocol in Arduino compatible boards. This library supports both Master mode and Node/Device mode. It requires one hardware UART to work connected to an RS485 transceiver.

## Usage
To use the library, include it before setup and then declare an instance by passing a UART port and the sense pin input/output. On Arduino UNO this will be Serial, but on board with multiple serial ports, you can use Serial2, Serial 3 etc. You cannot use SerialUSB. **Note you can only have one instance at one time**.
```
#include "jvs.h"
JVS jvs(Serial, 2);     // HardwareSerial port, Sense pin
```
Once delcared, you will then need to run the initialization routines at startup either as a master node or a device node.
```
// Init as Device
void setup(){
    jvs.begin(8, false);    // Buffer size of 8 JVS frames, false = not master
    //Serial2.print("Waiting for ID from master node...");
    jvs.initDevice();       // Will hang here until initialized by the master node with and ID
    //Serial2.println("OK!");
}
```
```
// Init as Master
void setup(){
    jvs.begin(8, true);    // Buffer size of 8 JVS frames, false = not master
    //Serial2.println("Assigning device node IDs");
    int found = jvs.initMaster();       // Will hang here until devices are assigned or there is no JVS nodes. Returns number of JVS nodes
    //Serial2.print("Found: ");
    //Serial2.print(found);
    //Serial2.println(" nodes");
}
```
Then you will need to put update() in your loop.
```
void loop(){
    jvs.update();
}
```
Or you can read the data in manually:
```
void loop(){
    if(Serial.available()){
        JVS_Frame inData[8];            // Buffer size of 8, adjust accordingly
        byte ackID = JVS_MASTER_ADDR;   // Master node ID, should always be 0x00
        byte count = 0;                 // Number of received messages
        unsigned long endTimer = millis();
        while(Serial.available() && (millis() - endTimer) >= 10){
            jvs.readFrame(inData[count++]);    // Read in JVS frame.
        }
        jvs.sendAck(ackID);
    }
}
```
## Data Storage/JVS Frame layout
The data for commands and arguments is stored in JVS_Frame.data:
```
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
```
## JVS Info
Both master and device nodes will need some data to report back when requested. The master needs the game name filled in where as devices need to report their name and what features it supports. If no data is supplied, it is prefilled with generic data. **Note: When set as device, _ident is used where as _mainID is used when set to master**.
```
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
```
