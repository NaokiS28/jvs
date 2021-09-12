# jvs
### This is still WIP and may not work correctly!
This is a WIP library for communicating over the JVS protocol in Arduino compatible boards. This library supports both Master mode and Node/Device mode. It requires one hardware UART to work connected to an RS485 transceiver such as the MAX RS485 TTL converter module.

You will also need to connect the sense pin to pin 1 of USB-A connector. This should be pulled to ground with a 1kΩ resistor to GND on the input port. A master can only have 1 input port per instance, a device must have 1 ouput port but may optionally have an input port for daisy chaning IO devices. The data lines are connected between ports, the sense line must connect to an output for the output port and an input for the input port.

## Hardware
# Device mode
This library requires a MAX485 transceiver or equivalent connected to the serial port RX and TX pins. You also need to specify an RTS (Request to Send) pin which is connected to both DE and ~RE.

Finally, in device mode you will either need a transistor (BC547 or 2n2222 or eqv.) for pulling the sense line low; whilst you can use the IO pin directly, it is better practice to do it this way and the library would need to be changed to uninvert the pin. 

Here is an example with downstream IO sensing which is needed for daisychaining.
![alt text](https://github.com/NaokiS28/jvs/tree/main/doc/JVS-Phy.png?raw=true)

# Master mode
In master mode, it is advised to use an LM393 opamp to handle 5V, 2.5v and 0v sensing. This allows one to use one pin to sense IO status (Present with/out ID, not present).

The SEGA/Nintendo/Namco Triforce does single pin sensing which is good enough for most use cases:
![alt text](https://github.com/NaokiS28/jvs/tree/main/doc/triforce-host.png?raw=true)

Another option is to remove the AND gate and use two pins for better sensing. This allows one to know exactly the state of the sense in pin, be it IO board not connected, IO is present with no ID or present and ID'd. The SEGA Chihiro uses a verison of this sensing:
![alt text](https://github.com/NaokiS28/jvs/tree/main/doc/chihiro-host.png?raw=true)

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
    jvs.begin(DEVICE_NODE);    // Begin as device node
    jvs.initDevice();
}
```
```
// Init as Master
void setup(){
    jvs.begin(MASTER_NODE);    // Begin as master node
    int found = jvs.initMaster();       // Will hang here until devices are assigned or there is no JVS nodes. Returns number of JVS nodes
    //Serial2.print("Found: ");
    //Serial2.print(found);
    //Serial2.println(" nodes");
}
```
Then you will need to put update() in your loop. You can have the JVS library run command codes:
```
void loop(){
    if(jvs.update()){
        jvs.runCommand();
    }
}
```
Or you can read the data in manually:
```
void loop(){
    if(jvs.update()){
        JVS_Frame inFrame;      // Frame in
        inFrame = jvs.read();   // Read frame into buffer
    }
}
```
## Data Storage/JVS Frame layout
The data for commands and arguments is stored in JVS_Frame.data:
```
struct JVS_Frame {
    uint8_t sync = 0xE0;            // Sync is always E0
    uint8_t nodeID = 0;             // Node that frame is intended for, 0xFF is a boradcast frame
    uint8_t numBytes = 0;           // Includes all subsequent bytes (Status, data and sum) in frame after this byte
    uint8_t statusCode = 1;
    union{
        uint8_t data [254] = {0};   // Is command and parameters from master, report and results from device
        uint16_t data16 [127];      // Some parameters are 16-bit ints
        char dataString [102];      // When reading node IDs, they are stored as ASCII
    } ;
    uint8_t sum = 0;                // Checksum of ID, numbytes, data bytes
    uint8_t cmdCount = 1;           // Number of commands counted. Only counts standard commands (not manufacturer specific)
};
```
## JVS Info
Both master and device nodes will need some data to report back when requested. The master needs the game name filled in where as devices need to report their name and what features it supports. If no data is supplied, it is prefilled with generic data. **Note: When set as device, _ident is used where as _mainID is used when set to master**.
```
struct JVS_Info {
    char ident[100] = {"JVS IO Library;github.com/NaokiS28/jvs;VER:0.1 Beta"};
    char mainID[100] = {"Generic JVS Maker;Generic JVS Game;Ver A"};
    uint8_t cmdRev = 11;            // Stored as BCD, some JVS hosts (Triforce) will report ver 0.0 if it's higher than 1.1
    uint8_t jvsRev = 30;            // Stored as BCD, JVS rev. 20 and 30 are valid
    uint8_t comRev = 10;            // Stored as BCD, use 10 unless you need to otherwise
    uint8_t totalFeatures = 4;      // How many features supported, this isnt sent to the master but for the library
    featureTypes featureSupport[16] = {
        switchInput, coinInput, analogInput, gpOutput       // Supported features. These are listed in jvs_message.h
    };
    uint8_t featureParameters[16][3] = {                    // Parameters for the list of supported features.
        {2, 12, 0},     // Is switchInput in generic construction
        {2, 0, 0},      // 2 Coin slots
        {6, 10, 0},     // 6 ADCs, 10-Bit resolution
        {8, 0, 0}       // 8 GPOs (general purpose output)
    };
};
```
