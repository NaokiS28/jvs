/* 
    JVS Protocol driver
    NKS 2021-2022

    Sense out is timing specific. It must be set low like so:
        If the IO board is the last in the chain:
            Host -> Reset
            Host -> Reset (within 20 ms)
            Host -> SOF 
            Host -> Rest of ID packet.
            Device -> Set sense low
            Device -> Send "Status Normal" response
        
        If the IO board is NOT the last in the chain:
            Set sense low after receiving SOF and sense input is low.
*/

#ifndef JVS_H
#define JVS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <jvs_message.h>
//#include <jvs_commands.h>


// Debugging tools: Use with caution as they may break comms with some hosts
#define DBG_SERIAL Serial           // CHANGE THIS or disable logging if you do not want debug output on this port.
//#define JVS_VERBOSE       // Enables library logging, but might break comms when running as device
#define JVS_ERROR
//#define JVS_VERBOSE_LOG_FRAME     // Enables frame logging but will break comms with certain hosts
//#define JVS_VERBOSE_LOG_CMG       // Same as above but for command packet decoder
//#define JVS_VERBOSE_CMD

#ifdef JVS_VERBOSE
#warning "JVS Verbose mode is enabled. You may experience communication issues or disconnects when running in device mode!"
#endif

#ifdef JVS_VERBOSE_CMD
#ifdef JVS_VERBOSE_LOG_CMD
#ifdef JVS_VERBOSE_LOG_FRAME
#warning "JVS Frame/Command logging is very slow and *will* break device mode!"
#endif
#endif
#endif


#define BUFFER_FULL         1
#define BUFFER_ADD_ERROR    2
#define BUFFER_READ_ERROR   3

#ifndef JVS_BUFFER_SIZE
#define JVS_BUFFER_SIZE 1
#endif

#define JVS_SENSE_INACTIVE  0
#define JVS_SENSE_ACTIVE    1

#define HOST_NODE           1
#define DEVICE_NODE         0

#define JVS_DEFAULT_BUAD    115200
#define JVS_MAX_DATA        254
#define JVS_DEFAULT_ENCODING Mode_8N1

#define JVS_BROADCAST_ADDR  0xFF
#define JVS_HOST_ADDR     0x00

#define JVS_SYNC            0xE0
#define JVS_MARK            0xD0

// Broadcast commands
#define JVS_RESET_CODE      0xF0
#define JVS_SETADDR_CODE    0xF1
#define JVS_COMCHG_CODE     0xF2
// Init commands
#define JVS_IOIDENT_CODE    0x10
#define JVS_CMDREV_CODE     0x11
#define JVS_JVSREV_CODE     0x12
#define JVS_COMVER_CODE     0x13
#define JVS_FEATCHK_CODE    0x14
#define JVS_MAINID_CODE     0x15
// Data I/O commands
#define JVS_READSWITCH_CODE     0x20
#define JVS_READCOIN_CODE       0x21
#define JVS_READANALOG_CODE     0x22
#define JVS_READROTARY_CODE     0x23
#define JVS_READKEY_CODE        0x24
#define JVS_READSCREENPOS_CODE  0x25
#define JVS_READMISC_CODE       0x26
// Output commands
#define JVS_READPAYOUT_CODE     0x2E
#define JVS_DATARETRY_CODE      0x2F
#define JVS_COINDECREASE_CODE   0x30
#define JVS_PAYOUTINCREASE_CODE 0x31
#define JVS_GENERICOUT1_CODE    0x32
#define JVS_ANALOGOUT_CODE      0x33
#define JVS_CHARACTEROUT_CODE   0x34
#define JVS_COININCREASE_CODE   0x35
#define JVS_PAYOUTDECREASE_CODE 0x36
#define JVS_GENERICOUT2_CODE    0x37        // Sega Type 1 IO does not support this command
#define JVS_GENERICOUT3_CODE    0x38        // Sega Type 1 IO does not support this command

// Commands 0x60 to 0x7F are manufacturer specific and not covered here

// Status code
#define JVS_STATUS_NORMAL           1
#define JVS_STATUS_UNKNOWNCMD       2       // Sega IO sends this if there is a parameter error
#define JVS_STATUS_CHECKSUMERROR    3
#define JVS_STATUS_OVERFLOW         4       // Sega IO sends this back when it receives a empty packet

// Report codes
#define JVS_REPORT_NORMAL           1
#define JVS_REPORT_PARAMETERERROR   2
#define JVS_REPORT_DATAERROR        3
#define JVS_REPORT_BUSY             4

// Coin Condition codes
#define JVS_COIN_NORMAL             0
#define JVS_COIN_JAM                1
#define JVS_COIN_NOCOUNTER          2
#define JVS_COIN_BUSY               3

// Library errors
#define JVS_NOTREADY                -1

// JVS Feature list (for use with jvs_message):
#define JVS_FEATURE_END 0
#define JVS_FEATURE_SWITCH 1
#define JVS_FEATURE_COIN 2
#define JVS_FEATURE_ANALOG 3
#define JVS_FEATURE_ROTARY 4
#define JVS_FEATURE_KEYCODE 5
#define JVS_FEATURE_SCREEN 6
#define JVS_FEATURE_MISC 7
#define JVS_FEATURE_CARD 16
#define JVS_FEATURE_MEDAL 17
#define JVS_FEATURE_GPO 18
#define JVS_FEATURE_ANALOG_OUT 19
#define JVS_FEATURE_CHARACTER 20
#define JVS_FEATURE_BACKUP 21

// JVS character output types (for use with jvs_message):
#define JVS_CHARACTER_ASCII 1
#define JVS_CHARACTER_ALPHA 2
#define JVS_CHARACTER_KATA 3
#define JVS_CHARACTER_KANJI 4

#define DEC2BCD(dec) (((dec / 10) << 4) + (dec % 10))
#define BCD2DEC(bcd) (((bcd >> 4) * 10) + (bcd & 0xf))


struct JVS_Controls {
    byte cabinet = 0;

    byte player1[2] = {0, 0};
    byte player2[2] = {0, 0};
    byte player3[2] = {0, 0};
    byte player4[2] = {0, 0};

    byte coin1[2] = {0,0};
    byte coin2[2] = {0,0};
    byte coin3[2] = {0,0};
    byte coin4[2] = {0,0};
};


class JVS {
    public:
    //JVS(HardwareSerial &_ser, int _sense, int _rts);                  // Legacy
    //JVS(HardwareSerial &_ser, int _sense, int _rts, int _senseIn);    // Legacy
    JVS(HardwareSerial &_ser, bool m, int _rts, int _senseA, int _senseB = 255);

    uint8_t nodeID;                // This node's ID

    void reset();
    void sendReset();               // Send reset request out. Only host can initiate this
    void setInfo(JVS_Info &in) { _info = &in; }
    void sendStatus(int s);
    void sendReport(int s, int r);

    void setMachineArray(byte *arr) { machineSwitches = arr; }
    void setPlayerArray(byte *arr) { playerArray = arr; }
    void setAnalogArray(int *arr) { analogArray = arr; }
    void setCoinArray(uint16_t *arr) { coinSlots = arr; }
    void setCoinMechArray(byte *arr) { coinCondition = arr; }
    void setOutputArray(byte *arr) { outputSlots = arr; }

    int begin(unsigned long _b = JVS_DEFAULT_BUAD);
    int available();
    int initHost();
    int initDevice();
    int write(JVS_Frame &_frame);
    int read(JVS_Frame &_frame);
    int update();
    int runCommand();
    int runCommand(JVS_Frame &_b);

    uint8_t findFeatureParam(featureTypes t, uint8_t p) {
        // Returns parameter for given feature. IE asking for parameter 0 for gpOutputs will return number of outputs
        if(p > 2) p = 2;
        return _info->featureParameters[featureLoc[t]][p]; 
    }

    // Host mode commands
    // These only work in host mode. They will fail silently if in device mode
    void ioIdentify(uint8_t id);                                // Request IO Board name
    void requestVersions(uint8_t id);                           // Request concatenated versions
    void writeMainID(uint8_t id);                               // Send host name
    void readSwitches(uint8_t id, uint8_t p, uint8_t d);        // Request switch inputs
    void readCoins(uint8_t id, uint8_t c);                      // Request coin slot status
    void readAnalog(uint8_t id, uint8_t c);                     // Request analog inputs
    void writeAnalog(uint8_t id, uint8_t c, uint16_t d);        // Write analog output data (singular)
    void writeAnalog(uint8_t id, uint8_t c, uint16_t* d);       // Write analog output data (array)
    void writeCharacter(uint8_t id, uint8_t d);                 // Write character display output (singular)
    void writeCharacter(uint8_t id, uint8_t c, uint8_t* d);     // Write character display output (array)
    void readRotary(uint8_t id, uint8_t c);                     // Request rotary input data
    void readKeypad(uint8_t id);                                // Request keypad/keyboard code
    void readSceenPos(uint8_t id, uint8_t c);                   // Request screen pos
    void readMiscSwitch(uint8_t id, uint8_t b);                 // Request misc. input data
    void readPayout(uint8_t id, uint8_t c);                     // Request remaining payouts
    void requestRetransmit(uint8_t id);                         // Command a retransmit of last package
    void increaseCoin(uint8_t id, uint8_t s, uint16_t c);       // Increment a coin counter
    void decreaseCoin(uint8_t id, uint8_t s, uint16_t c);       // Decrement a coin counter
    void increasePayout(uint8_t id, uint8_t s, uint16_t c);     // Increment a hopper payout
    void decreasePayout(uint8_t id, uint8_t s, uint16_t c);     // Decrement a hopper payout

    // GPOs
    void writeOutputs(uint8_t id);                                  // Write outputSlots array
    void writeOutputs(uint8_t id, uint8_t data);                    // Write to first 8 output channels
    void writeOutputs(uint8_t id, uint16_t num, uint8_t* data);     // Write to outputs using GPO1 command
    void writeOutputByte(uint8_t id, uint16_t idx, uint8_t data);   // Write to outputs using GPO2 command
    void writeOutputBit(uint8_t id, uint16_t idx, uint8_t data);    // Write to outputs using GPO3 command

    private:

    //CircularBuffer<JVS_Frame,JVS_BUFFER_SIZE> rxbuffer;   // RX FIFO buffer. Read it fast enough and might not be even needed
    bool rxFlag = false;
    JVS_Frame rxbuffer;
    JVS_Frame _lastSent;
    HardwareSerial* _serial;        // Hardware serial port to use
    JVS_Info*   _info = NULL;              // Pointer to the information array
    //JVS_Frame   _outgoingFrame;     // JVS frame to send out

    // IO
    uint8_t* machineSwitches = NULL;       // Button storage for cab switches: 
        // From MSB->LSB: 7: Test, 6: Tilt 1, 5: Tilt 2, 4: Tilt 3, 3-0: Unused
    uint8_t* playerArray = NULL;          // Pointer to player array. This is read as:
    /* byte playerSwitches[4][2] = {
        // Player 1, note that this is single stick. Dual stick uses buttons 1-4 for UDLR
        // Mahjong uses stick and buttons on byte 0 for A-F (Bits 5 - 0), byte 1 for G-N (Bits 7-0)
        {           
            0,      // MSBFIRST 7: Start, 6: Service, 5: Up, 4: Down, 3: Left, 2: Right, 1: Button 1, 0: Button 2
            0       // MSBFIRST 7: Button 3, 6: Button 4, 5: Button 5, 4: Button 6, 3: Button 7, 2; Button 8, 0-1: Not used 
        },       
        {0,0},      // Player 2
        {0,0},      // Player 3
        {0,0}       // Player 4
    } */
    uint16_t* coinSlots = NULL;     // Pointer to coin slot counter, read as: literal uint counter [per player]
    uint8_t* coinCondition = NULL;     // Coin conditions [per player]
    uint8_t* outputSlots = NULL;
    int* analogArray = NULL;


    bool isHost = false;         // Set to true if this is the host node
    bool jvsReady = false;         // Is set to true when ready to operate
    uint8_t devicesAvailable = 0;     // How many JVS devices are in the chain
    int senseOutPin;                  // Sense pin is connected to a 2N2222 transistor.
    int senseInPin;                // Input sense pin for connecting to downstream IOs
    int rtsPin;                    // Pin for MAX485 transmit enable
    uint8_t featureLoc[32];        // Where the parameters for each feature is stored.

    void setSense(bool s);          // Set the sene pin's output. Only slave should do this, host is read-only
    int  setAddress();
    uint8_t setID(uint8_t id);          // Sets this node's ID, returns current ID
    uint8_t calculateSum(JVS_Frame &_f, bool send = false);
    int readFrame();

    unsigned char reverse(unsigned char b) {
        b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
        b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
        b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
        return b;
    }

    uint32_t lastSent;
};
#endif