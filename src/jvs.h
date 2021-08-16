#ifndef JVS_H
#define JVS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <jvs_message.h>
#include <jvs_buffer.h>

/* 
    JVS Protocol driver
    NKS 2021
*/

#define JVS_DEFAULT_BUAD    115200
#define JVS_MAX_DATA        102
//#define JVS_DEFAULT_ENCODING Mode_8N1

#define JVS_BROADCAST_ADDR  0xFF
#define JVS_MASTER_ADDR     0x00

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
#define JVS_PAYOUTDECREAsE_CODE 0x36
#define JVS_GENERICOUT2_CODE    0x37
#define JVS_GENERICOUT3_CODE    0x38

// Commands 0x60 to 0x7F are manufacturer specific and not covered here

// Status code
#define JVS_STATUS_NORMAL           1
#define JVS_STATUS_UNKNOWNCMD       2
#define JVS_STATUS_CHECKSUMERROR    3
#define JVS_STATUS_ACKERROR         4

// Report codes
#define JVS_REPORT_NORMAL           1
#define JVS_REPORT_PARAMETERERROR   2
#define JVS_REPORT_DATAERROR        3
#define JVS_REPORT_BUSY             4

// Debug mode:
#ifdef JVS_VERBOSE
HardwareSerial _debugSerial = Serial
#endif


class JVS {
    public:
    JVS(HardwareSerial &_ser, int _sense);
    int begin(byte _rSize = 8, bool m = false, unsigned long _b = JVS_DEFAULT_BUAD);
    void reset();
    bool available();
    int initMaster();
    int initDevice();
    int sendFrame(uint8_t _id, JVS_Frame &_frame);
    int readFrame(uint8_t _ackID, JVS_Frame &_frame);
    int update();

    private:
    HardwareSerial* _serial;        // Hardware serial port to use
    JVS_Info*   _info;              // Pointer to the information array
    JVS_Frame   _outgoingFrame;     // JVS frame to send out
    JVSBuffer   _rxbuffer;            // JVS frame buffer
    JVSBuffer   _txbuffer;          // TX buffer

    bool _isMaster = false;         // Set to true if this is the master node
    bool _jvsReady = false;         // Is set to true when ready to operate
    byte _devicesAvailable = 0;     // How many JVS devices are in the chain
    int _sensePin;                  // Sense pin is being used as an open collector output
    uint8_t _nodeID;                // This node's ID
    
    void setSense(bool s);          // Set the sene pin's output. Only slave should do this, master is read-only
    void sendReset();               // Send reset request out. Only master can initiate this
    int  setAddress();
    uint8_t setID(uint8_t id);          // Sets this node's ID, returns current ID
    uint8_t calculateSum(JVS_Frame &_f);
    int readFrame();
};


#endif