/* 
    JVS Protocol driver
    NKS 2021

    Sense out is timing specific. It must be set low like so:
        If the IO board is the last in the chain:
            Master -> Reset
            Master -> Reset (within 20 ms)
            Master -> SOF 
            Device -> Set sense low
            Master -> Rest of ID packet.
        
        If the IO board is NOT the last in the chain:
            Set sense low after receiving SOF and sense input is low.
*/

#ifndef JVS_H
#define JVS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <jvs_message.h>
#include <jvs_commands.h>
#include <CircularBuffer.h>

// Debugging tools: Use with caution as they may break comms with some hosts
//#define DBG_SERIAL Serial
//#define JVS_VERBOSE       // Enables library logging, but might break comms when running as device
//#define JVS_VERBOSE_LOG_FRAME     // Enables frame logging but will break comms with certain hosts
//#define JVS_VERBOSE_LOG_CMG       // Same as above but for command packet decoder

#define BUFFER_FULL         1
#define BUFFER_ADD_ERROR    2
#define BUFFER_READ_ERROR   3

#define JVS_SENSE_INACTIVE  0
#define JVS_SENSE_ACTIVE    1

#define MASTER_NODE         1
#define DEVICE_NODE         0

#define JVS_DEFAULT_BUAD    115200
#define JVS_MAX_DATA        254
#define JVS_DEFAULT_ENCODING Mode_8N1

#define JVS_BROADCAST_ADDR  0xFF
#define JVS_MASTER_ADDR     0x00

#define JVS_SYNC            0xE0

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
#define JVS_STATUS_OVERFLOW         4

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

#ifdef JVS_VERBOSE
#ifndef DBG_SERIAL
#define DBG_SERIAL Serial
#endif
#endif

#define DEC2BCD(dec) (((dec / 10) << 4) + (dec % 10))

class JVS {
    public:
    JVS(HardwareSerial &_ser, int _sense, int _rts);
    JVS(HardwareSerial &_ser, int _sense, int _rts, int _senseIn);
    void reset();
    void sendReset();               // Send reset request out. Only master can initiate this
    void setInfo(JVS_Info &in) { _info = &in; }
    void sendStatus(int s);
    void sendReport(int s, int r);

    int begin(bool m, unsigned long _b = JVS_DEFAULT_BUAD);
    int available();
    int initMaster();
    int initDevice();
    int write(JVS_Frame &_frame);
    int read(JVS_Frame &_frame);
    int update();
    int runCommand();
    int runCommand(JVS_Frame &_b);

    // IO
    byte machineSwitches = 0;       // Button storage for cab switches: 
        // From MSB->LSB: 7: Test, 6: Tilt 1, 5: Tilt 2, 4: Tilt 3, 3-0: Unused
    byte* playerArray = NULL;          // Pointer to player array. This is read as:
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
    byte* coinSlots = NULL;            // Pointer to coin slot array, read as:
    /* byte coinSlots[2][2] = {
        {           // Slot 1
            0,      // MSBFIRST 7-6: Coin condition, 5-0: Coin counter MSB
            0       // MSBFIRST 7-0: Coin counter LSB, total of 16383 coins(!!!)
        },
        { 0, 0 }
    }

    */                                                          

    private:

    CircularBuffer<JVS_Frame,3> rxbuffer;   // RX FIFO buffer. Read it fast enough and might not be even needed
    HardwareSerial* _serial;        // Hardware serial port to use
    JVS_Info*   _info;              // Pointer to the information array
    JVS_Frame   _outgoingFrame;     // JVS frame to send out

    bool isMaster = false;         // Set to true if this is the master node
    bool jvsReady = false;         // Is set to true when ready to operate
    uint8_t devicesAvailable = 0;     // How many JVS devices are in the chain
    int sensePin;                  // Sense pin is connected to a 2N2222 transistor.
    int senseInPin;                // Input sense pin for connecting to downstream IOs
    int rtsPin;                    // Pin for MAX485 transmit enable
    uint8_t nodeID;                // This node's ID

    void setSense(bool s);          // Set the sene pin's output. Only slave should do this, master is read-only
    int  setAddress();
    uint8_t setID(uint8_t id);          // Sets this node's ID, returns current ID
    uint8_t calculateSum(JVS_Frame &_f);
    int readFrame();
};


#endif