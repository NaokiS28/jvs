#ifndef JVS_CPP
#define JVS_CPP

#include "jvs.h"

int JVS::begin(byte _rSize, bool m, unsigned long _b){
    int errorCode = 0;
    _serial->begin(_b);     // 115200 baud with 8N1 encoding
    digitalWrite(_sensePin, LOW);
    if(m){
        // This is the master node
        _isMaster = true;
        _nodeID = 0x00;
        pinMode(_sensePin, INPUT);  // Should be externally pulled low?
    } else {
        setSense(HIGH);         // Set sense pin to 5v (HIGH)
    }

    if(!_rxbuffer.initWithSize(_rSize)){
        bitSet(errorCode, 1);
    }
    if(!_txbuffer.initWithSize(_rSize)){
        bitSet(errorCode, 2);
    }
}

void JVS::reset(){
    setSense(HIGH);         // High means ID not assigned
    _nodeID = 0;
}

void JVS::sendReset(){
    if(_isMaster){
        _outgoingFrame.data[0] = JVS_RESET_CODE;
        _outgoingFrame.data[1] = 0xD9;  // This doesn't change, neither does the code but cest la vie
        sendFrame(JVS_BROADCAST_ADDR, _outgoingFrame);
    } 
    #ifdef JVS_VERBOSE
    else {
        _debugSerial.println(F("JVS: Can't send reset, not master"));
    }
    #endif
}

int JVS::setAddress(){
    if(_isMaster){
        // Keep sending out addresses to devices until master's sense line is pulled low
        bool _senseIn = digitalRead(_sensePin);
        
        uint8_t _id = 0x01; // Start at ID 1
        uint8_t _try = 0;
        uint32_t _recTimeout;
        JVS_Frame _receivedFrame;
        bool sendReceive = 1;   // 1 Send new address, 0 received confirmation
        while(_senseIn){
            _senseIn = digitalRead(_sensePin);
            if(sendReceive){
                // Send new address out
                // Send the address change message
                _outgoingFrame.data[0] = JVS_SETADDR_CODE;
                _outgoingFrame.data[1] = _id; 
                sendFrame(JVS_BROADCAST_ADDR, _outgoingFrame);
                _recTimeout = millis();
            } else {
                if(_serial->available()){
                    readFrame(JVS_MASTER_ADDR, _receivedFrame);
                    // Frame received from device
                    if(_receivedFrame.data[0] == JVS_REPORT_NORMAL){
                        _devicesAvailable++;
                        if(_id < 0x1F) _id++;
                        else {
                            #ifdef JVS_VERBOSE
                            _debugSerial.println(F("JVS: Devices exceed 0x1F??"));
                            #endif
                            break;
                        }
                        sendReceive = 1;
                        _try = 0;
                    } else if (_try < 3){
                        sendReceive = 1;
                        _try++;
                        #ifdef JVS_VERBOSE
                        _debugSerial.print(F("JVS: Bad response from device: 0x"));
                        _debugSerial.println(_receivedFrame.data[0], HEX);
                        #endif
                    } else {
                        sendReceive = 0;
                        #ifdef JVS_VERBOSE
                        _debugSerial.println(F("JVS: Gave up trying"));
                        #endif
                    }
                } else if((millis() - _recTimeout) >= 1000){
                    #ifdef JVS_VERBOSE
                    _debugSerial.println(F("JVS: Timeout error when sending address"));
                    #endif
                    break;
                }
            }
        }
        if(_devicesAvailable == 0){
            // No devices were found
            #ifdef JVS_VERBOSE
            _debugSerial.println(F("JVS: No JVS devices found"));
            #endif
            return 0;
        } else {
            #ifdef JVS_VERBOSE
            _debugSerial.print(F("JVS: Found "));
            _debugSerial.print(_devicesAvailable);
            _debugSerial.println(F(" device(s)"));
            #endif
            return 1;
        }
    }
    #ifdef JVS_VERBOSE
    else {
        _debugSerial.println(F("JVS: Can't set addresses, not master"));
    }
    #endif
    return 0;
}

int JVS::initMaster(){
    // JVS spec says to send reset twice, then auto assign IDs.
    sendReset();
    sendReset();
    setAddress();
}

int JVS::initDevice(){
    // JVS spec says that all devices need to listen to the auto ID function
    // then set it's sense line low when the node has it's ID.
    bool _idAssigned = false;
    JVS_Frame _receivedFrame;
    while(_idAssigned){
        if(_serial->available()){
            int _msgError = readFrame(JVS_MASTER_ADDR, _receivedFrame);
            if(!_msgError){
                // Packet decoded sucessfully
                if(digitalRead(_sensePin) == LOW){
                    // Next device in chain (if there is one) is setup, this is our ID
                    _nodeID = _receivedFrame.data[1];
                    _idAssigned = true;
                    #ifdef JVS_VERBOSE
                    _debugSerial.print(F("JVS: This node's ID: 0x"));
                    _debugSerial.println(_nodeID, HEX);
                    #endif
                } else {
                    // Not our ID
                    _idAssigned = false;
                }
            }
        }
    }
}

int JVS::readFrame(){
    JVS_Frame _frame;

    byte _d = _serial->read();
    byte _errorCode = 1;        // 1 = Bad SOF, 2 = Too big, 3 = Bad sum
    // Start of Frame
    if(_d == 0xD0) {
        // Mark received
        // Sync needs to be received as 0xCF, software adds 1 to make the sync mark.
        // I dont see why but it is what it is
        _d = _serial->read() + 1;
        if(_d == 0xE0) {
            _frame._sync = _d;
            _errorCode == 0;
        }
    }
    if(_errorCode != 0){
        // Bad SOF
        #ifdef JVS_VERBOSE
        _debugSerial.print(F("JVS: Bad SOF"));
        _debugSerial.println(_nodeID, HEX);
        #endif
    } else {
        // Start of frame good
        _frame._nodeID = _serial->read();       // ID of the packet
        _frame._numBytes = _serial->read();     // Number of packet bytes
        if(_frame._numBytes > JVS_MAX_DATA){    // Max frame size is 102 bytes
            // Too big
            #ifdef JVS_VERBOSE
            _debugSerial.print(F("JVS: Packet too big"));
            _debugSerial.println(_nodeID, HEX);
            #endif
        } else {
            _serial->readBytes(_frame.dataString, _frame._numBytes-1);
            _frame._sum = _serial->read();
             // Data read, check sum
            byte _sum = calculateSum(_frame);
            if(_frame._sum != _sum){
                _errorCode = 3;
                #ifdef JVS_VERBOSE
                _debugSerial.print(F("JVS: Bad sum"));
                _debugSerial.println(_nodeID, HEX);
                #endif
            } else {
                _rxbuffer.append(_frame);
            }
        }
    }
    
    if(!_rxbuffer.append(_frame)){
        _errorCode = 4;
    }
    return _errorCode;
}

// Set the node ID. 0x00 is reserved for the master, 0xFF for broadcast packets
uint8_t JVS::setID(uint8_t id){
    if((id != JVS_MASTER_ADDR && !_isMaster) && id != JVS_BROADCAST_ADDR){
        _nodeID = id;
    }
    return _nodeID;
}

int JVS::update(){
    if(_serial->available()){
        JVS_Frame _inData;
        uint8_t result = readFrame();

        if(!_isMaster){    // Only send ack to 0x00 if we're not the master
        _outgoingFrame._nodeID = JVS_MASTER_ADDR;
        switch(result){
            case 1: case 2: case 3:
                // SOF
                // Bad sum
                _outgoingFrame.data[0] = JVS_STATUS_CHECKSUMERROR;
                break;
            default:
                _outgoingFrame.data[0] = JVS_STATUS_NORMAL;
                break;
        }
        sendFrame(JVS_MASTER_ADDR, _outgoingFrame);
        
        }
    }
}

bool JVS::available (void) {
    // Based on code from ACAN2515 by by Pierre Molinaro
    // https://github.com/pierremolinaro/acan2515

    noInterrupts () ;
    const bool hasReceivedMessage = _rxbuffer.count () > 0 ;
    interrupts () ;

  return hasReceivedMessage ;
}

// Sets the JVS sense line. Send HIGH to set sense to 5V
void JVS::setSense(bool s){
    if(!s){
        // Set sense output to active (low)
        pinMode(_sensePin, OUTPUT);
    } else {
        pinMode(_sensePin, INPUT_PULLUP);
    }
}

// Calculate JVS packet checksum
uint8_t JVS::calculateSum(JVS_Frame &_f){
    uint32_t _s;
    _s += _f._nodeID;
    _s += _f._numBytes;
    for(int s = 0; s < _f._numBytes; s++){
        _s += _f.dataString[s];
    }
    _s = (_s % 256);
    return _s;
}


#endif