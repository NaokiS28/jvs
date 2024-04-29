

#ifndef JVS_CPP
#define JVS_CPP

#include "jvs.h"
/**
 * JVS(*Serial port, *Master/Node, *RTS pin, *Sense pin, Sense pin 2, IO connected pin)
 * @brief Constructor for the JVS library.
 * @param _ser:         The serial port to use. Must be a HardwareSerial class.
 * @param _m:           Use either HOST_NODE or DEVICE_NODE to specify this device's role.
 * @param _rts:         Specify the pin to use for setting the !REN/TEN pins on the MAX485.
 * @param _senseA:      In HOST mode, this is the Sense INPUT. In DEVICE mode, this is the sense OUTPUT.
 * @param _senseB:      (Device only) - This is the Sense INPUT for daisy-chaining IO boards. Not required, use 255 to tell the library to ignore this.
 * @param _ioConnect:   When in HOST mode or DEVICE mode with daisy chaining, the pin given here will be used to test for IO Board presence. Not required.
*/
JVS::JVS(HardwareSerial &_ser, bool _m, int _rts, int _senseA, int _senseB, int _ioConnect){
    _serial = &_ser;
    rtsPin = _rts;

    classOpMode = useUART;

    if(_m == HOST_NODE){
        senseOutPin = 255;
        senseInPin = _senseA;
        if (_ioConnect != 255) { ioConnectedPin = _ioConnect; }
        isHost = HOST_NODE;
    } else {
        senseOutPin = _senseA;
        senseInPin = _senseB;
        isHost = DEVICE_NODE;
    }
    if (_ioConnect != 255) { ioConnectedPin = _ioConnect; }
}

/**
 * JVS(*USB Serial port, *Master/Node, *RTS pin, *Sense pin, Sense pin 2, IO connected pin)
 * @brief Constructor for the JVS library.
 * @param _ser:         The serial port to use. Must be a HardwareSerial class.
 * @param _m:           Use either HOST_NODE or DEVICE_NODE to specify this device's role.
*/
JVS::JVS(usb_serial_class &_ser, bool _m){
    _usb = &_ser;
    rtsPin = 255;

    classOpMode = useUSB;

    senseOutPin = 255;
    senseInPin = 255;
    ioConnectedPin = 255;

    if(_m == HOST_NODE){
        isHost = HOST_NODE;
    } else {
        isHost = DEVICE_NODE;
    }
}

/**
 * JVS(*USB Serial port, *Master/Node, *RTS pin, *Sense pin, Sense pin 2, IO connected pin)
 * @brief Constructor for the JVS library.
 * @param _ser:         The serial port to use. Must be a HardwareSerial class.
 * @param _m:           Use either HOST_NODE or DEVICE_NODE to specify this device's role.
*/
JVS::JVS(usb_serial_class &_ser, bool _m, bool _USEBETA){
    _usb = &_ser;
    rtsPin = 255;

    classOpMode = useBUFFER;

    senseOutPin = 255;
    senseInPin = 255;
    ioConnectedPin = 255;

    if(_m == HOST_NODE){
        isHost = HOST_NODE;
    } else {
        isHost = DEVICE_NODE;
    }
}

/**
 * JVS(*Rx Buffer (uint8_t *), *Tx Buffer (uint8_t *), *Master/Node, *Sense pin 1 (uint8_t *), Sense pin 2 (uint8_t *))
 * You should make sure to use the getSenseOutState and setSenseInState when using this constructor.
 * @brief Constructor for the JVS library.
 * @param _ser:         The serial port to use. Must be a HardwareSerial class.
 * @param _rx:          RX buffer to read from
 * @param _rxSize:      RX buffer size (recommended minimum is 64 bytes)
 * @param _tx:          TX buffer to read from
 * @param _txSize:      TX buffer size (recommended minimum is 64 bytes)
 * @param _m:           Use either HOST_NODE or DEVICE_NODE to specify this device's role.
*/
JVS::JVS(FIFO_Buffer &_rx, FIFO_Buffer &_tx, bool _m){
    //_rxBuff = &_rx;
    //_txBuff = &_tx;

    classOpMode = useBUFFER;

    _m = HOST_NODE;
}

/**
 * begin(_b)
 * @brief       Inits this JVS instace.
 * @param _b:   Specify an override for the Baud rate. Note JVS expect 115200 and you *must* start at this baud rate!
 * @returns:    0 on success, 1 on error.
*/
int JVS::begin(unsigned long _b){
    int errorCode = 0;

    switch (classOpMode)
    {
    case useUART:
        _serial->begin(_b);     // 115200 baud with 8N1 encoding
        pinMode(rtsPin, OUTPUT);
        digitalWrite(rtsPin, LOW);

        //_rxBuff = new FIFO_Buffer();
        //_txBuff = new FIFO_Buffer();

        /*if(_rxBuff == nullptr || _txBuff == nullptr){
            return -1;
        }*/

        if(isHost == HOST_NODE){
            // This is the host node
            nodeID = JVS_HOST_ADDR;
            if(senseInPin == 255){
                errorCode = 1;
            } else {
                pinMode(senseInPin, INPUT_PULLUP);
            }
        } else {
            if(senseInPin != 255) {
                pinMode(senseInPin, INPUT_PULLUP);
            }
            nodeID = 254;
            pinMode(senseOutPin, OUTPUT);
            setSenseOutState(JVS_SENSE_PRESENT);         // Set sense pin to 2.5v
        }
        break;
    case useUSB:
        //_rxBuff = new FIFO_Buffer();
        //_txBuff = new FIFO_Buffer();

        /*if(_rxBuff == nullptr || _txBuff == nullptr){
            return -1;
        }*/

        if(isHost == HOST_NODE){
            // This is the host node
            nodeID = JVS_HOST_ADDR;
        } else {
            nodeID = 254;
        }
        _usb->begin(115200);
        break;
    case useBUFFER:
        break;
    default:
        return -1;
        break;
    }

    if(ioConnectedPin != 255){
        pinMode(ioConnectedPin, INPUT_PULLUP);
    }
    return errorCode;
}


/**
 * reset()
 * @brief (Node only) - Resets the JVS library when in Node mode. Host is never reset.
 * @returns:    Nothing.
*/
void JVS::resetNode(){
    if(!isHost){
        setSenseOutState(JVS_SENSE_PRESENT);         // High means ID not assigned
        nodeID = 254;
        // If we were handling other things like analog outputs, we would need to clear those too.
        if(outputSlots != nullptr) {
            uint8_t tempB = 0;
            for(int x = 0; x < findFeatureParam(gpOutput, 0); x += 8){
                // Count up how many bytes are needed for given parameter
                tempB++;
            }
            memset(outputSlots, 0, tempB);
        }
    }
}

/**
 * sendStatus(int):
 * @brief       (Node only) - Send an empty status frame back to the Host in cases such as an unknown command or bad sum.
 * @param _s:   The status code to send, options can be JVS_STATUSCODE_UNKNOWNCMD, JVS_STATUSCODE_OVERFLOW or JVS_STATUSCODE_CHECKSUMERROR
 * @returns:    Status of the packet transmission
*/
int JVS::sendStatus(int _s){
    if(!isHost){
        JVS_Frame report;
        report.nodeID = JVS_HOST_ADDR;
        report.statusCode = _s;
        return write(report);
    }
    return -1;
}

/**
 *  sendReset():
 *  @brief      (Host only) - Sends a reset sequence to all nodes.
 *  @returns    0 on success, 1 on error.
*/
int JVS::sendReset(){
    if(isHost){
        uint8_t dataBytes[2] = {JVS_RESET_CODE, 0xD9};
        JVS_Frame report(JVS_BROADCAST_ADDR, 2, dataBytes, 0, true, true);
        
        int errorCode = 0;
        for(int i = 0; i < 2; i++){
            errorCode = write(report);
            if(errorCode){
                return errorCode;
            }
            delay(10);
        }
        return errorCode;
    } else {
        return 1;
    }
}

/**
 *  _queryNodes():
 *  @brief      (Host only) - Queries all connected nodes and assigns them an ID until the Sense line is brought low.
 *  @returns    (int) Number of connected nodes, -1: General error, -10: No IOs present, -11: No response was given, -12: bad response
*/
int JVS::_queryNodes(){
    int errorCode = -1;
    //bool senseLine = digitalRead(senseInPin);
    bool done = false;
    byte iosFound = 0;
    byte idNumber = 0x01;
    byte tryCount = 0;

    if(isHost){
        // Keep sending out addresses to devices until host's sense line is pulled low
        if(ioConnectedPin != 255){
            // IO Presence pin is here, use it
            if(!digitalRead(ioConnectedPin)) {   // High when IO board not present
                errorCode = -10;
            }
        } else {
            while (!done){
                JVS_Frame query;
                query.nodeID = JVS_BROADCAST_ADDR;
                query.data[0] = idNumber;

                int txStatus = write(query);
                if(txStatus) { return txStatus; }
                JVS_Frame reply = waitForReply();
                if(reply.sync != JVS_SYNC) { 
                    if(tryCount < 3) tryCount++;
                    else return -11;
                } else {
                    // Got reply
                    if(reply.statusCode != JVS_STATUS_NORMAL || reply.data[0] != JVS_REPORT_NORMAL){ return -12; }
                    else if(reply.statusCode == JVS_STATUS_NORMAL && reply.data[0] == JVS_REPORT_NORMAL){
                        idNumber++;
                        iosFound++;
                    } 
                    if(classOpMode != useUSB){
                        if(!digitalRead(senseInPin)){
                            // Sense in low, last IO board was assigned
                            done = true;
                            errorCode = iosFound;
                        }
                    }
                }
            }
        }
    }
    return errorCode;
}

/**
 *  initHost():
 *  @brief (Host only) - Initializes the JVS library and any connected IO boards.
 *  @returns (int) Number of connected nodes, -1 on error
*/
int JVS::initHost(){
    // JVS spec says to send reset twice, then auto assign IDs.
    while(millis() < 5000){
        // This should be run after a 5 second grace period at startup to allow other IO boards to init.
    }
    sendReset();
    int nodes = _queryNodes();
    if(nodes) { jvsReady = true; }
    return nodes;
}

/**
 * initDevice(_in):
 * @brief       (Device only) - Initializes the JVS library in device mode.
 * @param _in:  JVS_Info array which is needed for running runCommand().
 * @returns     (int) 0 on sucess, 1 if no info array was passed.
*/
int JVS::initDevice(JVS_Info &_in){
    isHost = false;

    _info = &_in;
    // Run through feature support and find out the order given
    // This allows runCommand to know where the parameters are kept
    if(_info != nullptr){
        for(int i = 0; i < _info->totalFeatures; i++){
            featureLoc[_info->featureSupport[i]] = i;
        }
        return 0;
    } else {
        return 1;
    }
}

/**
 * initDevice(_in):
 * @brief       (Device only) - Initializes the JVS library in device mode.
 * @param _in:  JVS_Info array which is needed for running runCommand().
 * @returns     (int) 0 on sucess, 1 if no info array was passed.
*/
int JVS::initDevice(){
    isHost = false;
    return 0;
}

/**
 * write(JVS_Frame):
 * @brief           Writes a JVS packet to the UART port.
 * @param _frame:   JVS_Frame packet to send. This will be stored in-case of retries.
 * @returns         (int) 0 on sucess, 1 if no info array was passed.
*/
int JVS::write(JVS_Frame &_frame){
    writeFrame(_frame);
    _lastSent = _frame;

    return 0;
}

int JVS::writeFrame(JVS_Frame &_frame){
    if(!_frame.escaped){
        _frame.EscapePacket();
    }
    
    uint8_t * msg = _frame.ToArray();
    uint8_t size = _frame.numBytes + (isHost ? 2 : 3);

    #ifdef JVS_VERBOSE
    DBG_SERIAL.println("JVS: writeFrame():");
    DBG_SERIAL.print("Size: ");
    DBG_SERIAL.println(size);
    
    for(uint8_t i = 0; i < size; i++){
        DBG_SERIAL.println(msg[i], HEX);
    }
    #endif

    switch(classOpMode){
        case useUART:
            digitalWrite(rtsPin, HIGH);
            _serial->write(msg, size);
            _serial->flush();
            digitalWrite(rtsPin, LOW);
            break;
        case useUSB:
            _usb->write(msg, size);
            break;
        case useBUFFER:
            //_txBuff->push(_frame.ToArray(), _frame.numBytes + 3);
            txFlag = true;
            break;
        default:
            break;
    }

    delete msg;

    return 0;
}

int JVS::read(JVS_Frame &_frame){
    _frame = rxbuffer;
    rxFlag = false;
    return 0;
}

JVS_Frame JVS::waitForReply(){
    JVS_Frame reply;
    uint32_t startTime = millis();
    uint16_t timeoutPeriod = 1000;
    uint8_t tryCount = 0;
    int _available = 0;

    while(reply.sync != JVS_SYNC){
        while (!(_available >= 5)){
            if(classOpMode == useUSB){
                _available = _usb->available();
            } else {
                _available = _serial->available();
            }
            if(millis() - startTime > (uint32_t)timeoutPeriod){
                if (tryCount > 0 && tryCount < 4){
                    write(_lastSent);
                    startTime = millis();
                } else if (tryCount > 3){
                    // Request timed out
                    reply.sync = 0xFF;
                    return reply;
                } else {
                    delay(10);
                }
            }
        }
        read(reply);
    }
    return reply;
}

int JVS::readFrame(bool _doRetry, uint8_t _idx){
    #ifdef JVS_VERBOSE_CMD
    DBG_SERIAL.println(F("JVS readFrame"));
    #endif
    JVS_Frame _frame;
    byte _d = 0x00;
    //byte _errorCode = 1;        // 1 = Bad SOF, 2 = Too big, 3 = Bad sum
    byte bytesToRead = 0;
    byte dataCount = 0;
    byte index = _idx;
    //bool retry = _doRetry;
    bool markReceived = false;
    int _available  = 0;

    if(classOpMode == useUSB){
        _available = _usb->available();
    } else {
        _available = _serial->available();
    }
    
    #ifdef JVS_VERBOSE
    DBG_SERIAL.print(F("JVS: readFrame: Bytes available: "));
    DBG_SERIAL.println(_available);
    #endif

    if(_available < 5){
        return 1;
    }

    uint8_t c = 0;
    while(index < 6 && _available > 0){
        if(classOpMode == useUSB){
            _d = _usb->read();
            _available = _usb->available();
            //DBG_SERIAL.println(_d);
            //DBG_SERIAL.println(_available);
        } else {
            _d = _serial->read();
            _available = _serial->available();
        }
        
        #ifdef JVS_VERBOSE_CMD
        
        DBG_SERIAL.print(c++);
        DBG_SERIAL.print(": ");
        DBG_SERIAL.println(_d, HEX);
        #endif
        if(_d == JVS_SYNC && index != 0){
            #ifdef JVS_VERBOSE_CMD
            DBG_SERIAL.println("JVS SYNC");
            #endif
            return readFrame(true, 1);
        }
        if(_d != JVS_MARK){
            if (markReceived){
                _d -= 1;
                markReceived = false;
            }
            switch(index){
                case 0:
                    // Sync
                    _frame.sync = _d;
                    if (_frame.sync == JVS_SYNC){ index++; }
                    break;
                case 1:
                    // ID
                    _frame.nodeID = _d;
                    if ((_frame.nodeID == JVS_HOST_ADDR && isHost) || ((_frame.nodeID == nodeID || _frame.nodeID == JVS_BROADCAST_ADDR) && !isHost)){
                        index++;
                    } else {
                        #ifdef JVS_VERBOSE_CMD
                        DBG_SERIAL.println("JVS Wrong ID");
                        #endif
                        return 1;
                    }
                    break;
                case 2:
                    _frame.numBytes = _d;
                    bytesToRead = _available;
                    if (bytesToRead < _frame.numBytes){
                        if (!_waitForBytes(_frame.numBytes)){
                            return 1;
                        }
                    }
                    if (_frame.numBytes < 3){
                        index+=2;
                    } else {
                        index++;
                    }
                    break;
                case 3:
                    if(isHost){
                        _frame.statusCode = _d;
                        if(_frame.statusCode != JVS_STATUS_NORMAL){
                            index++;
                        }
                    } else {
                        _frame.data[dataCount++] = _d;
                    }
                    index++;
                    break;
                case 4:
                    _frame.data[dataCount++] = _d;
                    if(isHost && dataCount >= (_frame.numBytes - 2)){
                        index++;
                    } else if(!isHost && dataCount >= (_frame.numBytes - 1)){
                        index++;
                    }
                    break;
                case 5:
                    _frame.sum = _d;
                    index++;
                    break;
                default:
                    index = 0;
                    break;
            }
        } else {
            markReceived = true;
        }
    }
    if(_frame.sync == JVS_SYNC && _frame.CompareSum(isHost, false)){
        if(isHost){
            switch (_frame.statusCode){
            case JVS_STATUS_NORMAL:
                rxbuffer = _frame;
                rxFlag = true;
                return 0;
                break;
            case JVS_STATUS_CHECKSUMERROR:
                write(_lastSent);
                _frame = waitForReply();
                if(_frame.sync == JVS_SYNC){
                    rxbuffer = _frame;
                    rxFlag = true;
                    return 0;
                } else {
                    return 1;
                }
            default:
                return _frame.statusCode;
                break;
            }
        } else {
            rxbuffer = _frame;
            rxFlag = true;
            return 0;
        }
    } else {
        #ifdef JVS_ERROR
        DBG_SERIAL.println(F("JVS Error: Frame malformed, dropped."));
        #endif
    }
    return 1;
}

int JVS::_waitForBytes(uint16_t totalBytes, uint16_t _tO){
    #ifdef JVS_VERBOSE
    DBG_SERIAL.print(F("JVS: _waitForBytes("));
    DBG_SERIAL.print(totalBytes);
    DBG_SERIAL.println(F(")"));
    #endif
    uint16_t timeout = _tO;     // Timeout 
    uint32_t startTime = millis();
    int _available = 0;

    while (_available < (int)totalBytes){
        if(classOpMode == useUSB){
            _available = _usb->available();
        } else {
            _available = _serial->available();
        }

        if ((millis() - startTime) > timeout){
            return 0;
        }
    }
    return _available;
}

// Set the node ID. 0x00 is reserved for the host, 0xFF for broadcast packets
uint8_t JVS::setID(uint8_t id){
    #ifdef JVS_VERBOSE
    DBG_SERIAL.print(F("JVS: ID: "));
    DBG_SERIAL.println(id);
    #endif
    
    if(id != JVS_HOST_ADDR  && id != JVS_BROADCAST_ADDR){
        nodeID = id;
        jvsReady = true;
        setSenseOutState(JVS_SENSE_READY);
        //sendReport(JVS_STATUS_NORMAL, JVS_REPORT_NORMAL);
        return 0;
    }
    return id;
}

int JVS::update(){
    //byte code = 0;
    switch (classOpMode){
        case useUART:
            if(_serial->available() >= 5 && !rxFlag){
                rxFlag = (readFrame() == 0);
                //Serial.print("JVS: RX Flag: ");
                //Serial.println(rxFlag);
            }
            /*
            while(_serial->available()){
                _rxBuff->push(_serial->read());
                if(_rxBuff->bufferOverflow()){
                    break;
                }
            }
            */
            break;
        case useUSB:
            if(_usb->available() >= 5 && !rxFlag){
                rxFlag = (readFrame() == 0);
            }
        /*
            while(_usb->available()){
                _rxBuff->push(_usb->read());
                if(_rxBuff->bufferOverflow()){
                    break;
                }
            }
            */
            break;
        case useBUFFER:
            break;
        default:
            return 0;
            break;
    }
    /*
    if(_rxBuff->available() >= 5){
        rxFlag = true;
    }
    */
    return rxFlag;
}

int JVS::runCommand(){
    if(rxFlag){
        readFrame();
        rxFlag = false;
        return runCommand(rxbuffer);
    }
    return 0;
}

int JVS::runCommand(JVS_Frame &received){
    if(isHost){
       #ifdef JVS_ERROR
        DBG_SERIAL.println("JVS Error: runCommand only works in device mode!");
        #endif
        return 1; 
    } /*else {
        if(received.nodeID != JVS_BROADCAST_ADDR && received.nodeID != nodeID){
            return 0;
        }
    }*/

    if(_info == NULL){
        #ifdef JVS_ERROR
        DBG_SERIAL.println("JVS Error: Info array not set, runCommand will likely fail!");
        #endif
        return 2;
    }

    int  errorCode = 0;
    // Read the command byte
    uint8_t dataCount = 0;
    int _idx = 0;      // Temp int
    uint16_t temp = 0;       // Generic temp int
    int tempB = 0;      // Generic temp int
    int tempC = 0;      // Generic temp int
    bool assignID = false;     // To specify whether to take the given ID or wait for downstream ports

    bool responseNeeded = true;
    //JVS_Frame response;
    struct {
        uint8_t nodeID = 0;
        uint8_t data[100];
        uint8_t statusCode = JVS_STATUS_NORMAL;
        uint8_t numBytes = 0;
    } response;

    response.nodeID = JVS_HOST_ADDR;
    for(int c = 0; c < received.numBytes-1; c++){
        #ifdef JVS_VERBOSE_CMD
        DBG_SERIAL.print(F("JVS: runCommand: "));
        DBG_SERIAL.println(received.data[c], HEX);
        #endif
        switch (received.data[c]){
            // Defaults to doing nothing, user to respond
            case JVS_RESET_CODE:
                c++;
                if(received.data[c] == 0xD9){
                    #ifdef JVS_VERBOSE
                    DBG_SERIAL.println(F("JVS Recieved: Reset"));
                    #endif
                    nodeID = 254;
                    resetNode();
                    responseNeeded = false;
                } else {
                    // Do nothing?
                }
                break;

            case JVS_SETADDR_CODE:
                #ifdef JVS_VERBOSE
                DBG_SERIAL.println(F("JVS Recieved: Set ID"));
                #endif
                c++;    // Get next data byte
                if(senseInPin != 255){
                    // if sense in pin isnt 254, then look for dwonstream IO
                    // Is downstream port reporting an ID (if one exists)
                    assignID = !digitalRead(senseInPin);
                    #ifdef JVS_VERBOSE
                    DBG_SERIAL.println(senseInPin);
                    DBG_SERIAL.println((assignID ? "assignID: True" : "assignID: False"));
                    #endif
                } else {
                    if(nodeID == 254) {
                        assignID = true;
                    }
                }

                if(assignID){
                    if(setID(received.data[c]) == 254){
                        #ifdef JVS_VERBOSE
                        DBG_SERIAL.println(F("JVS Recieved: Couldn't set ID"));
                        #endif
                    } else {
                        #ifdef JVS_VERBOSE
                        DBG_SERIAL.print(F("JVS: runCommand: nodeID is "));
                        DBG_SERIAL.println(nodeID, HEX);
                        #endif
                        responseNeeded = true;
                        response.data[dataCount++] = JVS_REPORT_NORMAL;
                    }
                } else {
                    #ifdef JVS_VERBOSE
                    DBG_SERIAL.println(F("JVS: Set downstream ID"));
                    #endif
                    responseNeeded = false;
                }
                c++;
                break;

            case JVS_IOIDENT_CODE:
                // IO ident
                #ifdef JVS_VERBOSE
                DBG_SERIAL.println(F("JVS Recieved: Identify"));
                #endif
                responseNeeded = true;
                response.data[dataCount++] = JVS_REPORT_NORMAL;
                for(int s = 0; s < 99; s++){
                    if(_info->ident[s] != 0){
                        response.data[s+1] = _info->ident[s];
                        dataCount++;
                    } else {
                        response.data[s+1] = 0;
                        dataCount++;
                        break;
                    }
                }
                response.data[dataCount++] = 0;
                #ifdef JVS_VERBOSE_CMD_PACKET
                DBG_SERIAL.print(F("OUT: "));
                DBG_SERIAL.println(response.dataString);
                #endif
                break;

            case JVS_CMDREV_CODE:
                // CMD Rev
                #ifdef JVS_VERBOSE
                DBG_SERIAL.println(F("JVS Recieved: Report Command version."));
                #endif
                responseNeeded = true;
                response.data[dataCount++] = JVS_REPORT_NORMAL;
                response.data[dataCount++] = DEC2BCD(_info->cmdRev);
                #ifdef JVS_VERBOSE_CMD_PACKET
                DBG_SERIAL.print(F("START AT: "));
                DBG_SERIAL.println(dataCount-1);
                DBG_SERIAL.println(F("OUT: "));
                DBG_SERIAL.println(response.data[dataCount-1]);
                DBG_SERIAL.println(response.data[dataCount]);
                #endif
                break;

            case JVS_JVSREV_CODE:
                #ifdef JVS_VERBOSE
                DBG_SERIAL.println(F("JVS Recieved: Report JVS version."));
                #endif
                // JVS Rev
                responseNeeded = true;
                response.data[dataCount++] = JVS_REPORT_NORMAL;
                response.data[dataCount++] = DEC2BCD(_info->jvsRev);
                #ifdef JVS_VERBOSE_CMD_PACKET
                DBG_SERIAL.print(F("START AT: "));
                DBG_SERIAL.println(dataCount-1);
                DBG_SERIAL.println(F("OUT: "));
                DBG_SERIAL.println(response.data[dataCount-1]);
                DBG_SERIAL.println(response.data[dataCount]);
                #endif
                break;

            case JVS_COMVER_CODE:
                #ifdef JVS_VERBOSE
                DBG_SERIAL.println(F("JVS Recieved: Report Comms. version."));
                #endif
                // Comm Rev
                responseNeeded = true;
                response.data[dataCount++] = JVS_REPORT_NORMAL;
                response.data[dataCount++] = DEC2BCD(_info->comRev);
                #ifdef JVS_VERBOSE_CMD_PACKET
                DBG_SERIAL.print(F("START AT: "));
                DBG_SERIAL.println(dataCount-1);
                DBG_SERIAL.println(F("OUT: "));
                DBG_SERIAL.println(response.data[dataCount-1]);
                DBG_SERIAL.println(response.data[dataCount]);
                #endif
                break; 

            case JVS_FEATCHK_CODE:
                #ifdef JVS_VERBOSE
                DBG_SERIAL.println(F("JVS Recieved: Report supported features."));
                #endif
                // Feature support
                // For some reason the english translation doesn't mention the feature codes are in BCD
                responseNeeded = true;
                response.data[dataCount++] = JVS_REPORT_NORMAL;
                for(int f = 0; f < _info->totalFeatures; f++){
                    response.data[dataCount++] = DEC2BCD(_info->featureSupport[f]); // D 0
                    response.data[dataCount++] = _info->featureParameters[f][0];    // D 1
                    response.data[dataCount++] = _info->featureParameters[f][1];    // D 2
                    response.data[dataCount++] = _info->featureParameters[f][2];    // D 3
                    #ifdef JVS_VERBOSE_CMD_PACKET
                    DBG_SERIAL.println(f);
                    #endif
                }
                response.data[dataCount++] = 0;
                break;

            case JVS_MAINID_CODE:
                // Main board ident, send ack
                responseNeeded = true;
                //memcpy(_info->mainID, received.dataString, 100);
                _info->mainID[99] = 0;  // Always set last byte to 
                response.statusCode = JVS_STATUS_UNKNOWNCMD;
                //response.data[dataCount++] = JVS_REPORT_NORMAL;
                #ifdef JVS_VERBOSE_CMD_PACKET
                DBG_SERIAL.print(F("IN: "));
                DBG_SERIAL.println(_info->mainID);
                #endif
                dataCount = 0;
                break;

                case JVS_READSWITCH_CODE:
                    // Read switch inputs
                    if(playerArray == NULL){
                        #ifdef JVS_ERROR
                        DBG_SERIAL.println("JVS Error: Host requested switches, player array is NULL!");
                        #endif
                        return -3;
                    }
                    responseNeeded = true;

                    c++;                        // Get next data byte
                    temp = received.data[c++];  // How many players to read
                    if(temp > _info->featureParameters[featureLoc[switchInput]][0]) {
                        errorCode = 1;
                    }

                    tempB = received.data[c];  // How many bytes per player
                    if(tempB > (1 * (_info->featureParameters[featureLoc[switchInput]][1] / 8) + 1)){
                        errorCode = 1;
                    } else {
                        response.data[dataCount++] = JVS_REPORT_NORMAL; // Byte 0 report
                        response.data[dataCount++] = *machineSwitches;   // Byte 1 Test SW, TILT
                        for(byte p = 0; p < temp; p++){
                            // Player number
                            tempC = tempB * p;
                            for(byte b = 0; b < tempB; b++){
                                // Byte number
                                response.data[dataCount++] = playerArray[b + tempC];
                            }
                        }
                        errorCode = 0;
                    }

                    if(errorCode){
                        // If host request more players or buttons than supported, send error
                        memset(response.data, 0, sizeof(response.data));
                        dataCount = 0;
                        response.data[dataCount++] = JVS_REPORT_PARAMETERERROR;
                        errorCode = 0;
                    }
                    break;
            
            case JVS_READCOIN_CODE:
                    // Read coin inputs
                    if(coinSlots == NULL || coinCondition == NULL){
                        #ifdef JVS_ERROR
                        DBG_SERIAL.println("JVS Error: Host requested coins, coin slots/condition array is NULL!");
                        #endif
                        return -4;
                    }
                    responseNeeded = true;
                    c++;    // Get next parameter byte
                    // If host requests 2 players and IO supports 2, p = 0. 
                    // Host should not normally request more than what IO supports
                    if(_info->featureParameters[featureLoc[coinInput]][0] - received.data[c] < 0){
                        // If host request more players than supported, send error
                        response.data[dataCount++] = JVS_REPORT_PARAMETERERROR;
                    } else {
                        temp = received.data[c];  // How many players to read

                        response.data[dataCount++] = JVS_REPORT_NORMAL; // Byte 0 report

                        for(uint16_t p = 0; p < temp; p++){
                            // Coin slot condition
                            tempC = (JVS_COIN_NORMAL << 6);
                            if(coinCondition != nullptr){
                                tempC = (coinCondition[p] << 6); 
                            }
                            // Coin slot count
                            if(coinSlots != nullptr){
                                response.data[dataCount++] = (tempC | (highByte(coinSlots[p]) & 0x3F));
                                response.data[dataCount++] = lowByte(coinSlots[p]);
                            } else {
                                response.data[dataCount++] = 0;
                                response.data[dataCount++] = 0;
                            }
                        }
                    }
                    break;
                case JVS_COINDECREASE_CODE:
                    // Decrease coin counter (the credit count, NOT the coin counter of the cabinet)
                    if(coinSlots == NULL || coinCondition == NULL){
                        #ifdef JVS_ERROR
                        DBG_SERIAL.println("JVS Error: Host requested coins, coin slots/condition array is NULL!");
                        #endif
                        return -4;
                    }
                    responseNeeded = true;
                    c++;
                    _idx = received.data[c++];
                    if(_idx > findFeatureParam(coinInput, 0)){
                        // Check the slots
                        response.data[dataCount++] = JVS_REPORT_PARAMETERERROR;    
                    } else if ((received.numBytes - 1) < 2){
                        // Not enough data in packet to fufill command
                        response.data[dataCount++] = JVS_REPORT_DATAERROR;
                    } else {
                        response.data[dataCount++] = JVS_REPORT_NORMAL;
                        temp = (received.data[c++] << 8);
                        temp |= received.data[c];
                        if(temp > coinSlots[_idx - 1]){
                            coinSlots[_idx - 1] = 0;
                        } else {
                            coinSlots[_idx - 1] -= temp;
                        }
                        #ifdef JVS_VERBOSE_CMD
                        DBG_SERIAL.print(_idx);
                        DBG_SERIAL.print(": ");
                        DBG_SERIAL.print(coinSlots[_idx - 1]);
                        DBG_SERIAL.print(" - ");
                        DBG_SERIAL.println(temp);
                        #endif
                    }
                    break;
                    
                case JVS_COININCREASE_CODE:
                    // Increase coin counter (the credit count, NOT the coin counter of the cabinet)
                    if(coinSlots == NULL || coinCondition == NULL){
                        #ifdef JVS_ERROR
                        DBG_SERIAL.println("JVS Error: Host requested coins, coin slots/condition array is NULL!");
                        #endif
                        return -4;
                    }
                    responseNeeded = true;
                    c++;
                    _idx = received.data[c++];
                    if(_idx > findFeatureParam(coinInput, 0)){
                        // Check the slots
                        response.data[dataCount++] = JVS_REPORT_PARAMETERERROR;    
                    } else if ((received.numBytes - 1) < 2){
                        // Not enough data in packet to fufill command
                        response.data[dataCount++] = JVS_REPORT_DATAERROR;
                    } else {
                        response.data[dataCount++] = JVS_REPORT_NORMAL;
                        temp = (received.data[c++] << 8);
                        temp |= received.data[c];
                        coinSlots[_idx - 1] += temp;

                        #ifdef JVS_VERBOSE_CMD
                        DBG_SERIAL.print(_idx);
                        DBG_SERIAL.print(": ");
                        DBG_SERIAL.print(coinSlots[_idx - 1]);
                        DBG_SERIAL.print(" + ");
                        DBG_SERIAL.println(temp);
                        #endif
                    }
                    break;

                case JVS_GENERICOUT1_CODE:
                    // GPO 1
                    if(outputSlots == NULL){
                        #ifdef JVS_ERROR
                        DBG_SERIAL.println("JVS Error: Host wrote to output, but output array is NULL!");
                        #endif
                        return -5;
                    }
                    #ifdef JVS_VERBOSE_CMD
                        DBG_SERIAL.print("JVS: runCommand(0x32): Output data: ");
                        for(int i = 1; i < received.numBytes - 1; i++){
                            DBG_SERIAL.print(received.data[i], HEX);
                            DBG_SERIAL.print(" ");
                        }
                        DBG_SERIAL.println();
                        #endif

                    responseNeeded = true;
                    c++;
                    tempB = 0;
                    
                    for(int x = 0; x < findFeatureParam(gpOutput, 0); x += 8){
                        tempB++;
                    }
                    if((temp - tempB) < 0){
                        // If host request more bytes than supported, send error
                        response.data[dataCount++] = JVS_REPORT_PARAMETERERROR;
                    } else {
                        response.data[dataCount++] = JVS_REPORT_NORMAL;
                        for(uint16_t o = 0; o < (uint16_t)tempB; o++){
                            outputSlots[o] = received.data[c++];
                        }
                    }
                    break;
                case JVS_DATARETRY_CODE:
                    // Host was not happy with the packet, resend
                    responseNeeded = false; // We need to send a specific packet back
                    write(_lastSent);
                    break;
                // Default case is to send an unknown command response.
                // Any commands executed before this will still send
                // Any commands after this will be not be ran.
                default:
                case 0x02:
                    // Unknown
                        #ifdef JVS_ERROR
                        DBG_SERIAL.print("JVS Warning: Host sent an unkown command: 0x");
                        DBG_SERIAL.println(received.data[c], HEX);
                        #endif
                    responseNeeded = true;
                    response.numBytes = 0;  // Data bytes
                    response.statusCode = JVS_STATUS_UNKNOWNCMD;
                    c+= received.numBytes-1;
                    errorCode = 12;
                    break;
            }
        }

        if(responseNeeded == true){
            if(!errorCode) { response.numBytes = dataCount; }
            #ifdef JVS_VERBOSE
            DBG_SERIAL.print(F("JVS: runCommand: Send reply to: "));
            DBG_SERIAL.println(received.data[0], HEX);
            #endif
            JVS_Frame reply(response.nodeID, response.numBytes, response.data, response.statusCode, false, true);
            write(reply);
        }
    return errorCode;
}

int JVS::available (void) {
  return rxFlag;
}

// Sets the JVS sense line. Send HIGH to set sense to 5V
void JVS::setSense(bool s){
    #ifdef JVS_VERBOSE
    DBG_SERIAL.print(F("JVS: SENSE OUT: "));
    if(s){
        DBG_SERIAL.println("0V");
        digitalWrite(senseOutPin, HIGH);
    }   else {
        DBG_SERIAL.println("2.5V");
        digitalWrite(senseOutPin, LOW);
    }
    #else
    if(useUSB) return;
    else if(s){
        // Set sense output to active
        digitalWrite(senseOutPin, HIGH);
    } else {
        digitalWrite(senseOutPin, LOW);
    }
    #endif
}

// Calculate JVS packet checksum
/*
uint8_t JVS::calculateSum(JVS_Frame &_f, bool _host, bool _send){
    uint32_t _s = 0;
    _s += _f.nodeID;
    _s += _f.numBytes;
    if((!_host && _send) || (_host && !_send)){
        _s += _f.statusCode;
    }

    for(int s = 0; s < _f.numBytes-1; s++){
        _s += _f.dataString[s];
    }
    _s = (_s % 256);
    return _s;
}*/

/* Host mode JVS commands */

void JVS::writeOutputs(uint8_t id){
    // Write outputSlots array
    // Only use for IO boards with more than 8 GPOs
    writeOutputs(id, sizeof(outputSlots), outputSlots);
}

void JVS::writeOutputs(uint8_t id, uint8_t data){
    // GPO1
    // Writes output byte array from GPO 0 to X. Use this for single bytes or when IO board supports 8 or less outputs
    #ifdef JVS_VERBOSE
    DBG_SERIAL.println(F("JVS: Write GPO2"));
    #endif

    if(isHost){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 3;
        out.data[0] = JVS_GENERICOUT1_CODE;
        out.data[1] = 1;
        out.data[2] = reverse(data);    // Why reverse?
        write(out);
    }
}

void JVS::writeOutputs(uint8_t id, uint16_t num, uint8_t* data){
    // GPO1
    // Writes output byte array from GPO 0 to X
    // Only use for IO boards with more than 8 GPOs

    #ifdef JVS_VERBOSE
    DBG_SERIAL.println(F("JVS: Write GPO1"));
    #endif

    if(isHost){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = num + 2;
        out.data[0] = JVS_GENERICOUT1_CODE;
        out.data[1] = num;
        for(uint16_t m = 0; m < num; m++){
            out.data[m + 2] = data[m];
        }
        write(out);
    }
}

void JVS::writeOutputByte(uint8_t id, uint16_t idx, uint8_t data){
    // GPO2
    // Writes output byte array from GPO 0 to X
    // Not all IO boards support this command

    #ifdef JVS_VERBOSE
    DBG_SERIAL.println(F("JVS: Write GPO2"));
    #endif

    if(isHost){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 3;
        out.data[0] = JVS_GENERICOUT2_CODE;
        out.data[1] = idx;
        out.data[2] = data;
        write(out);
    }
}

void JVS::writeOutputBit(uint8_t id, uint16_t idx, uint8_t data){
    // GPO2
    // Writes output byte array from GPO 0 to X
    // Not all IO boards support this command

    #ifdef JVS_VERBOSE
    DBG_SERIAL.println(F("JVS: Write GPO3"));
    #endif
    
    if(isHost){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 3;
        out.data[0] = JVS_GENERICOUT3_CODE;
        out.data[1] = idx;
        if(data > 2) data = 2;
        out.data[2] = data;
        write(out);
    }
}

void JVS::ioIdentify(uint8_t id){
    // Send IO Identify command to ID
    if(isHost){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 1;
        out.data[0] = JVS_IOIDENT_CODE;
        write(out);
    }
}

void JVS::requestVersions(uint8_t id){
    // Send version request as concatenated packet
    // Read reply in order of CMD Rev, JVS Rev, COM Version and then supported features
    if(isHost){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 4;
        out.data[0] = JVS_CMDREV_CODE;
        out.data[1] = JVS_JVSREV_CODE;
        out.data[2] = JVS_COMVER_CODE;
        out.data[3] = JVS_FEATCHK_CODE;
        write(out);
    }
}

void JVS::writeMainID(uint8_t id){
    // Write the ID string located in _info under mainID
    if(isHost){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 1;
        out.data[0] = JVS_MAINID_CODE;
        // We could use strncpy for this but then this way allows us to add
        // a terminating 0 at the last data byte
        for(int s = 0; s <= 100; s++){
            if(_info->mainID[s] != 0 && s < 100){
                out.data[s+1] = _info->mainID[s];
                out.numBytes++;
            } else {
                out.data[s+1] = 0;
                out.numBytes++;
                break;
            }
        }
        write(out);
    }
}

void JVS::readSwitches(uint8_t id, uint8_t p, uint8_t d){
    // Request switch data from ID. P = How many players, D = how many bytes per player
    if(isHost && p > 0 && d > 0){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 3;
        out.data[0] = JVS_READSWITCH_CODE;
        out.data[1] = p;
        out.data[2] = d;
        write(out);
    }
}

void JVS::readCoins(uint8_t id, uint8_t c){
    // Request coin data from ID. C = How many coin slots to read
    if(isHost && c > 0){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 2;
        out.data[0] = JVS_READCOIN_CODE;
        out.data[1] = c;
        write(out);
    }
}

void JVS::readAnalog(uint8_t id, uint8_t c){
    // Request analog data from ID. C = How many channels to read
    if(isHost && c > 0){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 2;
        out.data[0] = JVS_READANALOG_CODE;
        out.data[1] = c;
        write(out);
    }
}

void JVS::writeAnalog(uint8_t id, uint8_t c, uint16_t d){
    // Request misc. switch data from ID. B = how many bytes to read
    if(isHost && c > 0){
        JVS_Frame out;
        out.numBytes = 4;
        out.data[0] = JVS_ANALOGOUT_CODE;
        out.data[1] = c;
        out.data[2] = highByte(c);
        out.data[3] = lowByte(c);
        write(out);
    }
}

void JVS::writeAnalog(uint8_t id, uint8_t c, uint16_t* d){
    // Request misc. switch data from ID. B = how many bytes to read
    if(isHost && c > 0 && c < 250 && d != NULL){
        JVS_Frame out;
        out.numBytes = 2;
        out.data[0] = JVS_ANALOGOUT_CODE;
        out.data[1] = c;
        for(int a = 0; a < c; a++){
            out.data[a + 2] = highByte(c);
            out.data[a + 3] = lowByte(c);
            out.numBytes += 2;
        }
        write(out);
    }
}

void JVS::writeCharacter(uint8_t id, uint8_t d){
    // Request misc. switch data from ID. B = how many bytes to read
    if(isHost){
        JVS_Frame out;
        out.numBytes = 3;
        out.data[0] = JVS_CHARACTEROUT_CODE;
        out.data[1] = 1;
        out.data[2] = d;
        write(out);
    }
}

void JVS::writeCharacter(uint8_t id, uint8_t c, uint8_t* d){
    // Request misc. switch data from ID. B = how many bytes to read
    if(isHost && c > 0 && c < 250 && d != NULL){
        JVS_Frame out;
        out.numBytes = 2;
        out.data[0] = JVS_CHARACTEROUT_CODE;
        out.data[1] = c;
        for(int a = 0; a < c; a++){
            out.data[a + 2] = d[a];
            out.numBytes ++;
        }
        write(out);
    }
}

void JVS::readRotary(uint8_t id, uint8_t c){
    // Request rotary data from ID. C = How many channels to read
    if(isHost && c > 0){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 2;
        out.data[0] = JVS_READROTARY_CODE;
        out.data[1] = c;
        write(out);
    }
}

void JVS::readKeypad(uint8_t id){
    // Request keypad data from ID.
    if(isHost){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 1;
        out.data[0] = JVS_READKEY_CODE;
        write(out);
    }
}

void JVS::readSceenPos(uint8_t id, uint8_t c){
    // Request touch screen position data from ID. C = Channel index
    if(isHost && c > 0){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 2;
        out.data[0] = JVS_READSCREENPOS_CODE;
        out.data[1] = c;
        write(out);
    }
}

void JVS::readMiscSwitch(uint8_t id, uint8_t b){
    // Request misc. switch data from ID. B = how many bytes to read
    if(isHost && b > 0){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 2;
        out.data[0] = JVS_READMISC_CODE;
        out.data[1] = b;
        write(out);
    }
}

void JVS::readPayout(uint8_t id, uint8_t c){
    // Request payout hopper data from ID. c = Which channel to read
    if(isHost && c > 0){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 2;
        out.data[0] = JVS_READPAYOUT_CODE;
        out.data[1] = c;
        write(out);
    }
}

void JVS::requestRetransmit(uint8_t id){
    // Request retransmit of last packet
    if(isHost){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 1;
        out.data[0] = JVS_DATARETRY_CODE;
        write(out);
    }
}

void JVS::increaseCoin(uint8_t id, uint8_t s, uint16_t c){
    // Increment coin counter amount on IO. S = Which slot, C = Amount
    if(isHost && s > 0 && c > 0){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 4;
        out.data[0] = JVS_COININCREASE_CODE;
        out.data[1] = s;
        out.data[2] = highByte(c);
        out.data[3] = lowByte(c);
        write(out);
    }
}

void JVS::decreaseCoin(uint8_t id, uint8_t s, uint16_t c){
    // Decrement coin counter amount on IO. S = Which slot, C = Amount
    if(isHost && s > 0 && c > 0){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 4;
        out.data[0] = JVS_COINDECREASE_CODE;
        out.data[1] = s;
        out.data[2] = highByte(c);
        out.data[3] = lowByte(c);
        write(out);
    }
}

void JVS::increasePayout(uint8_t id, uint8_t s, uint16_t c){
    // Increase payout amount for IO to issue. S = Which slot, C = Amount
    if(isHost && s > 0 && c > 0){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 4;
        out.data[0] = JVS_PAYOUTINCREASE_CODE;
        out.data[1] = s;
        out.data[2] = highByte(c);
        out.data[3] = lowByte(c);
        write(out);
    }
}

void JVS::decreasePayout(uint8_t id, uint8_t s, uint16_t c){
    // Increase payout amount for IO to issue. S = Which slot, C = Amount
    if(isHost && s > 0 && c > 0){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 4;
        out.data[0] = JVS_PAYOUTDECREASE_CODE;
        out.data[1] = s;
        out.data[2] = highByte(c);
        out.data[3] = lowByte(c);
        write(out);
    }
}

#endif