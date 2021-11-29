#ifndef JVS_CPP
#define JVS_CPP

#include "jvs.h"

JVS::JVS(HardwareSerial &_ser, int _sense, int _rts){
    _serial = &_ser;
    sensePin = _sense;
    senseInPin = 255;
    rtsPin = _rts;
}

JVS::JVS(HardwareSerial &_ser, int _sense, int _rts, int _senseIn){
    _serial = &_ser;
    sensePin = _sense;
    senseInPin = _senseIn;
    rtsPin = _rts;
}

int JVS::begin(bool m, unsigned long _b){
    int errorCode = 0;
    _serial->begin(_b);     // 115200 baud with 8N1 encoding
    pinMode(rtsPin, OUTPUT);
    digitalWrite(rtsPin, LOW);

    if(m == MASTER_NODE){
        #ifdef JVS_VERBOSE
        DBG_SERIAL.println(F("JVS: MASTER MODE"));
        #endif
        // This is the master node
        nodeID = JVS_MASTER_ADDR;
        isMaster = true;
        if(senseInPin == 255){
            #ifdef JVS_VERBOSE
            DBG_SERIAL.println(F("ERROR: NO SENSE INPUT PIN GIVEN"));
            #endif
        } else {
            pinMode(senseInPin, INPUT_PULLUP);
        }
    } else {
        #ifdef JVS_VERBOSE
        DBG_SERIAL.println(F("JVS: DEVICE MODE"));
        #endif
        isMaster = false;
        if(senseInPin < 255) {
            pinMode(senseInPin, INPUT_PULLUP);
            #ifdef JVS_VERBOSE
            DBG_SERIAL.println(F("JVS: Downstream Enabled"));
            #endif
        }
        pinMode(sensePin, OUTPUT);
        setSense(JVS_SENSE_INACTIVE);         // Set sense pin to 2.5v
    }

    return errorCode;
}

void JVS::reset(){
    #ifdef JVS_VERBOSE
    DBG_SERIAL.println(F("JVS: RESET"));
    #endif

    setSense(JVS_SENSE_INACTIVE);         // High means ID not assigned
    nodeID = 0;
}

void JVS::sendStatus(int s){
    if(!isMaster){
        #ifdef JVS_VERBOSE
        DBG_SERIAL.print(F("JVS: SEND STATUS: "));
        DBG_SERIAL.println(s, HEX);
        #endif
        sendReport(s, 0);
    } 
}

void JVS::sendReport(int s, int r){
    if(!isMaster){
        JVS_Frame report;
        if(r) {
            report.numBytes = 1;            // Send 1 report byte back
            report.data[0] = r;
            #ifdef JVS_VERBOSE
            DBG_SERIAL.print(F("JVS: SEND REPORT: "));
            DBG_SERIAL.println(r, HEX);
            #endif
        }
        report.nodeID = JVS_MASTER_ADDR;
        report.statusCode = s;
        write(report);
    } 
}

void JVS::sendReset(){
    devicesAvailable = 0;
    if(isMaster){
        JVS_Frame report;
        #ifdef JVS_VERBOSE
        DBG_SERIAL.println(F("JVS: SEND RESET"));
        #endif
        report.numBytes = 2;
        report.nodeID = JVS_BROADCAST_ADDR;
        report.data[0] = JVS_RESET_CODE;
        report.data[1] = 0xD9;  // This doesn't change, neither does the code but cest la vie
        write(report);
    } 
    #ifdef JVS_ERROR
    else {
        DBG_SERIAL.println(F("JVS: Can't send reset, not master"));
    }
    #endif
}

int JVS::setAddress(){
    // Tidy this up?
    if(isMaster){
        // Keep sending out addresses to devices until master's sense line is pulled low
        bool _senseIn = digitalRead(senseInPin);
        
        #ifdef JVS_VERBOSE
        DBG_SERIAL.println(F("JVS: Set IDs"));
        #endif

        uint8_t _id = 0x01; // Start at ID 1
        uint8_t _try = 0;
        uint32_t recTimeout;

        bool sendReceive = 1;   // 1 Send new address, 0 received confirmation
        bool breakLoop = false;

        // Sense no IO board connected
        while(!breakLoop){
            _senseIn = digitalRead(senseInPin);
            if(sendReceive){
                if(!digitalRead(senseInPin)){
                    #ifdef JVS_VERBOSE
                    if(digitalRead(senseInPin) == LOW) DBG_SERIAL.println(F("JVS: Sense low"));
                    #endif
                    breakLoop = true;
                } else {
                #ifdef JVS_VERBOSE
                DBG_SERIAL.print(F("JVS: SEND ID: "));
                DBG_SERIAL.println(_id);
                #endif
                JVS_Frame _request;
                // Send new address out
                // Send the address change message
                _request.nodeID = JVS_BROADCAST_ADDR;
                _request.numBytes = 2;
                _request.data[0] = JVS_SETADDR_CODE;
                _request.data[1] = _id; 
                write(_request);
                recTimeout = millis();
                sendReceive = 0;
                }
            } else {
                if(update()){
                    // Frame received from device
                    JVS_Frame _receivedFrame;
                    read(_receivedFrame);
                    if(_receivedFrame.data[0] == JVS_REPORT_NORMAL){
                        if(_id < 0x0F) {
                            #ifdef JVS_VERBOSE
                            DBG_SERIAL.print(F("JVS: Found device: "));
                            DBG_SERIAL.println(_id);
                            #endif
                            _id++;
                            devicesAvailable++;
                            delay(80);
                        } else {
                            #ifdef JVS_ERROR
                            DBG_SERIAL.println(F("JVS: Devices exceed 0x1F??"));
                            #endif
                            break;
                        }
                        sendReceive = 1;
                        _try = 0;
                    } else if (_try < 3){
                        sendReceive = 1;
                        _try++;
                        #ifdef JVS_ERROR
                        DBG_SERIAL.print(F("JVS: Bad response from device: 0x"));
                        DBG_SERIAL.println(_receivedFrame.data[0], HEX);
                        #endif
                    } else {
                        sendReceive = 0;
                        #ifdef JVS_ERROR
                        DBG_SERIAL.println(F("JVS: Gave up trying"));
                        #endif
                    }
                } /*else if(_id < 0x0F && (millis() - recTimeout) >= 150 && digitalRead(senseInPin) == HIGH){
                    _id++;
                    endReceive = 1;
                } */else if((millis() - recTimeout) >= 150){
                    #ifdef JVS_ERROR
                    DBG_SERIAL.println(F("JVS: Timeout error whilst waiting for report"));
                    #endif
                    breakLoop = true;
                }
            }
        }

        if(devicesAvailable == 0){
            // No devices were found
            #ifdef JVS_ERROR
            DBG_SERIAL.println(F("JVS: No JVS devices found"));
            #endif
            return 0;
        } else {
            #ifdef JVS_VERBOSE
            DBG_SERIAL.print(F("JVS: Found "));
            DBG_SERIAL.print(devicesAvailable);
            DBG_SERIAL.println(F(" device(s)"));
            #endif
            return devicesAvailable;
        }
    }
    #ifdef JVS_VERBOSE
    else {
        DBG_SERIAL.println(F("JVS: Can't set addresses, not master"));
    }
    #endif
    return 0;
}

int JVS::initMaster(){
    // JVS spec says to send reset twice, then auto assign IDs.
    // This should be run after a 5 second grace period at startup
    #ifdef JVS_VERBOSE
    DBG_SERIAL.println(F("JVS: INIT MASTER"));
    #endif
    sendReset();
    delay(35);
    sendReset();
    delay(35);
    byte nodes = setAddress();
    jvsReady = true;
    return nodes;
}

int JVS::initDevice(){
    // JVS spec says that all devices need to listen to the auto ID function
    // then set it's sense line low when the node has it's ID.
    #ifdef JVS_VERBOSE
    DBG_SERIAL.println(F("JVS: INIT DEVICE"));
    #endif
    isMaster = false;

    // Run through feature support and find out the order given
    // This allows runCommand to know where the parameters are kept
    for(int i = 0; i < _info->totalFeatures; i++){
        featureLoc[_info->featureSupport[i]] = i;
    }
    return 0;
}

int JVS::write(JVS_Frame &_frame){
    while(millis() - lastSent < 40);

    uint8_t d = 0;
    uint8_t b = _frame.numBytes;
    uint8_t errorCode = 0;

    digitalWrite(rtsPin, HIGH);
    delayMicroseconds(100);
    // Sum is included in Number of bytes, adding 1 allows new users to specify data bytes more intuitively
    // Counts Status, Report, Data and Sum
    if(isMaster){
        // Data and sum
        _frame.numBytes++; 
    } else {
        _frame.numBytes+= 2; // For Status and Sum byte
    }

    uint8_t sumByte = calculateSum(_frame, true);

    _serial->write(_frame.sync);       // Sync byte, E0, must be sent as start of frame (SOF)
    //_serial->flush();
    _serial->write(_frame.nodeID);     // Send to ID
    //_serial->flush();
    
    _serial->write(_frame.numBytes);   // How many bytes to send (excludes status)
    _serial->flush();
    _frame.numBytes = b;
    if(!isMaster){
        // Device needs to sen ack codes
        _serial->write(_frame.statusCode);   // Status code
        _serial->flush();
        if(_frame.statusCode != JVS_STATUS_NORMAL){
            errorCode = 2;
            sumByte = 4;    // Naughty hack
        }
    } 
    if(!errorCode || errorCode == 2){
        for(int f = 0; f < b; f++){
            d = _frame.data[f];
            // As JVS_SYNC is used to signal SOF, this check is to see if D0 needs to be injected to signify
            //  that the byte is not a SOF.
            if(d == JVS_SYNC) {
                _serial->write(0xD0);
                d--;
                //_serial->flush();
            }
            _serial->write(d);
            _serial->flush();
        }
    }
    _serial->write(sumByte);
    _serial->flush();
    digitalWrite(rtsPin, LOW);

    lastSent = millis();

    // Log after because JVS is impatient
    #ifdef JVS_VERBOSE_LOG_FRAME
    DBG_SERIAL.println();
    DBG_SERIAL.println(F("JVS: Send frame:"));
    DBG_SERIAL.print(F("Sync: "));
    DBG_SERIAL.println(_frame.sync, HEX);
    DBG_SERIAL.print(F("ID: "));
    DBG_SERIAL.println(_frame.nodeID, HEX);
    DBG_SERIAL.print(F("# of bytes: "));
    DBG_SERIAL.println(_frame.numBytes +1);
    if(!isMaster){
        DBG_SERIAL.print(F("Status: "));
        DBG_SERIAL.println(_frame.statusCode);
    }
    DBG_SERIAL.println(F("Bytes: "));
    for(int db = 0; db < b; db++){
        DBG_SERIAL.print(_frame.data[db], HEX);
        DBG_SERIAL.print(' ');
    }
    DBG_SERIAL.println();
    DBG_SERIAL.print(F("Sum: "));
    DBG_SERIAL.println(sumByte, HEX);
    DBG_SERIAL.println();
    #endif
    return 0;
}

int JVS::read(JVS_Frame &_frame){
    _frame = rxbuffer; //.shift();
    rxFlag = false;
    return 0;
}

int JVS::readFrame(){
    JVS_Frame _frame;
    byte _d = _serial->read();
    byte _errorCode = 1;        // 1 = Bad SOF, 2 = Too big, 3 = Bad sum
    byte _sum = 0;
    byte bytesToRead = 0;

    // Start of Frame
    if(_d == 0xE0) {
        _frame.sync = _d;
        _errorCode = 0;
    }
    /*
    if(_errorCode != 0){
        // Bad SOF
        #ifdef JVS_ERROR
        DBG_SERIAL.print(F("JVS: Bad SOF: 0x"));
        DBG_SERIAL.println(_frame.sync, HEX);
        #endif
        while(_serial->available()) {
            uint8_t in = _serial->read();
            if(in == JVS_SYNC){
                _errorCode = 0;
                _frame.sync = in;
                break;
            }
        }
    }
    */
    while(!_serial->available());
    if(_errorCode == 0) {
        // Start of frame good
        _frame.nodeID = _serial->read();       // ID of the packet
        while(!_serial->available());
        _frame.numBytes = _serial->read();     // Number of packet bytes
        _frame.cmdCount = _frame.numBytes - 1;
        while(!_serial->available());
        if(isMaster){
        // Device needs to sen ack codes
            _frame.statusCode = _serial->read();   // Status code
            bytesToRead = _frame.numBytes-1;  // For status code
            // Need to process status in future
        } else {
            // Master will not send these, calculateSum needs them, so blank them for correct sum
            _frame.statusCode = 0;
            bytesToRead = _frame.numBytes;
        }

        if(_frame.numBytes > JVS_MAX_DATA){    // Max frame size is 102 bytes. Minimum size is 2 bytes (Number of bytes is counted)
            // Too big
            _errorCode = 2;
            #ifdef JVS_ERROR
            DBG_SERIAL.print(F("JVS: Message has too many data bytes: "));
            DBG_SERIAL.println(_frame.numBytes);
            #endif
        } else {
            // Check for  blank frame
            if(_frame.numBytes-1 == 0) {
                #ifdef JVS_ERROR
                DBG_SERIAL.println(F("JVS Recieved: Unknown/blank frame"));
                #endif
                sendStatus(JVS_STATUS_UNKNOWNCMD);
                _errorCode = 4;
            } else {
                // Read frame data bytes
                for(int i = 0; i < bytesToRead-1; i++){
                    while(!_serial->available());
                    byte b = _serial->read();
                    if(b == 0xD0 && (bytesToRead-1 > 0)) {
                        // Mark received
                        // This means this byte equals a SOF byte and so needs to be translated
                        _frame.data[i] = _serial->read() + 1;
                    } else {
                        _frame.data[i] = b;
                    }
                }
            }
            // Sum byte
            while(!_serial->available());
            _frame.sum = _serial->read();

            // Dumping the FCA2 PCB showed that you need to escape the sum byte too
            if(_frame.sum == 0xD0) {
                while(!_serial->available());
                _frame.sum = _serial->read() + 1;
            }

            // Data read, check sum
            _sum = calculateSum(_frame);

            if(_frame.sum != _sum){
                #ifdef JVS_ERROR
                DBG_SERIAL.print(F("JVS: Received bad sum, expected: 0x"));
                DBG_SERIAL.print(_sum, HEX);
                DBG_SERIAL.print(F(" got: 0x"));
                DBG_SERIAL.println(_frame.sum, HEX);
                #endif
                _errorCode = 3;
                sendStatus(JVS_STATUS_CHECKSUMERROR);
            }
        }
    }
    if(_errorCode == 0){ //&& rxbuffer.available()){
        #ifdef JVS_VERBOSE
        DBG_SERIAL.println(F("JVS Recieved: To buffer"));
        #endif
        rxFlag = true;
        rxbuffer = _frame; //.push(_frame);
    }
    
    // Log this after as JVS is an impatient protocol
    #ifdef JVS_VERBOSE_LOG_FRAME
    DBG_SERIAL.println();
    DBG_SERIAL.println(F("JVS: Start of Frame"));
    DBG_SERIAL.print(F("JVS: For ID: "));
    DBG_SERIAL.println(_frame.nodeID, HEX);
    DBG_SERIAL.print(F("JVS: # of Bytes: "));
    DBG_SERIAL.println(_frame.numBytes, HEX);
    DBG_SERIAL.println(F("JVS: Data bytes:"));
    for(int db = 0; db < _frame.numBytes-2; db++){
        DBG_SERIAL.println(_frame.data[db], HEX);
    }
    DBG_SERIAL.print(F("JVS: Received Sum:"));
    DBG_SERIAL.println(_frame.sum, HEX);
    switch(_errorCode){
        case 1:
            DBG_SERIAL.print(F("JVS: Bad SOF"));
            DBG_SERIAL.println(nodeID, HEX);
            break;
        case 2:
            DBG_SERIAL.print(F("JVS: Packet too big"));
            DBG_SERIAL.println(nodeID, HEX);
            break;
        case 3:
            DBG_SERIAL.print(F("JVS: Bad sum, expected: "));
            DBG_SERIAL.println(_sum, HEX);
            break;
        }
    #endif
    
    return _errorCode;
}

// Set the node ID. 0x00 is reserved for the master, 0xFF for broadcast packets
uint8_t JVS::setID(uint8_t id){
    #ifdef JVS_VERBOSE
    DBG_SERIAL.print(F("JVS: ID: "));
    DBG_SERIAL.println(id);
    #endif
    
    if(id != JVS_MASTER_ADDR  && id != JVS_BROADCAST_ADDR){
        nodeID = id;
        jvsReady = true;
        setSense(JVS_SENSE_ACTIVE);
        //sendReport(JVS_STATUS_NORMAL, JVS_REPORT_NORMAL);
        return 0;
    }
    return 1;
}

int JVS::update(){
    // static uint32_t lastReceived;

    if(_serial->available()){
        readFrame(); // lastReceived = millis();
    } 
    return available();
}

int JVS::runCommand(){
    rxFlag = false;
    return runCommand(rxbuffer);
    /*
    if(rxbuffer.size()){
        JVS_Frame _b = rxbuffer.shift();
        return runCommand(_b);
    } else {
        return 1;
    }*/
}

int JVS::runCommand(JVS_Frame &received){
    int  errorCode = 0;
    // Read the command byte
    uint8_t dataCount = 0;
    int index = 0;      // Temp int
    int temp = 0;       // Generic temp int
    int tempB = 0;      // Generic temp int
    //int tempC = 0;      // Generic temp int
    bool n = false;

    bool responseNeeded = true;
        JVS_Frame response;

        response.nodeID = JVS_MASTER_ADDR;
        for(int c = 0; c < received.numBytes-1; c++){
            //if(c != 0) dataCount++;

            if(senseInPin < 255){
                // Is downstream port reporting an ID (if one exists)
                n = 0; //digitalRead(senseInPin);
                #ifdef JVS_VERBOSE
                DBG_SERIAL.println(senseInPin);
                DBG_SERIAL.println(n);
                #endif
            }

            #ifdef JVS_VERBOSE_CMD
            //DBG_SERIAL.print(F("C: "));
            //DBG_SERIAL.println(c);
            //DBG_SERIAL.print(F("CM: "));
            //DBG_SERIAL.println(received.numBytes-1);
            DBG_SERIAL.print(F("COMMAND: "));
            DBG_SERIAL.println(received.data[c], HEX);
            #endif
            switch (received.data[c]){
                // Defaults to doing nothing, user to respond
                case 0xF0:
                    #ifdef JVS_VERBOSE
                    DBG_SERIAL.println(F("JVS Recieved: Reset"));
                    #endif
                    digitalWrite(rtsPin, LOW);
                    reset();
                    responseNeeded = false;
                    c++;
                    break;
                case 0xF1:
                    #ifdef JVS_VERBOSE
                    DBG_SERIAL.println(F("JVS Recieved: Set ID"));
                    #endif
                    if(!n){
                        if(setID(received.data[c+1]) != 0){
                            #ifdef JVS_VERBOSE
                            DBG_SERIAL.println(F("JVS Recieved: Couldn't set ID"));
                            #endif
                        } else {
                            responseNeeded = true;
                            response.statusCode = JVS_STATUS_NORMAL;
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
                case 0x2F:
                    #ifdef JVS_VERBOSE
                    DBG_SERIAL.println(F("JVS Recieved: Bad sum, resend"));
                    #endif
                    //write(_outgoingFrame);
                    responseNeeded = false;
                    break;
                case 0x10:
                    // IO ident
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
                case 0x11:
                    // CMD Rev
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

                case 0x12:
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
    
                case 0x13:
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
                    
                case 0x14:
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

                case 0x15:
                    // Main board ident, send ack
                    responseNeeded = true;
                    memcpy(_info->mainID, received.dataString, 100);
                    _info->mainID[100] = 0;
                    response.statusCode = JVS_STATUS_UNKNOWNCMD;
                    response.data[dataCount++] = JVS_REPORT_NORMAL;
                    #ifdef JVS_VERBOSE_CMD_PACKET
                    DBG_SERIAL.print(F("IN: "));
                    DBG_SERIAL.println(_info->mainID);
                    #endif
                    c+= received.numBytes-1;
                    break;

                case 0x20:
                    // Read switch inputs
                    responseNeeded = true;
                    c++;
                    // If master requests 2 players and IO supports 2, p = 0. 
                    // Master should not normally request more than what IO supports
                    if(_info->featureParameters[featureLoc[switchInput]][0] - received.data[c] < 0){
                        // If master request more players than supported, send error
                        response.data[dataCount++] = JVS_REPORT_PARAMETERERROR;
                    } else {
                        temp = received.data[c++];  // How many players to read
                        tempB = _info->featureParameters[featureLoc[switchInput]][0];   // How many players
                        if(tempB >= temp) {
                            // Set player count to tempB
                            tempB = temp;
                        }
                        temp = received.data[c];  // How many bytes per player
                        //tempC = _info->featureParameters[featureLoc[switchInput]][1];   // How many switches per player
                        //Serial.println('S');Serial.println(temp);Serial.println(tempB);Serial.println(tempC);
                        response.data[dataCount++] = JVS_REPORT_NORMAL; // Byte 0 report
                        response.data[dataCount++] = machineSwitches;   // Byte 1 Test SW, TILT

                        for(byte p = 0; p < tempB; p++){
                            // Player number
                            for(byte b = 0; b < temp; b++){
                                // Byte number
                                //|| ((b * 8) <= tempC)
                                if(playerArray != nullptr){
                                    response.data[dataCount++] = *(playerArray + (2 * p) + b);
                                } else {
                                    response.data[dataCount++] = 0;
                                }
                            }
                        }
                    }
                    break;
                case 0x21:
                    // Read coin inputs
                    responseNeeded = true;
                    c++;    // Get next parameter byte
                    // If master requests 2 players and IO supports 2, p = 0. 
                    // Master should not normally request more than what IO supports
                    if(_info->featureParameters[featureLoc[coinInput]][0] - received.data[c] < 0){
                        // If master request more players than supported, send error
                        response.data[dataCount++] = JVS_REPORT_PARAMETERERROR;
                    } else {
                        temp = received.data[c];  // How many players to read

                        response.data[dataCount++] = JVS_REPORT_NORMAL; // Byte 0 report

                        for(int p = 0; p < temp; p++){
                            // Coin slot number
                            if(playerArray != nullptr){
                                response.data[dataCount++] = highByte(*(coinSlots + p));
                                response.data[dataCount++] = lowByte(*(coinSlots + p));
                            } else {
                                response.data[dataCount++] = 0;
                                response.data[dataCount++] = 0;
                            }
                        }
                    }
                    break;
                case 0x30:
                    // Decrease coin counter
                    responseNeeded = true;

                        index = received.data[c++];
                        temp = (received.data[c++] << 8);
                        temp &= (received.data[c]);
                        *(coinSlots + index) -= temp;
                        response.data[dataCount++] = JVS_REPORT_NORMAL;
                    break;
                case 0x35:
                    // Increase coin counter
                    responseNeeded = true;

                        index = received.data[c++];
                        temp = (received.data[c++] << 8);
                        temp &= received.data[c];
                        *(coinSlots + index) -= temp;
                        response.data[dataCount++] = JVS_REPORT_NORMAL;
                    break;
                case 0x32:
                    // GPO 1
                    responseNeeded = true;
                    temp = received.data[c++];
                    if((_info->featureParameters[featureLoc[gpOutput]][0] - temp) < 0){
                        // If master request more bytes than supported, send error
                        response.data[dataCount++] = JVS_REPORT_PARAMETERERROR;
                    } else {
                        for(int o = 0; o < temp; o++){
                            outputSlots[o] = received.data[c++];
                        }
                    }
                    break;
                // Default case is to send an unknown command response.
                // Any commands executed before this will still send
                default:
                case 0x02:
                    // Unknown
                    responseNeeded = true;
                    response.numBytes = 0;  // Data bytes
                    response.statusCode = JVS_STATUS_UNKNOWNCMD;
                    c+= received.numBytes-1;
                    errorCode = 12;
                    break;
            }
        }
        if(responseNeeded == true){
            response.numBytes = dataCount;
            write(response);
            #ifdef JVS_VERBOSE
            DBG_SERIAL.println(F("SEND"));
            DBG_SERIAL.println();
            #endif
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
    if(s == JVS_SENSE_ACTIVE){
        DBG_SERIAL.println("0V");
        digitalWrite(sensePin, HIGH);
    }   else {
        DBG_SERIAL.println("2.5V");
        digitalWrite(sensePin, LOW);
    }
    #else
    if(s == JVS_SENSE_ACTIVE){
        // Set sense output to active
        digitalWrite(sensePin, HIGH);
    } else {
        digitalWrite(sensePin, LOW);
    }
    #endif
}

// Calculate JVS packet checksum
uint8_t JVS::calculateSum(JVS_Frame &_f, bool send){
    uint32_t _s = 0;
    _s += _f.nodeID;
    _s += _f.numBytes;
    if((!isMaster && send) || (isMaster && !send)){
        _s += _f.statusCode;
    }

    for(int s = 0; s < _f.numBytes-1; s++){
        _s += _f.dataString[s];
    }
    _s = (_s % 256);
    return _s;
}

/* Master mode JVS commands */

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

    if(isMaster){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 3;
        out.data[0] = JVS_GENERICOUT1_CODE;
        out.data[1] = 1;
        out.data[2] = reverse(data);
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

    if(isMaster){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = num + 2;
        out.data[0] = JVS_GENERICOUT1_CODE;
        out.data[1] = num;
        for(int m = 0; m < num; m++){
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

    if(isMaster){
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
    
    if(isMaster){
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
    if(isMaster){
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
    if(isMaster){
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
    if(isMaster){
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
    if(isMaster && p > 0 && d > 0){
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
    if(isMaster && c > 0){
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
    if(isMaster && c > 0){
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
    if(isMaster && c > 0){
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
    if(isMaster && c > 0 && c < 250 && d != NULL){
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
    if(isMaster){
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
    if(isMaster && c > 0 && c < 250 && d != NULL){
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
    if(isMaster && c > 0){
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
    if(isMaster){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 1;
        out.data[0] = JVS_READKEY_CODE;
        write(out);
    }
}

void JVS::readSceenPos(uint8_t id, uint8_t c){
    // Request touch screen position data from ID. C = Channel index
    if(isMaster && c > 0){
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
    if(isMaster && b > 0){
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
    if(isMaster && c > 0){
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
    if(isMaster){
        JVS_Frame out;
        out.nodeID = id;
        out.numBytes = 1;
        out.data[0] = JVS_DATARETRY_CODE;
        write(out);
    }
}

void JVS::increaseCoin(uint8_t id, uint8_t s, uint16_t c){
    // Increment coin counter amount on IO. S = Which slot, C = Amount
    if(isMaster && s > 0 && c > 0){
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
    if(isMaster && s > 0 && c > 0){
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
    if(isMaster && s > 0 && c > 0){
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
    if(isMaster && s > 0 && c > 0){
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