//#include <Arduino.h>
//#include <Joystick.h>

#include <jvs.h>
#include <MegaPad.h>
#include <SuperPad.h>

#define outputA 39
#define outputB 41

#define JVS_SENSE_PIN 4
#define RTS_PIN 2

JVS jvs(Serial1, JVS_SENSE_PIN, RTS_PIN);
JVS_Info jvsInfo;

byte padInputs[2];
byte cabControls;
byte coinSlotStatus[2];
byte outputArray[2];
uint16_t  coinCount[2] = {0, 128};

// MD
MegaPad pad(23, 25, 27, 29, 31, 33, 35);

//Snes
SuperPad spad(22, 24, 26);

void setup() {
  Serial.begin(115200);

  pinMode(outputA, OUTPUT);
  pinMode(outputB, OUTPUT);

  memcpy(jvsInfo.ident, "MD/SNES to JVS;Naoki's Retro Corner;VER:0.1 Beta", 100);
  jvsInfo.totalFeatures = 3;
  jvsInfo.featureSupport[0] = switchInput;
  jvsInfo.featureSupport[1] = coinInput;
  jvsInfo.featureSupport[2] = gpOutput;
  jvsInfo.featureParameters[0][0] = 1;
  jvsInfo.featureParameters[0][1] = 10;
  jvsInfo.featureParameters[1][0] = 1;
  jvsInfo.featureParameters[2][0] = 8;


  jvs.setPlayerArray(padInputs);
  jvs.setCoinArray(coinCount);
  jvs.setOutputarray(outputArray);
  jvs.setCoinMechArray(coinSlotStatus);

  // Valid types are MASTER_NODE or DEVICE_NODE
  bool type = DEVICE_NODE;

  // JVS IO board information
  jvs.setInfo(jvsInfo);

  // Begin JVS class as device speified above
  jvs.begin(type);

  // Init device or master. Master runs through ID assigning
  if(type == MASTER_NODE){
    // Master node should wait for 5 seconds to allow IO boards to set up
    delay(5000);
    jvs.initMaster();
  } else {
    jvs.initDevice();
  }
  // Serial.println(pad.type());
}

void loop() {
  if(pad.update() || spad.update()){
    // JVS MSBFIRST 7: Start, 6: Service, 5: Up, 4: Down, 3: Left, 2: Right, 1: Button 1, 0: Button 2
    //  |- 2 MSBFIRST 7: Button 3, 6: Button 4, 5: Button 5, 4: Button 6, 3: Button 7, 2; Button 8, 0-1: Not used 
    // MD MSBFIRST: [0] = SCBARLDU, [1] = xxxxMZYX
    byte in[2];
    in[0] = pad.read(0);
    in[1] = pad.read(1);
    byte out[2];
    for(int b = 0; b < 8; b++){
      bitWrite(out[0], b, bitRead(in[0], 7 - b));
      bitWrite(out[1], b, bitRead(in[1], 7 - b));
    }
    // Shift buttons
    out[0] = (out[0] >> 2);
    out[1] = (out[1] << 5);                 // Move XYZ (MD6 only)
    bitWrite(out[0], 7, bitRead(in[0], 7)); // Move start
    bitWrite(out[0], 6, bitRead(in[1], 3)); // Move mode (service, MD6 only)
    bitWrite(out[1], 7, bitRead(in[0], 6)); // Move C

    cabControls = spad.read();
    if(bitRead(cabControls, 4)) coinCount[0]++; // Coin
    bitWrite(padInputs[0], 6, bitRead(cabControls, 5)); // P1 Service
    jvs.writePanelSw(cabControls);
    padInputs[0] = out[0];
    padInputs[1] = out[1];
  }

  // JVS update
  if(jvs.update()){
    jvs.runCommand();
    digitalWrite(outputA, bitRead(outputArray[0], 0));
    digitalWrite(outputB, bitRead(outputArray[0], 1));
  }
}
