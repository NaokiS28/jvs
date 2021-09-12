#include <Arduino.h>
//#include <Joystick.h>

#include <jvs.h>


#define JVS_SENSE_PIN 4
#define RTS_PIN 2

JVS jvs(Serial1, JVS_SENSE_PIN, RTS_PIN);
JVS_Info jvsInfo;

void setup() {
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
}

void loop() {

  // JVS update
  if(jvs.update()){
    jvs.runCommand();
  }
}
