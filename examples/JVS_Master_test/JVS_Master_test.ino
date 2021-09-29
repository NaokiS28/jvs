#include <jvs.h>

byte nodes = 0;
JVS_Info ioBoard[2];  // Number of IO boards you expect to discover

#define JVS_SENSE_PIN 4
#define JVS_SENSE_IN_PIN 3
#define RTS_PIN 2

JVS_Info jvsInfo;

JVS jvs(Serial1, JVS_SENSE_PIN, RTS_PIN, JVS_SENSE_IN_PIN);

void setup() {
  Serial.begin(115200);
  // Wait for 32u4 USB connection
  while(!Serial);

  Serial.println("JVS Master test");
  bool type = MASTER_NODE;
  // Copy a custom Mainboard ID. Not nessecery
  strncpy(jvsInfo.mainID, "Master IO Test Program;NKS (c)2021;Ver 0.01", 100);
  

  // JVS IO board information
  jvs.setInfo(jvsInfo);
  
  // Begin JVS class as device speified above
  jvs.begin(type);
  jvs.sendReset();  // Always send a reset at boot up
  
  // Init device or master. Master runs through ID assigning
  if(type == MASTER_NODE){
    // Master node should wait for 1-5 seconds to allow IO boards to set up
    delay(2000);
    Serial.println("JVS init");
    nodes = jvs.initMaster();
    delay(100);               // Sometimes 100 millisecond delay is needed
    Serial.print("Found ");
    Serial.print(nodes);
    Serial.print(F(" IO Board"));
    if(nodes == 1) Serial.println("s");
    else Serial.println();
    bool skipIO = false;
    if(nodes){
      JVS_Frame request;
      for(int n = 0; n < nodes; n++){
        delay(40);        // Might not be needed
        skipIO = false;
        
        // Request IO Name
        jvs.ioIdentify(n+1);
        
        uint32_t timeOut = millis();
        while(!jvs.update()){
          if(millis() - timeOut >= 250){
            // If no response
            Serial.print("No response from ID: 0x");
            Serial.println(n, HEX);
            skipIO = true;
            break;
          }
        }
      
      if(!skipIO){
        // IO responded to name request

        // Read and store IO name
        jvs.read(request);
        strncpy(ioBoard[n].ident, &request.dataString[1], 100);
        Serial.print("IO Board: "); Serial.println(ioBoard[n].ident);
  
        // Get details
        jvs.requestVersions(n+1);
  
        // Wait for IO board response
        while(!jvs.update());
  
        // Read and store JVS information

        // IO Board supported features and parameters
        jvs.read(request);
        ioBoard[n].cmdRev = request.data[1];
        ioBoard[n].jvsRev = request.data[3];
        ioBoard[n].comRev = request.data[5];
        for(int b = 7; b < request.numBytes-1; b++){
          if(ioBoard[n].featureSupport[b] > 0){
            ioBoard[n].totalFeatures++;
            ioBoard[n].featureSupport[b] = (featureTypes)request.data[b++];
            ioBoard[n].featureParameters[b][0] = request.data[b++];
            ioBoard[n].featureParameters[b][1] = request.data[b++];
            ioBoard[n].featureParameters[b][2] = request.data[b];
          } else {
            break;
          }
        }

        // JVS versions
        Serial.print("CMD Ver: "); Serial.println(ioBoard[n].cmdRev, HEX);
        Serial.print("JVS Ver: "); Serial.println(ioBoard[n].jvsRev, HEX);
        Serial.print("Comm Ver: "); Serial.println(ioBoard[n].comRev, HEX);
        
        Serial.print(ioBoard[n].totalFeatures); Serial.println(" Features: ");
        for(int f = 0; f < ioBoard[n].totalFeatures; f++){
          switch(ioBoard[n].featureSupport[f]){
            case switchInput:
              Serial.print(ioBoard[n].featureParameters[f][0]); Serial.print(" Players with ");
              Serial.print(ioBoard[n].featureParameters[f][1]); Serial.println(" Switch inputs"); break;
            case coinInput:
              Serial.print(ioBoard[n].featureParameters[f][0]); Serial.println(" Coin slots"); break;
            case analogInput:
              Serial.print(ioBoard[n].featureParameters[f][0]); Serial.print(" Analog inputs");
              if(ioBoard[n].featureParameters[f][1] > 0) { 
                Serial.print(" at "); 
                Serial.print(ioBoard[n].featureParameters[f][1]); Serial.println("-Bit resolution"); 
              } else {
                Serial.println();
              }
              break;
            case gpOutput:
              Serial.print(ioBoard[n].featureParameters[f][0]); Serial.println(" Outputs"); break;
            default: 
              break;
          }
        }

        // Write main board ID to IO board. Most IO boards will likely responded with unsupported command
        jvs.writeMainID(n+1);
        }
      }
    } else {
      Serial.println(F("FATAL: No IO boards found. Cannot continue"));
      while(true);
    }
  } else {
    jvs.initDevice();
  }
}

void loop() {
  
}
