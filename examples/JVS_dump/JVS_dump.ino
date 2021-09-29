// A quick and dirty tool used to sniff packets on the JVS bus

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(2, OUTPUT);
  digitalWrite(2,LOW);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  bool sense = digitalRead(3);
  static bool lastSense;
  static byte cmd = 0;
  static byte bytes = 0;
  bool breakLoop = 0;
  static byte id = 0;
  
  if(Serial1.available()){
    uint8_t in = 0;
    in = Serial1.read();
    //Serial.print(millis());
    //Serial.print(" - ");
    if(in == 0xE0){
      Serial.print(in, HEX);
      Serial.println(" - SOF");
      cmd = 1;
    } else {
      switch(cmd){
        case 1:
          Serial.print(in, HEX);
          Serial.println(" - ID");
          id = in;
          cmd++;
          break;
        case 2:
          Serial.print(in);
          Serial.println(" - # Bytes");
          bytes = in;
          cmd++;
          break;
        case 3:
          Serial.print(in, HEX);
          if(id != 0x00){
            switch(in){
              case 0xF0:
                Serial.println(" - Reset");
                break;
              case 0xF1:
                Serial.println(" - Set ID");
                break;
              case 0x10:
                Serial.println(" - Req. IO Name");
                break;
              case 0x15:
                Serial.println(" - Main board name");
                while(!breakLoop){
                  while(!Serial1.available());
                  char l = Serial1.read();
                  if(l == 0) breakLoop = true;
                  else Serial.print(l);
                }
                breakLoop = false;
                Serial.println();
                break;
              default:
                Serial.println();
                break;
            }
            cmd++;
          } else if (id == 0x00){
            // Status
            Serial.println(" - Status");
          }
          cmd++;
          break;
          case 4:
            // Data
            if (id == 0x00) {
              if(bytes > 40){
              Serial.println("IO board name");
              while(!breakLoop){
                  while(!Serial1.available());
                  char l = Serial1.read();
                  if(l == 0) breakLoop = true;
                  else Serial.print(l);
                }
                breakLoop = false;
                Serial.println();
            } else {
              Serial.println(in, HEX);
            } 
          } else {
              Serial.println(in, HEX);
            } 
            cmd = 0;
          break;
        default:
          Serial.print(in, HEX);
          Serial.println();
          break;
      }
    }
  }
  
  if(sense != lastSense){
    digitalWrite(4, sense);
    lastSense = sense;
    Serial.print("Sense: ");
    Serial.println(sense);
  }
}
