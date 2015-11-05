/*
 Calculator-powered DCC by Dr. C. Mitchell
 Based on example from d. bodnar  9-23-2014
 Uses CmdArduino library for DCC output
 Uses ArTICL library for calculator communication
 */

#include <EEPROM.h>
#include <CBL2.h>
#include <TIVar.h>

#include <DCCPacket.h>
#include <DCCPacketQueue.h>
#include <DCCPacketScheduler.h>

int LED = 13; // LED to blink when DCC packets are sent in loop
int Enable_PIN = 8; //low to enable DCC, high to stop

DCCPacketScheduler dps;
//unsigned int analog_value=0;
char speed_byte = 0;
char old_speed = 0;
bool force_update = false;
char temp;
byte Fx = 0;
byte DCCAddress = 3;
bool enable = true;

#include <Wire.h>

byte fn0to4 = 0;  // DCC function variables
byte fn5to8 = 0;
byte fn9to12 = 0;

unsigned long previousMillis = 0; // last time update
const long interval = 2000; // interval at which to do refresh (milliseconds)

CBL2* cbl;
const int lineRed = DEFAULT_TIP;
const int lineWhite = DEFAULT_RING;

// Lists are 2 + (9 * dimension) bytes,
// so incidentally a 255-byte max data length
// limits this demo's lists to 28 elements.
#define MAXDATALEN 255

uint8_t header[16];
uint8_t data[MAXDATALEN];

void setup() {
  pinMode(LED, OUTPUT);
  DCCAddress = EEPROM.read(0);  
  if(DCCAddress >=100){  // set default as 3 if not in proper range (0-99)
    DCCAddress = 3;
  }
  pinMode(Enable_PIN, OUTPUT); 
  enable = true;

  Serial.begin(115200);
  dps.setup();
  dps.setFunctions0to4(DCCAddress, DCC_SHORT_ADDRESS, B00000000); //clear functions
  dps.setFunctions5to8(DCCAddress, DCC_SHORT_ADDRESS, B00000000);    
  dps.setFunctions9to12(DCCAddress, DCC_SHORT_ADDRESS, B00000000);    

  cbl = new CBL2(lineRed, lineWhite);
  cbl->resetLines();
  //cbl->setVerbosity(true, &Serial);			// Comment this in for verbose message information
  
  // The following registers buffers for exchanging data, the maximum
  // allowed data length, and functions to call on Get() and Send().
  cbl->setupCallbacks(header, data, MAXDATALEN,
                      onGetAsCBL2, onSendAsCBL2);
    fn0to4 = 1;
    speed_byte = 1;
}

void loop() {
  //this section sends DCC updates every 2 seconds (interval)
  // not sure if it is necessary but the functions are slow to respond
  // at times - may be due to the DCC library setting priorities as the
  // speed controls always work 
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval || force_update) {
    previousMillis = currentMillis;  
    dps.setSpeed128(DCCAddress, DCC_SHORT_ADDRESS, speed_byte);
    dps.setFunctions(DCCAddress, DCC_SHORT_ADDRESS, fn0to4, fn5to8, fn9to12);
    digitalWrite(LED, !digitalRead(LED));
    force_update = false;
  }
  
  if (enable) {
    digitalWrite(Enable_PIN, LOW);// HIGH = disable DCC <- Why do we need to do this every loop?
  }
  
  // Get input from calculator
  int rval = 0;
  rval = cbl->eventLoopTick();
  if (rval && rval != ERR_READ_TIMEOUT) {
    Serial.print("Failed to run eventLoopTick: code ");
    Serial.println(rval);
  }

  //Functions will not work without this to limit speed command to only new speeds
  if (speed_byte != old_speed) {
    speed_byte = constrain(speed_byte, -127, 127);
    dps.setSpeed128(DCCAddress,DCC_SHORT_ADDRESS,speed_byte);
    old_speed = speed_byte;
  } 
  dps.update();
}  //END LOOP

// Callback when the CBL2 class has successfully received a variable
// from the attached calculator.
int onGetAsCBL2(uint8_t type, enum Endpoint model, int datalen) {
  Serial.print("Got variable of type ");
  Serial.print(type);
  Serial.print(" from endpoint of type ");
  Serial.println((int)model);
  
  // We only want to handle lists.
  if (type != VarTypes82::VarRList && type != 93)
    return -1;

  // Turn the LEDs on or off
  uint16_t list_len = TIVar::sizeWordToInt(&(data[0]));		// Convert 2-byte size word to int
  // List possibilities:
  //   [1, CV, CV value] : Programming
  //   [2, Feature, 0/1] : Turn function on/off
  //   [3, speed] :        Set speed
  //   [0]:                Emergency disable
  if (list_len >= 1) {
    int command = (int)TIVar::realToFloat8x(&data[2], model);	// First list element starts after 2-byte size word
    switch(command) {
      case 0: {
        enable = false;
        digitalWrite(Enable_PIN, HIGH);// HIGH = disable DCC
        Serial.println("EMERGENCY STOP");
        break;
      }
      case 1: {
        int CV = (int)TIVar::realToFloat8x(&data[2 + (1 * 9)], model);
        int val = (int)TIVar::realToFloat8x(&data[2 + (2 * 9)], model);
        Serial.print("Setting CV ");
        Serial.print(CV);
        Serial.print(" to ");
        Serial.println(val);
        dps.opsProgramCV(DCCAddress, DCC_SHORT_ADDRESS, CV, val);
        break;
      }
      case 2: {
        int func = (int)TIVar::realToFloat8x(&data[2 + (1 * 9)], model);
        bool val = (int)TIVar::realToFloat8x(&data[2 + (2 * 9)], model);
        if (func >= 1 && func <= 4) {
          fn0to4  = (fn0to4 & ~(1 << (func - 1))) | (val << (func - 1));
        } else if (func >= 5 && func <= 8) {
          fn5to8  = (fn5to8 & ~(1 << (func - 5))) | (val << (func - 5));
        } else if (func >= 9 && func <= 12) {
          fn9to12 = (fn9to12 & ~(1 << (func - 9))) | (val << (func - 9));
        }
        Serial.print("Functions set to ");
        Serial.print(fn0to4);
        Serial.print(", ");
        Serial.print(fn5to8);
        Serial.print(", ");
        Serial.println(fn9to12);
        force_update = true;
        break;
      }
      case 3: {
        speed_byte = (int)TIVar::realToFloat8x(&data[2 + (1 * 9)], model);
        Serial.print("Speed set to ");
        Serial.println((int)speed_byte);
        force_update = true;
        break;
      }
    }
  }
  return 0;
}

// Callback when the CBL2 class notices the attached calculator
// wants to start a Get() exchange. The CBL2 class needs to get
// any data to send before continuing the exchange.
int onSendAsCBL2(uint8_t type, enum Endpoint model, int* headerlen,
                 int* datalen, data_callback* data_callback)
{
  Serial.print("Got request for variable of type ");
  Serial.print(type);
  Serial.print(" from endpoint of type ");
  Serial.println((int)model);
  return -1;		// -1 means we have no data to send.
}
