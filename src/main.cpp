/* A simple TCO for Loconet 
 * based on the mrrwa library and the included examples 
 * The code in this main program is published under the GPL
 * All custom definitions should be in config.h
 */

#include <Arduino.h>              // for platformio in Visual Studio Code
#include <LocoNet.h>              // Loconet lib
#include <Bounce2.h>              // Debounce Atmega inputs
#include <config.h>               // All input / outputs definitioned in config.h
#ifdef MCP                        // Choice not fully implemented  
  #include <Wire.h>               // I2C library
  #include <Adafruit_MCP23017.h>  // MCP23017 library
#endif

#define LN_TX_PIN     7		// LN_RX_PIN hardcoded in lib, Atmega328 PB0, pin 

lnMsg  *LnPacket;          // pointer to a received LNet packet

const byte shift = SHIFTADDR;

const byte MegaInp[] = INATMEL;
const uint16_t MegaInpAdd[] = INATMELADD;
const byte MegaInpPos[] = INATMELPOS;
const int MegaInpAdd2[] = INATMELADD2;
const byte MegaInpPos2[] = INATMELPOS2;

const byte MegaOut[] = OUTATMEL;
const uint16_t MegaOutAdd[] = OUTATMELADD;
const byte MegaOutPos[] = OUTATMELPOS;

const byte MCPOut[] = OUTMCP;
const uint16_t MCPOutAdd[] = OUTMCPADD;
const byte MCPOutPos[] = OUTMCPPOS;

Bounce * buttons = new Bounce[sizeof(MegaInp)]; // one button obejct for each defined Atmega input

#ifdef MCP
  Adafruit_MCP23017 mcp;
#endif

void sendOPC_SW_REQ(int address, byte dir, byte on);  //functions defined at bottom, made known here
void notifySensor( uint16_t Address, uint8_t State );
void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction );
void notifySwitchReport( uint16_t Address, uint8_t Output, uint8_t Direction );
void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction );
void setLNTurnout(int address, byte dir);
void setLED(uint16_t Address, uint8_t Position);  

void setup() {
  LocoNet.init(LN_TX_PIN);                            // Initialisierung Loconet
  mcp.begin();                                        // Use default address 0

  for (byte i = 0; i < sizeof(MegaInp); i++) {        // Loop through all defined inputs on Atmega
    buttons[i].attach(MegaInp[i], INPUT_PULLUP);      // Setup the bounce instance for the current button
    buttons[i].interval(25);                          // interval in ms
  } 

  for (byte i = 0; i < sizeof(MegaOut); i++) {        // Loop through all defined outputs on Atmega
    pinMode(MegaOut[i], OUTPUT);
    digitalWrite(MegaOut[i], HIGH);
  }
 
  for (byte i = 0; i < sizeof(MCPOut); i++) {         // Loop through all defined outputs on MCP23017
    mcp.pinMode(MCPOut[i], OUTPUT);                   // use MCP specific pin functions
    mcp.digitalWrite(MCPOut[i], LOW);
  }
}

void loop() {
  // Check for any received LocoNet packets
  LnPacket = LocoNet.receive() ;
  if( LnPacket ) {
    LocoNet.processSwitchSensorMessage(LnPacket);
  }

// Loop through all Atmega input pins
  for (byte i = 0; i < sizeof(MegaInp); i++)  {
    buttons[i].update();                                  // Update the Bounce instance
    if (buttons[i].fell()) {                              // Input changed to LOW (connected to ground)?
      setLNTurnout(MegaInpAdd[i]-shift, MegaInpPos[i]);   // Send turnout command
      setLED(MegaInpAdd[i]-shift, MegaInpPos[i]);         // Call function to update LED
      if (MegaInpAdd2[i] != 0) {                          // If second address defiend for input, repeat above
        setLNTurnout(MegaInpAdd2[i]-shift, MegaInpPos2[i]); // Send turnout command
        setLED(MegaInpAdd2[i]-shift, MegaInpPos2[i]);       // Call function to update LED
      }
    }
  }
}                                                         // End of loop, outputs are handled in notifyXNetTrnt

// Callbacks from LocoNet.processSwitchSensorMessage()
// We tie into the ones connected to turnouts so we can capture anything
// that can change (or indicatea change to) a turnout's position.

void notifySensor( uint16_t Address, uint8_t State )
{ setLED(Address, State >> 4); }

void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction )
{ setLED(Address, Direction >> 5); }

void notifySwitchReport( uint16_t Address, uint8_t Output, uint8_t Direction )
{ setLED(Address, Direction >> 5); }

void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction )
{ setLED(Address, Direction >> 5); }

// Function to set LEDs / outputs
void setLED(uint16_t Address, uint8_t Position) {       
  for (byte i = 0; i < sizeof(MegaOut); i++) {        // Loop through all Atmega outputs
    if (Address == MegaOutAdd[i]-shift) {             // Output right address?
      digitalWrite(MegaOut[i], MegaOutPos[i]^Position);  // LED on/off dependend on pos 
    }
  }
  for (byte i = 0; i < sizeof(MCPOut); i++) {         // Loop through all MCP outputs
    if (Address == MCPOutAdd[i]-shift) {
      mcp.digitalWrite(MCPOut[i], MCPOutPos[i]^Position);
    }
  }
}

// Some turnout decoders (DS54?) can use solenoids, this code emulates the digitrax 
// throttles in toggling the "power" bit to cause a pulse
void setLNTurnout(int address, byte dir) {
    sendOPC_SW_REQ(address - 1, dir, 1);
    sendOPC_SW_REQ(address - 1, dir, 0);
}

// Construct a Loconet packet that requests a turnout to set/change its state
// Oder hier auch LocoNetClass::requestSwitch(uint16_t Address, uint8_t Output, uint8_t Direction)
void sendOPC_SW_REQ(int address, byte dir, byte on) { 
    lnMsg SendPacket ;
    
    int sw2 = 0x00;
    if (dir) { sw2 |= B00100000; }
    if (on) { sw2 |= B00010000; }
    sw2 |= (address >> 7) & 0x0F;
    
    SendPacket.data[ 0 ] = OPC_SW_REQ ;
    SendPacket.data[ 1 ] = address & 0x7F ;
    SendPacket.data[ 2 ] = sw2 ;
    
    LocoNet.send( &SendPacket );
}