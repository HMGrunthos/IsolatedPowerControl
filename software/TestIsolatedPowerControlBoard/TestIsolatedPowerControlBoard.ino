#include <Wire.h>

#define NEWTWIFREQ 1250

// IO expansion settings
#define MCP23008_ADDR ((byte)0b0100000)

// MCP23008 Register locations
#define IODIR ((byte)0x00)
#define IPOL ((byte)0x01)
#define GPINTEN ((byte)0x02)
#define DEFVAL ((byte)0x03)
#define INTCON ((byte)0x04)
#define IOCON ((byte)0x05)
#define GPPU ((byte)0x06)
#define INTF ((byte)0x07)
#define INTCAP ((byte)0x08)
#define GPIO ((byte)0x09)
#define OLAT ((byte)0x0A)

// IO settings
#define BUFFEROFF (byte)(0b00010000)
#define POWEROFF (byte)(0b00000001)

// ADC Definitions
#define MCP4716_ADDR ((byte)0b1100000)

// DAC Command definitioins
#define MCP47X6_CMD_MASK       0x1F
#define MCP47X6_CMD_VOLDAC     0x00
#define MCP47X6_CMD_VOLALL     0x40
#define MCP47X6_CMD_VOLCONFIG  0x80
#define MCP47X6_CMD_ALL        0x60

// DAC settings
#define MCP4716_FULLSCALE (0x3FF)
#define MCP4716_DEFAULTCFG (0b00100)

// #define DEBUGSTARTUP
// #define DEBUGSTATE

void setup() {
  Serial.begin(9600);
  Serial.println("Remote cooker control started.");

  initialiseCookerIO();
}

void loop() {
  static bool isOn = false;
  static bool isConnected = false;
  static uint16_t powerLevel = 0;
  static uint32_t nextStatusUpdate = millis() + 1000;
  static uint32_t nextcommandTime = millis() + 1000;
  
  while(Serial.available()) {
    int inChar = Serial.read();

    inChar = tolower(inChar);

    if(inChar == '+') {
      if(isOn) {
        if(powerLevel == 1023) {
          Serial.print("Power limited. ");
        } else {
          powerLevel += 31;
          nextcommandTime = 0;
          Serial.print("Power up. ");
        }
      } else {
        Serial.print("Not on. ");
      }
      nextStatusUpdate = 0;
    } else if(inChar == '-') {
      if(isOn) {
        if(powerLevel == 0) {
          Serial.print("Power limited. ");
        } else {
          powerLevel -= 31;
          nextcommandTime = 0;
          Serial.print("Power down. ");
        }
      } else {
        Serial.print("Not on. ");
      }
      nextStatusUpdate = 0;
    } else if(inChar == '1') {
      if(!isOn) {
        Serial.print("Turning on. ");
        isOn = true;
        powerLevel = 0;
        nextcommandTime = 0;
      } else {
        Serial.print("Already on. ");
      }
      nextStatusUpdate = 0;
    } else if(inChar == '0') {
      if(isOn) {
        Serial.print("Turning off. ");
        isOn = false;
        powerLevel = 0;
        nextcommandTime = 0;
      } else {
        Serial.print("Already off. ");
      }
      nextStatusUpdate = 0;
    } else if(inChar == 'c') {
      if(isConnected) {
        Serial.print("Disconnecting. ");
        disconnectRemoteControl();
        isConnected = false;
      } else {
        Serial.print("Connecting. ");
        connectRemoteControl();
        isConnected = true;
      }
      isOn = false;
      powerLevel = 0;
      nextStatusUpdate = 0;
    }
  }

  if(isConnected) {
    if(nextcommandTime < millis()) {
      nextcommandTime = millis() + 200;

      // Serial.println("Commanding.");
      setPowerLevel(powerLevel);
      if(isOn) {
        poweredOn();
      } else {
        poweredOff();
      }
    }
  }

  if(nextStatusUpdate < millis()) {
    nextStatusUpdate = millis() + 1500;
    if(isConnected) {
      Serial.print("Connected. ");
    } else {
      Serial.print("Disconnected. ");
    }
    if(isOn) {
      Serial.print("Power is on. ");
    } else {
      Serial.print("Power is off. ");
    }
    Serial.print("Current power level :");
    Serial.print(powerLevel);
    Serial.println(":");

    #ifdef DEBUGSTATE
      // Read the GPIO register value
      byte gpioVal;
      Wire.beginTransmission(MCP23008_ADDR);
        Wire.write(GPIO);
      Wire.endTransmission(false); // At lower clock rates the receiver doesn't like a repeated start here
      Wire.requestFrom(MCP23008_ADDR, (byte)1, (byte)true);
      gpioVal = Wire.read();
      Serial.print("GPIO Val :");
      Serial.println(gpioVal, BIN);
  
      // Read all the register values in the DAC
      byte dacRegVals[6];
      Wire.requestFrom(MCP4716_ADDR, (byte)6, (byte)true);
      for(byte idx = 0; idx < 6; idx++) {
        dacRegVals[idx] = Wire.read();
      }
      for(byte idx = 0; idx < 6; idx++) {
        Serial.print("Reg ");
        Serial.print(idx, HEX);
        Serial.print(", ");
        Serial.println(dacRegVals[idx], BIN);
      }
      Serial.println();
    #endif // DEBUGSTATE
  }
}

static byte cookerState;

void connectRemoteControl()
{
  Wire.beginTransmission(MCP4716_ADDR);
    Wire.write(MCP47X6_CMD_VOLDAC | ((byte)(MCP4716_FULLSCALE >> 6) & 0x0F)); // Since this writes the PD0 and PD1 bits it wakes the DAC back up from a sleeping state with an initial full scale output equivalent to 500W control setting
    Wire.write((byte)((MCP4716_FULLSCALE << 2) & 0xFC));
  Wire.endTransmission(true);

  // Set the latch bits to power up the opamp thus driving the power level but keep the cooker off
  cookerState = ~BUFFEROFF | POWEROFF;
  Wire.beginTransmission(MCP23008_ADDR);
    Wire.write(OLAT);
    Wire.write(cookerState);
  Wire.endTransmission(true);
}

void disconnectRemoteControl()
{
  // Set the latch bits to power down the opamp thus returning the power level control to the unit
  cookerState |= BUFFEROFF;
  Wire.beginTransmission(MCP23008_ADDR);
    Wire.write(OLAT);
    Wire.write(cookerState);
  Wire.endTransmission(true);

  // Turn off the DAC
  Wire.beginTransmission(MCP4716_ADDR);
    Wire.write(MCP47X6_CMD_VOLCONFIG | MCP4716_DEFAULTCFG); // This sets zero gain, shut down with 100k to ground and Vref is Vdd by default (in EEPROM)
  Wire.endTransmission(true);

  // Wait a bit for the unit to notice that the power level may have changed
  delay(250);
  
  // Set the latch bits to set the remote on/off switch off
  cookerState |= POWEROFF;
  Wire.beginTransmission(MCP23008_ADDR);
    Wire.write(OLAT);
    Wire.write(cookerState);
  Wire.endTransmission(true);
}

void poweredOn()
{
  // Set the latch bits to power up the cooker - assuming we're already connected
  cookerState &= ~POWEROFF;
  Wire.beginTransmission(MCP23008_ADDR);
    Wire.write(OLAT);
    Wire.write(cookerState);
  Wire.endTransmission(true);
}

void poweredOff()
{
  // Set the latch bits to power off the cooker - assuming we're already connected
  cookerState |= POWEROFF;
  Wire.beginTransmission(MCP23008_ADDR);
    Wire.write(OLAT);
    Wire.write(cookerState);
  Wire.endTransmission(true);
}

void setPowerLevel(uint16_t dacVal)
{
  dacVal = MCP4716_FULLSCALE - dacVal;
  Wire.beginTransmission(MCP4716_ADDR);
    Wire.write(MCP47X6_CMD_VOLDAC | ((byte)(dacVal >> 6) & 0x0F)); // Since this writes the PD0 and PD1 bits it wakes the DAC back up from a sleeping state
    Wire.write((byte)((dacVal << 2) & 0xFC));
  Wire.endTransmission(true);
}

void initialiseCookerIO()
{
  Wire.begin();

  // Initialize twi prescaler and bit rate
  TWSR |= (1<<TWPS0);
  TWSR |= (1<<TWPS1);
  TWBR = ((F_CPU / NEWTWIFREQ) - 16) / (2*64);

  #ifdef DEBUGSTARTUP
    byte regVals[11];
  
    Serial.println("Initial state...");
    
    // Read all the register values in the IO expander
    Wire.beginTransmission(MCP23008_ADDR);
      Wire.write(0);
    Wire.endTransmission(true); // At lower clock rates the receiver doesn't like a repeated start here
    Wire.requestFrom(MCP23008_ADDR, (byte)11, (byte)true);
    for(byte idx = 0; idx < 11; idx++) {
      regVals[idx] = Wire.read();
    }
    for(byte idx = 0; idx < 11; idx++) {
      Serial.print("RegIO ");
      Serial.print(idx, HEX);
      Serial.print(", ");
      Serial.println(regVals[idx], BIN);
    }
  
    // Read all the register values in the DAC
    Wire.requestFrom(MCP4716_ADDR, (byte)6, (byte)true);
    for(byte idx = 0; idx < 6; idx++) {
      regVals[idx] = Wire.read();
    }
    for(byte idx = 0; idx < 6; idx++) {
      Serial.print("RegDAC ");
      Serial.print(idx, HEX);
      Serial.print(", ");
      Serial.println(regVals[idx], BIN);
    }
    Serial.println();
  #endif // DEBUGSTARTUP

  // Set pull ups on unconnected pins
  Wire.beginTransmission(MCP23008_ADDR);
    Wire.write(GPPU);
    Wire.write(0b00001100);
  Wire.endTransmission(true);

  // Set the latch bits to turn the high side switch off and keep the buffer disabled
  cookerState = BUFFEROFF | POWEROFF;
  Wire.beginTransmission(MCP23008_ADDR);
    Wire.write(OLAT);
    Wire.write(cookerState);
  Wire.endTransmission(true);

  Wire.beginTransmission(MCP23008_ADDR);
    Wire.write(IODIR);
    Wire.write(0b11101110); // Drive the buffer signal and LED (high side switch)
  Wire.endTransmission(true);

/*
  // Initialise the DAC EEPROM - only need to do this once per board
  Wire.beginTransmission(MCP4716ADDR);
    // Wire.write(MCP47X6_CMD_ALL | 0b00000); // This sets zero gain, fully powered up and Vref is Vdd
    Wire.write(MCP47X6_CMD_ALL | MCP4716_DEFAULTCFG); // This sets zero gain, shut down with 100k to ground and Vref is Vdd by default (in EEPROM)
    Wire.write(0b11111111); // Power up value is high rail - shouldn't be used because we start up shut down. 500W is actually 5V
    Wire.write(0b11000000); // Power up value is high rail
  Wire.endTransmission(true);
*/

  // Initialise the DAC - Make sure it's disabled
  Wire.beginTransmission(MCP4716_ADDR);
    Wire.write(MCP47X6_CMD_VOLCONFIG | MCP4716_DEFAULTCFG); // This sets zero gain, shut down with 100k to ground and Vref is Vdd by default
  Wire.endTransmission(true);

  #ifdef DEBUGSTARTUP
    Serial.println("Post initialise...");
  
    // Read all the register values in the IO expander
    Wire.beginTransmission(MCP23008_ADDR);
      Wire.write(0);
    Wire.endTransmission(true); // At lower clock rates the receiver doesn't like a repeated start here
    Wire.requestFrom(MCP23008_ADDR, (byte)11, (byte)true);
    for(byte idx = 0; idx < 11; idx++) {
      regVals[idx] = Wire.read();
    }
    for(byte idx = 0; idx < 11; idx++) {
      Serial.print("RegIO ");
      Serial.print(idx, HEX);
      Serial.print(", ");
      Serial.println(regVals[idx], BIN);
    }
  
    // Read all the register values in the DAC
    Wire.requestFrom(MCP4716_ADDR, (byte)6, (byte)true);
    for(byte idx = 0; idx < 6; idx++) {
      regVals[idx] = Wire.read();
    }
    for(byte idx = 0; idx < 6; idx++) {
      Serial.print("RegDAC ");
      Serial.print(idx, HEX);
      Serial.print(", ");
      Serial.println(regVals[idx], BIN);
    }
    Serial.println();
  #endif // DEBUGSTARTUP
}

//  static uint16_t loopCnt = 0;
//
//  if((loopCnt & 0x1F) == 0) {
///*
//    // Read the GPIO register value
//    byte gpioVal;
//    Wire.beginTransmission(MCP23008ADDR);
//      Wire.write(GPIO);
//    Wire.endTransmission(false); // At lower clock rates the receiver doesn't like a repeated start here
//    Wire.requestFrom(MCP23008ADDR, (byte)1, (byte)true);
//    gpioVal = Wire.read();
//    Serial.print("GPIO Val :");
//    Serial.println(gpioVal, BIN);
//*/
//
//    // Serial.println("Sending LED state");
//    Wire.beginTransmission(MCP23008ADDR);
//      Wire.write(OLAT);
//      if(loopCnt & 0x20) {
//        //Wire.write(0b00000000 | BUFFEROFF); // Buffer disabled
//        Wire.write(0b00000000); // Buffer enabled
//        digitalWrite(13, LOW);
//      } else {
//        //Wire.write(0b00000001 | BUFFEROFF); // Buffer disabled
//        Wire.write(0b00000001); // Buffer enabled
//        digitalWrite(13, HIGH);
//      }
//    Wire.endTransmission(true);
//
//  } else {
//    // Set the volatile DAC value
//    uint16_t dacVal = loopCnt<<0;
//    Wire.beginTransmission(MCP4716ADDR);
//      Wire.write(MCP47X6_CMD_VOLDAC | ((byte)(dacVal >> 6) & 0x0F)); // Since this writes the PD0 and PD1 bits it wakes the DAC back up from a sleeping state
//      Wire.write((byte)((dacVal << 2) & 0xFC));
//      //Wire.write(MCP47X6_CMD_VOLDAC | ((byte)((0x3FF) >> 6) & 0x0F)); // FULL SCALE Since this writes the PD0 and PD1 bits it wakes the DAC back up from a sleeping state
//      //Wire.write((byte)(((0x3FF) << 2) & 0xFC)); // FULL SCALE
//    Wire.endTransmission(true);
//  }
//
//  delay(10);
//
//  loopCnt++;

/*
byte readRegs(byte regAddr, byte nBytes, byte *bOut)
{
  byte rVal;
  Wire.beginTransmission(MCP23008ADDR);
    Wire.write(regAddr);
  rVal = Wire.endTransmission(true); // At lower clock rates the receiver doesn't like a repeated start here
  Wire.requestFrom(MCP23008ADDR, nBytes, (byte)true);
  for(byte idx = 0; idx < nBytes; idx++) {
    bOut[idx] = Wire.read();
  }
  return rVal;
}
*/

/*
    // Read all the register values in the MCP23008
    byte regVals[11];
    readRegs(0, 11, (byte*)regVals);
    for(byte idx = 0; idx < 11; idx++) {
      Serial.print("Reg ");
      Serial.print(idx, HEX);
      Serial.print(", ");
      Serial.println(regVals[idx], BIN);
    }
    Serial.println();
*/

/*
    Serial.println();
    byte regVals[6];
    // Read all the register values in the DAC
    Wire.requestFrom(MCP4716ADDR, (byte)6, (byte)true);
    for(byte idx = 0; idx < 6; idx++) {
      regVals[idx] = Wire.read();
    }
    for(byte idx = 0; idx < 6; idx++) {
      Serial.print("Reg ");
      Serial.print(idx, HEX);
      Serial.print(", ");
      Serial.println(regVals[idx], BIN);
    }
    Serial.println();
*/

/*
    // Set the volatile DAC value to the half rail
    Wire.beginTransmission(MCP4716ADDR);
      Wire.write(MCP47X6_CMD_VOLDAC | 0b00000111); // Since this writes the PD0 and PD1 bits it wakes the DAC back up from a sleeping state
      Wire.write(0b11111100);
    Wire.endTransmission();
*/
