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

TwoWire secondI2CPort(2);

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
      secondI2CPort.beginTransmission(MCP23008_ADDR);
        secondI2CPort.write(GPIO);
      secondI2CPort.endTransmission(false); // At lower clock rates the receiver doesn't like a repeated start here
      secondI2CPort.requestFrom(MCP23008_ADDR, (byte)1, (byte)true);
      gpioVal = secondI2CPort.read();
      Serial.print("GPIO Val :");
      Serial.println(gpioVal, BIN);
  
      // Read all the register values in the DAC
      byte dacRegVals[6];
      secondI2CPort.requestFrom(MCP4716_ADDR, (byte)6, (byte)true);
      for(byte idx = 0; idx < 6; idx++) {
        dacRegVals[idx] = secondI2CPort.read();
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
  secondI2CPort.beginTransmission(MCP4716_ADDR);
    secondI2CPort.write(MCP47X6_CMD_VOLDAC | ((byte)(MCP4716_FULLSCALE >> 6) & 0x0F)); // Since this writes the PD0 and PD1 bits it wakes the DAC back up from a sleeping state with an initial full scale output equivalent to 500W control setting
    secondI2CPort.write((byte)((MCP4716_FULLSCALE << 2) & 0xFC));
  secondI2CPort.endTransmission(true);

  // Set the latch bits to power up the opamp thus driving the power level but keep the cooker off
  cookerState = ~BUFFEROFF | POWEROFF;
  secondI2CPort.beginTransmission(MCP23008_ADDR);
    secondI2CPort.write(OLAT);
    secondI2CPort.write(cookerState);
  secondI2CPort.endTransmission(true);
}

void disconnectRemoteControl()
{
  // Set the latch bits to power down the opamp thus returning the power level control to the unit
  cookerState |= BUFFEROFF;
  secondI2CPort.beginTransmission(MCP23008_ADDR);
    secondI2CPort.write(OLAT);
    secondI2CPort.write(cookerState);
  secondI2CPort.endTransmission(true);

  // Turn off the DAC
  secondI2CPort.beginTransmission(MCP4716_ADDR);
    secondI2CPort.write(MCP47X6_CMD_VOLCONFIG | MCP4716_DEFAULTCFG); // This sets zero gain, shut down with 100k to ground and Vref is Vdd by default (in EEPROM)
  secondI2CPort.endTransmission(true);

  // Wait a bit for the unit to notice that the power level may have changed
  delay(250);
  
  // Set the latch bits to set the remote on/off switch off
  cookerState |= POWEROFF;
  secondI2CPort.beginTransmission(MCP23008_ADDR);
    secondI2CPort.write(OLAT);
    secondI2CPort.write(cookerState);
  secondI2CPort.endTransmission(true);
}

void poweredOn()
{
  // Set the latch bits to power up the cooker - assuming we're already connected
  cookerState &= ~POWEROFF;
  secondI2CPort.beginTransmission(MCP23008_ADDR);
    secondI2CPort.write(OLAT);
    secondI2CPort.write(cookerState);
  secondI2CPort.endTransmission(true);
}

void poweredOff()
{
  // Set the latch bits to power off the cooker - assuming we're already connected
  cookerState |= POWEROFF;
  secondI2CPort.beginTransmission(MCP23008_ADDR);
    secondI2CPort.write(OLAT);
    secondI2CPort.write(cookerState);
  secondI2CPort.endTransmission(true);
}

void setPowerLevel(uint16_t dacVal)
{
  dacVal = MCP4716_FULLSCALE - dacVal;
  secondI2CPort.beginTransmission(MCP4716_ADDR);
    secondI2CPort.write(MCP47X6_CMD_VOLDAC | ((byte)(dacVal >> 6) & 0x0F)); // Since this writes the PD0 and PD1 bits it wakes the DAC back up from a sleeping state
    secondI2CPort.write((byte)((dacVal << 2) & 0xFC));
  secondI2CPort.endTransmission(true);
}

void initialiseCookerIO()
{
  secondI2CPort.begin();

  i2c_set_clk_control(I2C2, 3600);
  i2c_set_trise(I2C2, 41);

  #ifdef DEBUGSTARTUP
    byte regVals[11];
  
    Serial.println("Initial state...");
    
    // Read all the register values in the IO expander
    secondI2CPort.beginTransmission(MCP23008_ADDR);
      secondI2CPort.write(0);
    secondI2CPort.endTransmission(true); // At lower clock rates the receiver doesn't like a repeated start here
    secondI2CPort.requestFrom(MCP23008_ADDR, (byte)11, (byte)true);
    for(byte idx = 0; idx < 11; idx++) {
      regVals[idx] = secondI2CPort.read();
    }
    for(byte idx = 0; idx < 11; idx++) {
      Serial.print("RegIO ");
      Serial.print(idx, HEX);
      Serial.print(", ");
      Serial.println(regVals[idx], BIN);
    }
  
    // Read all the register values in the DAC
    secondI2CPort.requestFrom(MCP4716_ADDR, (byte)6, (byte)true);
    for(byte idx = 0; idx < 6; idx++) {
      regVals[idx] = secondI2CPort.read();
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
  secondI2CPort.beginTransmission(MCP23008_ADDR);
    secondI2CPort.write(GPPU);
    secondI2CPort.write(0b00001100);
  secondI2CPort.endTransmission(true);

  // Set the latch bits to turn the high side switch off and keep the buffer disabled
  cookerState = BUFFEROFF | POWEROFF;
  secondI2CPort.beginTransmission(MCP23008_ADDR);
    secondI2CPort.write(OLAT);
    secondI2CPort.write(cookerState);
  secondI2CPort.endTransmission(true);

  secondI2CPort.beginTransmission(MCP23008_ADDR);
    secondI2CPort.write(IODIR);
    secondI2CPort.write(0b11101110); // Drive the buffer signal and LED (high side switch)
  secondI2CPort.endTransmission(true);

/*
  // Initialise the DAC EEPROM - only need to do this once per board
  secondI2CPort.beginTransmission(MCP4716ADDR);
    // secondI2CPort.write(MCP47X6_CMD_ALL | 0b00000); // This sets zero gain, fully powered up and Vref is Vdd
    secondI2CPort.write(MCP47X6_CMD_ALL | MCP4716_DEFAULTCFG); // This sets zero gain, shut down with 100k to ground and Vref is Vdd by default (in EEPROM)
    secondI2CPort.write(0b11111111); // Power up value is high rail - shouldn't be used because we start up shut down. 500W is actually 5V
    secondI2CPort.write(0b11000000); // Power up value is high rail
  secondI2CPort.endTransmission(true);
*/

  // Initialise the DAC - Make sure it's disabled
  secondI2CPort.beginTransmission(MCP4716_ADDR);
    secondI2CPort.write(MCP47X6_CMD_VOLCONFIG | MCP4716_DEFAULTCFG); // This sets zero gain, shut down with 100k to ground and Vref is Vdd by default
  secondI2CPort.endTransmission(true);

  #ifdef DEBUGSTARTUP
    Serial.println("Post initialise...");
  
    // Read all the register values in the IO expander
    secondI2CPort.beginTransmission(MCP23008_ADDR);
      secondI2CPort.write(0);
    secondI2CPort.endTransmission(true); // At lower clock rates the receiver doesn't like a repeated start here
    secondI2CPort.requestFrom(MCP23008_ADDR, (byte)11, (byte)true);
    for(byte idx = 0; idx < 11; idx++) {
      regVals[idx] = secondI2CPort.read();
    }
    for(byte idx = 0; idx < 11; idx++) {
      Serial.print("RegIO ");
      Serial.print(idx, HEX);
      Serial.print(", ");
      Serial.println(regVals[idx], BIN);
    }
  
    // Read all the register values in the DAC
    secondI2CPort.requestFrom(MCP4716_ADDR, (byte)6, (byte)true);
    for(byte idx = 0; idx < 6; idx++) {
      regVals[idx] = secondI2CPort.read();
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
