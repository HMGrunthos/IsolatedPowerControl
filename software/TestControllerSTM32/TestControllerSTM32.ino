#include <Wire.h>

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

#define MCP23008ADDR ((byte)0b0100000)

#define NEWTWIFREQ 1250

// DAC Command definitioins
#define MCP47X6_CMD_MASK       0x1F
#define MCP47X6_CMD_VOLDAC     0x00
#define MCP47X6_CMD_VOLALL     0x40
#define MCP47X6_CMD_VOLCONFIG  0x80
#define MCP47X6_CMD_ALL        0x60

#define MCP4716ADDR ((byte)0b1100000)

#define BUFFEROFF (byte)(0b00010000)

#define LEDOFF (byte)(0b00000001)

TwoWire secondI2CPort(2);

void setup() {
  secondI2CPort.begin();
  //i2c_set_clk_control(I2C2, 40);
  //i2c_set_trise(I2C2, 9);

  // initialize twi prescaler and bit rate
  //TWSR &= ~(1<<TWPS0);
  // TWSR |= (1<<TWPS0);
  // TWSR |= (1<<TWPS1);
  // TWBR = ((F_CPU / NEWTWIFREQ) - 16) / (2*64);

  // digitalWrite(SDA, 0); // Turn off internal pullups
  // digitalWrite(SCL, 0); // Turn off internal pullups

  // Set pull ups on unconnected pins
  secondI2CPort.beginTransmission(MCP23008ADDR);
    secondI2CPort.write(GPPU);
    secondI2CPort.write(0b00001100);
  secondI2CPort.endTransmission(true);

  // Set the latch bits to turn the high side switch off and keep the buffer disabled
  secondI2CPort.beginTransmission(MCP23008ADDR);
    secondI2CPort.write(OLAT);
    secondI2CPort.write(BUFFEROFF | LEDOFF);
  secondI2CPort.endTransmission(true);

  secondI2CPort.beginTransmission(MCP23008ADDR);
    secondI2CPort.write(IODIR);
    secondI2CPort.write(0b11101110); // Drive the buffer signal and LED (high side switch)
  secondI2CPort.endTransmission(true);

  // Initialise the DAC EEPROM
  secondI2CPort.beginTransmission(MCP4716ADDR);
    // secondI2CPort.write(MCP47X6_CMD_ALL | 0b00000); // This sets zero gain, fully powered up and Vref is Vdd
    secondI2CPort.write(MCP47X6_CMD_ALL | 0b00100); // This sets zero gain, shut down with 100k to ground and Vref is Vdd by defaul (in EEPROM)
    secondI2CPort.write(0b00000000); // Power up value is low rail - shouldn't be used because we start up shut down
    secondI2CPort.write(0b00000000); // Power up value is low rail
  secondI2CPort.endTransmission(true);

  Serial.begin(9600);
  Serial.println("Hello world.");

  pinMode(13, OUTPUT); // On board LED enabled
  digitalWrite(13, LOW); // Abd off

  secondI2CPort.beginTransmission(MCP23008ADDR); // Enable the output buffer but keep the LED off
    secondI2CPort.write(OLAT);
    secondI2CPort.write(~BUFFEROFF | LEDOFF);
  secondI2CPort.endTransmission(true);
}

void loop() {
  static uint16_t loopCnt = 0;

  if((loopCnt & 0x1F) == 0) {
/*
    // Read the GPIO register value
    byte gpioVal;
    secondI2CPort.beginTransmission(MCP23008ADDR);
      secondI2CPort.write(GPIO);
    secondI2CPort.endTransmission(false); // At lower clock rates the receiver doesn't like a repeated start here
    secondI2CPort.requestFrom(MCP23008ADDR, (byte)1, (byte)true);
    gpioVal = secondI2CPort.read();
    Serial.print("GPIO Val :");
    Serial.println(gpioVal, BIN);
*/

    // Serial.println("Sending LED state");
    secondI2CPort.beginTransmission(MCP23008ADDR);
      secondI2CPort.write(OLAT);
      if(loopCnt & 0x20) {
        //secondI2CPort.write(0b00000000 | BUFFEROFF); // Buffer disabled
        secondI2CPort.write(0b00000000); // Buffer enabled
        digitalWrite(13, LOW);
      } else {
        //secondI2CPort.write(0b00000001 | BUFFEROFF); // Buffer disabled
        secondI2CPort.write(0b00000001); // Buffer enabled
        digitalWrite(13, HIGH);
      }
    secondI2CPort.endTransmission(true);

  } else {
    // Set the volatile DAC value
    uint16_t dacVal = loopCnt<<0;
    secondI2CPort.beginTransmission(MCP4716ADDR);
      secondI2CPort.write(MCP47X6_CMD_VOLDAC | ((byte)(dacVal >> 6) & 0x0F)); // Since this writes the PD0 and PD1 bits it wakes the DAC back up from a sleeping state
      secondI2CPort.write((byte)((dacVal << 2) & 0xFC));
      //secondI2CPort.write(MCP47X6_CMD_VOLDAC | ((byte)((0x3FF) >> 6) & 0x0F)); // FULL SCALE Since this writes the PD0 and PD1 bits it wakes the DAC back up from a sleeping state
      //secondI2CPort.write((byte)(((0x3FF) << 2) & 0xFC)); // FULL SCALE
    secondI2CPort.endTransmission(true);
  }

  delay(10);

  loopCnt++;
}

byte readRegs(byte regAddr, byte nBytes, byte *bOut)
{
  byte rVal;
  secondI2CPort.beginTransmission(MCP23008ADDR);
    secondI2CPort.write(regAddr);
  rVal = secondI2CPort.endTransmission(true); // At lower clock rates the receiver doesn't like a repeated start here
  secondI2CPort.requestFrom(MCP23008ADDR, nBytes);
  for(byte idx = 0; idx < nBytes; idx++) {
    bOut[idx] = secondI2CPort.read();
  }
  return rVal;
}

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
    secondI2CPort.requestFrom(MCP4716ADDR, (byte)6, (byte)true);
    for(byte idx = 0; idx < 6; idx++) {
      regVals[idx] = secondI2CPort.read();
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
    secondI2CPort.beginTransmission(MCP4716ADDR);
      secondI2CPort.write(MCP47X6_CMD_VOLDAC | 0b00000111); // Since this writes the PD0 and PD1 bits it wakes the DAC back up from a sleeping state
      secondI2CPort.write(0b11111100);
    secondI2CPort.endTransmission();
*/
