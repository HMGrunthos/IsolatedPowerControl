#include <string.h>

#include <Wire.h>
#include <PID_v1.h>
#include <RingBuffer.h>
#include <Adafruit_MAX31865.h>
#include <M12BY02AA.h>

// Use software SPI: CS, DI, DO, CLK
// Adafruit_MAX31865 max = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 max32865[2] = {Adafruit_MAX31865(PA4), Adafruit_MAX31865(PA8)};
RingBuffer<uint16_t, 16> maxRTDReadings[2];

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
//#define RREF      430.0
// #define RREF      429.885 // First
#define RREF      430.111 // Second (long pt100 probe in ice bath measures 99.6808575203077, boiling = 137.663313719333)
// #define RREF      429.942 // Third (short spare pt100 in ice bath measures 100.481676612245)
                             //       (short mounted pt100 in ice bath measures 100.381262413061)
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
// #define RNOMINAL  100.0
#define RNOMINAL  99.6808575203077

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

#define POWERHYST 5

// #define DEBUGSTARTUP
// #define DEBUGSTATE

TwoWire secondI2CPort(2);

Vfd vfd(0);  //create new Vfd instance with display address 7

struct RTDMeasurement {
  float temp;
  uint8_t fault;
};

struct RTDEstimate {
  float xMeas;
  float xEst;
  float vEst;
  bool ready;
};

static struct CookerState {
  uint8_t ioLatchState;
  uint_fast8_t isOn;
  uint_fast8_t isConnected;
  uint16_t powerLevel;
  uint_fast32_t controlTimeout;
} cookerStatus = {.ioLatchState = BUFFEROFF | POWEROFF, .isOn = false, .isConnected = false, .powerLevel = 0, .controlTimeout = 0};

static double measuredTemp = 15;
static double outputDrive = 0;
static double targetTemp = 55;

//Specify the links and initial tuning parameters
static PID myPID(&measuredTemp, &outputDrive, &targetTemp, 80, 1.5, 75, P_ON_M, DIRECT); // P_ON_M specifies that Proportional on Measurement be used
                                                                                     // P_ON_E (Proportional on Error) is the default behavior

template<typename T> T SQR(T val) {return val*val;}

void setup() {
  Serial.begin(115200);
  Serial.println("Temperature controller.");
  
  max32865[0].begin(MAX31865_3WIRE, PA3);  // set to 2WIRE or 4WIRE as necessary
  // max32865[1].begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  
  max32865[0].setWires(MAX31865_3WIRE);
  max32865[0].enableBias(true);
  max32865[0].autoConvert(true);
  max32865[0].setFilterFreq(50);
  max32865[0].clearFault();

  max32865[0].readRTDReg();
  max32865[0].readRTDReg(); // Reading two samples will ensure that we're syncronised with the automatic conversions
  // We are now assured that we'll see a negative going edge on PA3
  attachInterrupt(digitalPinToInterrupt(PA3), maxConversion0, FALLING);

  Serial.println("Started temperature sensors.");

  initialiseCookerIO();

  Serial.println("Started cooker control.");
  
  Wire.begin();

  i2c_set_clk_control(I2C1, 450); // Ser VFD I2C frequency to 10kHz
  i2c_set_trise(I2C1, 0);

  Serial.println("Started VFD I2C port.");
  
  vfd.brightness = 3; // valid brightness values are 0 through 15
  vfd.init();
  vfd.clear();
  vfd.setLEDBits((~RED_BIT) & (~YELLOW_BIT) & ~(GREEN_BIT));

  myPID.SetSampleTime(200);
  myPID.SetOutputLimits(0, 1230);
  myPID.SetMode(MANUAL);

  Serial.println("Initialisation complete.");
}

void loop() {
  static uint_fast8_t selRTD = 0;
  static uint_fast32_t nextStatusUpdate = millis() + 1500;

  static RTDEstimate currentTemps[2] = {{0, 0, 0, false}, {0, 0, 0, false}};

  bool newReading;
  noInterrupts();
    newReading = !maxRTDReadings[0].isEmpty();
  interrupts();

  if(newReading) {
    uint_fast8_t rtdIdx = 0;

    uint16_t regVal;
    maxRTDReadings[rtdIdx].lockedPop(regVal);

    uint8_t faultVal;
    if(regVal & 0x0001) { // There's a fault
      noInterrupts();
        faultVal = max32865[rtdIdx].readFault(); // Get the vaule of the fault register
        max32865[rtdIdx].clearFault(); // And then clear it
      interrupts();
    } else {
      faultVal = 0;
    }

    struct RTDMeasurement rtdValue = getRTDTemperature(regVal >> 1, faultVal); // The right shift throws away the fault bit

    if(!rtdValue.fault) {
      if(!currentTemps[rtdIdx].ready) {
        currentTemps[rtdIdx].xMeas = rtdValue.temp;
        currentTemps[rtdIdx].xEst = rtdValue.temp;
        currentTemps[rtdIdx].vEst = 0;
        currentTemps[rtdIdx].ready = true;
      } else {
        const float fSample = 50.25;
        const float dt = 1/fSample;
        const float sysNoise = 0.3;
        const float measNoise = 0.3;
        const float gamma = SQR(sysNoise)/SQR(measNoise) * SQR(dt);
        const float r = (4 + gamma - sqrt(8*gamma + SQR(gamma)))/4;
        const float alpha = 1 - SQR(r);
        const float beta = 2*(2 - alpha) - 4*sqrt(1 - alpha);

        float xPred = currentTemps[rtdIdx].xEst + currentTemps[rtdIdx].vEst * dt;
        currentTemps[rtdIdx].xMeas = rtdValue.temp;
        float innov = currentTemps[rtdIdx].xMeas - xPred;
        currentTemps[rtdIdx].xEst = xPred + alpha*innov;
        currentTemps[rtdIdx].vEst += beta*innov / dt;

        // const float alpha = 0.117394; // 3dB at about 1Hz at a sample rate of 50.25Hz
        // currentTemps[rtdIdx] = alpha*rtdValue.temp + (1 - alpha)*currentTemps[rtdIdx];

        if(rtdIdx == selRTD) {
          myPID.SetMode(AUTOMATIC);
        }
      }

      if(rtdIdx == selRTD) {
        measuredTemp = currentTemps[rtdIdx].xEst;
      }
    } else { // There is a fault
      currentTemps[rtdIdx].ready = false; // If we get an error from the temperature probe then restart the filter
      if(rtdIdx == selRTD) {
        myPID.SetMode(MANUAL);
      }
    }
  }

  checkSerialCommands(&nextStatusUpdate);
  updateCookerState();
  updateStatusDisplay(&nextStatusUpdate, currentTemps, selRTD);
}

void checkSerialCommands(uint_fast32_t *nextStatusUpdate)
{
  while(Serial.available()) {
    int inChar = Serial.read();

    inChar = tolower(inChar);

    if(inChar == '+') {
      // cookerPowerUp();
      *nextStatusUpdate = 0;
    } else if(inChar == '-') {
      // cookerPowerDown();
      *nextStatusUpdate = 0;
    } else if(inChar == '1') {
      // cookerTurnOn();
      *nextStatusUpdate = 0;
    } else if(inChar == '0') {
      // cookerTurnOff();
      *nextStatusUpdate = 0;
    } else if(inChar == 'c') {
      connectRemoteControl();
      *nextStatusUpdate = 0;
    } else if(inChar == 'd') {
      disconnectRemoteControl();
      *nextStatusUpdate = 0;
    }
  }
}

void updateCookerState()
{  
  bool updatedPID = myPID.Compute();

  if(updatedPID) {
    cookerStatus.controlTimeout = millis() + 1000;

    if(cookerStatus.isConnected) {
      double onLevel = 207;
      if(!cookerStatus.isOn) {
        onLevel += POWERHYST;
      }
      // Turn PID output into a drive signal
      if(outputDrive >= onLevel) {
        static uint_fast8_t toggle;
        if(!cookerStatus.isOn) {
          poweredOn();
        }
        setPowerLevel(outputDrive - 207);
        if(toggle++ & 0x1) {
          if(cookerStatus.powerLevel > 816) { // 4 to 5
            vfd.setLEDBits(YELLOW_BIT);
          } else if(cookerStatus.powerLevel > 612) { // 3 to 4
            vfd.setLEDBits(GREEN_BIT | YELLOW_BIT);
          } else if(cookerStatus.powerLevel > 408) { // 2 to 3
            vfd.setLEDBits(GREEN_BIT);
          } else if(cookerStatus.powerLevel > 204) { // 1 to 2
            vfd.setLEDBits(GREEN_BIT | RED_BIT);
          } else { // 0 to 1
            vfd.setLEDBits(RED_BIT);
          }
        } else {
          vfd.setLEDBits(0); // disable all LEDs
        }
      } else {
        if(cookerStatus.isOn) {
          poweredOff();
        }
        vfd.setLEDBits(RED_BIT);
      }
    } else {
      vfd.setLEDBits(0);
    }
  } else if(cookerStatus.controlTimeout < millis()) {
    cookerStatus.controlTimeout = millis() + 5000;
    if(cookerStatus.isConnected) {
      setPowerLevel(0);
      poweredOff();
    }
  }
}

void updateStatusDisplay(uint_fast32_t *nextStatusUpdate, struct RTDEstimate *currentTemps, uint_fast8_t selRTD)
{
  if(*nextStatusUpdate < millis()) {
    *nextStatusUpdate = millis() + 1500;

    if(cookerStatus.isConnected) {
      Serial.print("C. ");
    } else {
      Serial.print("D. ");
    }
    if(cookerStatus.isOn) {
      Serial.print("P. On. ");
    } else {
      Serial.print("P Off. ");
    }
    Serial.print("Curr level:");
    Serial.print(cookerStatus.powerLevel);
    Serial.print(": PID:");
    Serial.print(outputDrive);
    Serial.print(": Temp0:");
    if(currentTemps[0].ready) {
      Serial.print(currentTemps[0].xEst);
      Serial.print("(");
      Serial.print(currentTemps[0].xMeas);
      Serial.print(")/");
      Serial.print(currentTemps[0].vEst);
    } else {
      Serial.print("FAIL");
    }
    Serial.print(": Temp1:");
    if(currentTemps[1].ready) {
      Serial.print(currentTemps[1].xEst);
      Serial.print("(");
      Serial.print(currentTemps[1].xMeas);
      Serial.print(")/");
      Serial.print(currentTemps[1].vEst);
    } else {
      Serial.print("FAIL");
    }
    Serial.print(": T:");
    Serial.print(millis()/(float)1000);
    // Serial.print(": Loop period:");
    // Serial.print((millis() - loopStart) / (float)loopCount);
    Serial.println(":");

    if(currentTemps[selRTD].ready) {
      char pString[12];
      static char lastString[12] = "";

      sprintf(pString, "Temp%i %6.2f", 1, currentTemps[selRTD].xEst);

      uint_fast8_t firstDiff;
      for(firstDiff = 0; (firstDiff < 12) && (pString[firstDiff] == lastString[firstDiff]); firstDiff++);

      if(firstDiff != 12) {
        vfd.sendMessage(pString + firstDiff, firstDiff, 12-firstDiff); // send a message, 12 characters long
      }
      
      strcpy(lastString, pString);
    } else {
      char pString[12];
      sprintf(pString, "Temp%i  fault", 1);
      vfd.sendMessage(pString, 12);
    }

    #ifdef DEBUGSTATE
      // Read the GPIO register value
      byte gpioVal;
      secondI2CPort.beginTransmission(MCP23008_ADDR);
        secondI2CPort.write(GPIO);
      secondI2CPort.endTransmission(false); // At lower clock rates the receiver doesn't like a repeated start here
      secondI2CPort.requestFrom(MCP23008_ADDR, (byte)1);
      gpioVal = secondI2CPort.read();
      Serial.print("GPIO Val :");
      Serial.println(gpioVal, BIN);
  
      // Read all the register values in the DAC
      byte dacRegVals[6];++
      secondI2CPort.requestFrom(MCP4716_ADDR, (byte)6);
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

void maxConversion0()
{
  uint16_t rtdVal = max32865[0].readRTDReg();
  maxRTDReadings[0].push(rtdVal);
}

void initialiseCookerIO()
{
  cookerStatus.ioLatchState = BUFFEROFF | POWEROFF;
  cookerStatus.isOn = false;
  cookerStatus.isConnected = false;
  cookerStatus.powerLevel = 0;
  
  secondI2CPort.begin();

  i2c_set_clk_control(I2C2, 3600);
  i2c_set_trise(I2C2, 63);

  //gpio_set_mode(GPIOB, 11, GPIO_AF_OUTPUT_PP);
  //gpio_set_mode(GPIOB, 10, GPIO_AF_OUTPUT_PP);

  #ifdef DEBUGSTARTUP
    byte regVals[11];
  
    Serial.println("Initial state...");
    
    // Read all the register values in the IO expander
    secondI2CPort.beginTransmission(MCP23008_ADDR);
      secondI2CPort.write(0);
    secondI2CPort.endTransmission(true); // At lower clock rates the receiver doesn't like a repeated start here
    secondI2CPort.requestFrom(MCP23008_ADDR, (byte)11);
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
    secondI2CPort.requestFrom(MCP4716_ADDR, (byte)6);
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
  cookerStatus.ioLatchState = BUFFEROFF | POWEROFF;
  secondI2CPort.beginTransmission(MCP23008_ADDR);
    secondI2CPort.write(OLAT);
    secondI2CPort.write(cookerStatus.ioLatchState);
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
    secondI2CPort.requestFrom(MCP23008_ADDR, (byte)11);
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
    secondI2CPort.requestFrom(MCP4716_ADDR, (byte)6);
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

void connectRemoteControl()
{
  if(!cookerStatus.isConnected) {
    Serial.println("Connecting..");
  } else {
    Serial.println("Already connected..");
    return;
  }

  secondI2CPort.beginTransmission(MCP4716_ADDR);
    secondI2CPort.write(MCP47X6_CMD_VOLDAC | ((byte)(MCP4716_FULLSCALE >> 6) & 0x0F)); // Since this writes the PD0 and PD1 bits it wakes the DAC back up from a sleeping state with an initial full scale output equivalent to 500W control setting
    secondI2CPort.write((byte)((MCP4716_FULLSCALE << 2) & 0xFC));
  secondI2CPort.endTransmission(true);

  // Set the latch bits to power up the opamp thus driving the power level but keep the cooker off
  cookerStatus.ioLatchState = ~BUFFEROFF | POWEROFF;
  secondI2CPort.beginTransmission(MCP23008_ADDR);
    secondI2CPort.write(OLAT);
    secondI2CPort.write(cookerStatus.ioLatchState);
  secondI2CPort.endTransmission(true);

  cookerStatus.isOn = false;
  cookerStatus.powerLevel = 0;
  cookerStatus.isConnected = true;
}

void disconnectRemoteControl()
{
  if(cookerStatus.isConnected) {
    Serial.print("Disconnecting. ");
  } else {
    Serial.println("Not connected..");
    return;
  }
 
  // Set the latch bits to power down the opamp thus returning the power level control to the unit
  cookerStatus.ioLatchState |= BUFFEROFF;
  secondI2CPort.beginTransmission(MCP23008_ADDR);
    secondI2CPort.write(OLAT);
    secondI2CPort.write(cookerStatus.ioLatchState);
  secondI2CPort.endTransmission(true);

  // Turn off the DAC
  secondI2CPort.beginTransmission(MCP4716_ADDR);
    secondI2CPort.write(MCP47X6_CMD_VOLCONFIG | MCP4716_DEFAULTCFG); // This sets zero gain, shut down with 100k to ground and Vref is Vdd by default (in EEPROM)
  secondI2CPort.endTransmission(true);

  // Wait a bit for the unit to notice that the power level may have changed
  delay(250);
  
  // Set the latch bits to set the remote on/off switch off
  cookerStatus.ioLatchState |= POWEROFF;
  secondI2CPort.beginTransmission(MCP23008_ADDR);
    secondI2CPort.write(OLAT);
    secondI2CPort.write(cookerStatus.ioLatchState);
  secondI2CPort.endTransmission(true);

  cookerStatus.isOn = false;
  cookerStatus.powerLevel = 0;
  cookerStatus.isConnected = false;
}

void setPowerLevel(uint16_t powerLevel)
{
  cookerStatus.powerLevel = powerLevel;
  uint16_t dacVal = MCP4716_FULLSCALE - powerLevel;
  secondI2CPort.beginTransmission(MCP4716_ADDR);
    secondI2CPort.write(MCP47X6_CMD_VOLDAC | ((byte)(dacVal >> 6) & 0x0F)); // Since this writes the PD0 and PD1 bits it wakes the DAC back up from a sleeping state
    secondI2CPort.write((byte)((dacVal << 2) & 0xFC));
  secondI2CPort.endTransmission(true);
}

void poweredOn()
{
  // Set the latch bits to power up the cooker - assuming we're already connected
  cookerStatus.ioLatchState &= ~POWEROFF;
  secondI2CPort.beginTransmission(MCP23008_ADDR);
    secondI2CPort.write(OLAT);
    secondI2CPort.write(cookerStatus.ioLatchState);
  secondI2CPort.endTransmission(true);
  cookerStatus.isOn = true;
}

void poweredOff()
{
  // Set the latch bits to power off the cooker - assuming we're already connected
  cookerStatus.ioLatchState |= POWEROFF;
  secondI2CPort.beginTransmission(MCP23008_ADDR);
    secondI2CPort.write(OLAT);
    secondI2CPort.write(cookerStatus.ioLatchState);
  secondI2CPort.endTransmission(true);
  cookerStatus.isOn = false;
}

struct RTDMeasurement getRTDTemperature(uint16_t rtdReg, uint8_t fault)
{
  struct RTDMeasurement rtdValue;

  rtdValue.temp = Adafruit_MAX31865::temperature(rtdReg, RNOMINAL, RREF);

  // Decode and print any faults
  rtdValue.fault = fault;
  if (rtdValue.fault) {
    Serial.print("\tFault 0x"); Serial.println(rtdValue.fault, HEX);
    if (rtdValue.fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("\tRTD High Threshold"); 
    }
    if (rtdValue.fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("\tRTD Low Threshold"); 
    }
    if (rtdValue.fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("\tREFIN- > 0.85 x Bias"); 
    }
    if (rtdValue.fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("\tREFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (rtdValue.fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("\tRTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (rtdValue.fault & MAX31865_FAULT_OVUV) {
      Serial.println("\tUnder/Over voltage"); 
    }
  }

  return rtdValue;
}
