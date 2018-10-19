/*************************************************** 
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>

#include <Adafruit_MAX31865.h>
#include <M12BY02AA.h>

// Use software SPI: CS, DI, DO, CLK
// Adafruit_MAX31865 max = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 max32865[2] = {Adafruit_MAX31865(PA4), Adafruit_MAX31865(PA8)};

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
//#define RREF      430.0
// #define RREF      429.885 // First
// #define RREF      430.111 // Second (long pt100 probe in ice bath measures 99.6808575203077, boiling = 137.663313719333)
#define RREF      429.942 // Third (short spare pt100 in ice bath measures 100.481676612245)
                          //       (short mounted pt100 in ice bath measures 100.381262413061)
//
The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

Vfd vfd(0);  //create new Vfd instance with display address 7

TwoWire secondI2CPort(2);

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");

  max32865[0].begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  max32865[1].begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

  Wire.begin();
  
  vfd.brightness = 3; // valid brightness values are 0 through 15
}


void loop() {
  static uint32_t nextSwitchTime = millis() + 1000;
  static uint_fast8_t selRTD = 0;

  if(nextSwitchTime < millis()) {
    nextSwitchTime = millis() + 2000;
    // selRTD = (selRTD + 1) & 0x1;
    selRTD = 0;
  }
  
  for(uint_fast8_t rtdIdx = 0; rtdIdx < 1; rtdIdx++) {
    uint16_t rtd = max32865[rtdIdx].readRTD();
  
    Serial.print("RTD[");
    Serial.print(rtdIdx);
    Serial.print("] value: "); Serial.println(rtd);
    float ratio = rtd;
    ratio /= 32768;
    Serial.print("\tRatio = "); Serial.println(ratio,8);
    Serial.print("\tResistance = "); Serial.println(RREF*ratio,8);
    Serial.print("\tTemperature = "); Serial.println(max32865[rtdIdx].temperature(RNOMINAL, RREF));

    if(rtdIdx == selRTD) {
      char pString[12];
      sprintf(pString, "Temp%i %6.2f", rtdIdx+1, max32865[rtdIdx].temperature(RNOMINAL, RREF));
      vfd.sendMessage(pString, 12); // send a message, 12 characters long
      //Serial.println(pString);
      //Serial.println(strlen(pString));
    }
  
    // Check and print any faults
    uint8_t fault = max32865[rtdIdx].readFault();
    if (fault) {
      Serial.print("\tFault 0x"); Serial.println(fault, HEX);
      if (fault & MAX31865_FAULT_HIGHTHRESH) {
        Serial.println("\tRTD High Threshold"); 
      }
      if (fault & MAX31865_FAULT_LOWTHRESH) {
        Serial.println("\tRTD Low Threshold"); 
      }
      if (fault & MAX31865_FAULT_REFINLOW) {
        Serial.println("\tREFIN- > 0.85 x Bias"); 
      }
      if (fault & MAX31865_FAULT_REFINHIGH) {
        Serial.println("\tREFIN- < 0.85 x Bias - FORCE- open"); 
      }
      if (fault & MAX31865_FAULT_RTDINLOW) {
        Serial.println("\tRTDIN- < 0.85 x Bias - FORCE- open"); 
      }
      if (fault & MAX31865_FAULT_OVUV) {
        Serial.println("\tUnder/Over voltage"); 
      }
      max32865[rtdIdx].clearFault();
    }

    Serial.println();
  }

  delay(1000);
}
