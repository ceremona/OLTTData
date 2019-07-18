/*
 * Cere Davis
 * @file OLTTLogger.ino
 * 
 * Logging OLTT data values from various sensors
 * + control logic
 * 
 * 12 July 2019
 *  
 */

#include "DFRobot_I2CMultiplexer.h"
#include <OneWire.h>
#include <DallasTemperature.h> 
#include <Wire.h>
#include "Adafruit_MPRLS.h"
int ledState = LOW;

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
//Arduino pins
// OneWire Data wire is plugged into port 14 on the Arduino
#define ONE_WIRE_BUS 22
#define PRESSURE_SCA 18 
#define PRESSURE_SCL 19
//
DFRobot_I2CMultiplexer I2CMulti(0x70);
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWireBus(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature oneWireSensors(&oneWireBus);
DeviceAddress addr;

int I2CInit(void) {
 if (!mpr.begin(MPRLS_DEFAULT_ADDR, &Wire)) {
//  Note: use &Wire2 constructor argument for SDA2/SCL2, etc
//   Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    delay(10);
    return 1;
  } else {
    //Serial.println("Found MPRLS sensor");
    return 0;
  }
}

void blink(void) {
  if (ledState == LOW) {
      ledState = HIGH;
  } else {
      ledState = LOW;
  }
  digitalWrite(LED_BUILTIN, ledState);
}

void setup(void)
{
  // start serial port
  Serial.begin(9600);
  // MPRLS pressure sensor
  pinMode(PRESSURE_SCA,INPUT);
  pinMode(PRESSURE_SCL,INPUT);
  // One Wire Thermocouple Sensors 
  pinMode(ONE_WIRE_BUS,INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);  

  if(I2CInit()) Serial.println("Failed to communicate with MPRLS Pressure port sensor, check wiring?");
  // Start up the OneWire library
//  I2CInit();
  delay(20);
  oneWireSensors.begin();
  Serial.println("Pressure1, Pressure2, Pressure3, Temp1, Temp2, Temp3, Temp4, Temp5, Temp6");
}
  
void loop(void)
{ 
  I2CMulti.selectPort(1); 
  float pressure1_kPa = mpr.readPressure()/10;
  delay(20);
  I2CMulti.selectPort(2); 
  float pressure2_kPa = mpr.readPressure()/10;
  delay(20);
  I2CMulti.selectPort(5); 
  float pressure3_kPa = mpr.readPressure()/10;
  delay(20);
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
//  Serial.print("Requesting temperatures...");
  oneWireSensors.requestTemperatures(); // Send the command to get temperatures
//  Serial.println("DONE");
  Serial.printf("%4.2f, %4.2f, %4.2f, ", pressure1_kPa, pressure2_kPa, pressure3_kPa);

  
  for (uint8_t s=0; s < oneWireSensors.getDeviceCount(); s++) {
    // get the unique address 
    oneWireSensors.getAddress(addr, s);
    // just look at bottom two bytes, which is pretty likely to be unique
    int smalladdr = (addr[6] << 8) | addr[7]; 
//    Serial.print("Temperature for the device #"); Serial.print(s); 
//    Serial.print(" with ID #"); Serial.print(smalladdr);
//    Serial.print(" is: ");
      Serial.printf("%3.3f, ", oneWireSensors.getTempCByIndex(s)); 
  }
  Serial.print("\n");
  blink();
}
