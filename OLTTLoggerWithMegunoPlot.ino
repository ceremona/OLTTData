/*
 * Cere Davis
 * @file OLTTLogger.ino
 * 
 * Logging OLTT data values from various sensors
 * + control logic
 * 
 * 25 May 2019
 *  
 */

#include "DFRobot_I2CMultiplexer.h"
#include "MegunoLink.h" // Helpful functions for communicating with MegunoLink. 
#include <OneWire.h>
#include <DallasTemperature.h>  // Version 3.7.6
#include <Wire.h>
#include "Adafruit_MPRLS.h"

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
//Arduino pins
// OneWire Data wire is plugged into port 14 on the Arduino
#define ONE_WIRE_BUS 22
#define PRESSURE_SCA 18 
#define PRESSURE_SCL 19
int ledState = LOW;

String Temp1 = "T_coldwater_bath";
String Temp2 = "T_thimble_filter";
String Temp3 = "T_mixing_chamber";
String Temp4 = "T_separator_column";
String Temp5 = "T_rotometer";
String Pressure1 = "P_mixing_chamber";
String Pressure2 = "P_rotometer";
String Pressure3 = "P_separator_column";

//  MegunoPlot stuff
Message Msg("CSV"); //"Data" = the taget message channel (remember to select this in megunolink)
// The plot we are sending data to.
TimePlot TempPlot1((char*)Temp1.c_str()), TempPlot2((char*)Temp2.c_str()), TempPlot3((char*)Temp3.c_str()), TempPlot4((char*)Temp4.c_str()), TempPlot5((char*)Temp5.c_str()), PressurePlot1((char*)Pressure1.c_str()), PressurePlot2((char*)Pressure2.c_str()), PressurePlot3((char*)Pressure3.c_str());
//TimePlot TempPlot1("Temp1"), TempPlot2("Temp2"), TempPlot3("Temp3"), TempPlot4("Temp4"), TempPlot5("Temp5"), PressurePlot1("Pressure1"), PressurePlot2("Pressure2"), PressurePlot3("Pressure3");

// I2C multiplexer
DFRobot_I2CMultiplexer I2CMulti(0x70);

// Pressure sensors
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWireBus(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature oneWireSensors(&oneWireBus);
DeviceAddress addr;

int I2CInit() {
 if (! mpr.begin(MPRLS_DEFAULT_ADDR, &Wire)) {
//  Note: use &Wire2 constructor argument for SDA2/SCL2, etc  
//    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    return 1;
  } else {
//    Serial.println("Found MPRLS sensor");
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

void setup()
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PRESSURE_SCA,INPUT);
  pinMode(PRESSURE_SCL,INPUT);
  // One Wire Thermocouple Sensors 
  pinMode(ONE_WIRE_BUS,INPUT_PULLUP);  
  
  // MegunoPlot stuff
  Msg.Clear();
  Msg.StartLogging();
  
  /*  
  TempPlot1.SetTitle("Temp1");
  TempPlot1.SetXlabel("Time");
  TempPlot1.SetYlabel("Temp C");
  TempPlot1.SetXRange(1/30);
  TempPlot1.SetSeriesProperties(F("Temp1"), Plot::Red, Plot::Solid, 2, Plot::FilledCircle);

  PressurePlot1.SetTitle("Pressure");
  PressurePlot1.SetXlabel("Time");
  PressurePlot1.SetYlabel("hPa");
  PressurePlot1.SetXRange(1/30);
  PressurePlot1.SetSeriesProperties(F("Pressure1"), Plot::Blue, Plot::Solid, 2, Plot::FilledCircle);
  //End Megunoplot stuff
  */

  if(I2CInit()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?"); 
  } 
  oneWireSensors.begin();
}

void loop()
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
    oneWireSensors.requestTemperatures(); // Send the command to get temperatures
    
  Msg.Begin(); 

    Serial.printf("%4.2f, ", pressure1_kPa);
    Serial.printf("%4.2f, ", pressure2_kPa);
    Serial.printf("%4.2f, ", pressure3_kPa);
  
  for (uint8_t s=0; s < oneWireSensors.getDeviceCount(); s++) {
    // get the unique address 
    oneWireSensors.getAddress(addr, s);
    // just look at bottom two bytes, which is pretty likely to be unique
    int smalladdr = (addr[6] << 8) | addr[7];
//    Serial.print(" with ID #"); Serial.print(smalladdr);
    Serial.printf("%3.3f, ", oneWireSensors.getTempCByIndex(s)); 
  }
  Msg.End();
    delay(20);
   
    oneWireSensors.getAddress(addr,0);
    int smalladdr = (addr[6] << 8) | addr[7];
    TempPlot1.SendData((char*)Temp1.c_str(), oneWireSensors.getTempCByIndex(0));
    oneWireSensors.getAddress(addr,1); 
    smalladdr = (addr[6] << 8) | addr[7];
    TempPlot2.SendData((char*)Temp2.c_str(), oneWireSensors.getTempCByIndex(1));
    oneWireSensors.getAddress(addr,2); 
    smalladdr = (addr[6] << 8) | addr[7];
    TempPlot3.SendData((char*)Temp3.c_str(), oneWireSensors.getTempCByIndex(2));
    oneWireSensors.getAddress(addr,3); 
    smalladdr = (addr[6] << 8) | addr[7];
    TempPlot4.SendData((char*)Temp4.c_str(), oneWireSensors.getTempCByIndex(3));
    oneWireSensors.getAddress(addr,4); 
    smalladdr = (addr[6] << 8) | addr[7];
    TempPlot5.SendData((char*)Temp5.c_str(), oneWireSensors.getTempCByIndex(4));
 //   oneWireSensors.getAddress(addr,5); 

    PressurePlot1.SendData((char*)Pressure1.c_str(), pressure1_kPa);
    PressurePlot2.SendData((char*)Pressure2.c_str(), pressure2_kPa);
    PressurePlot3.SendData((char*)Pressure3.c_str(), pressure3_kPa);
    delay(100);
    blink(); 
}
