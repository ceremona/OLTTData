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

#include "MegunoLink.h" // Helpful functions for communicating with MegunoLink. 
#include <OneWire.h>
#include <DallasTemperature.h> 
#include <Wire.h>
#include "Adafruit_MPRLS.h"

//  MegunoPlot stuff
Message Msg("CSV"); //"Data" = the taget message channel (remember to select this in megunolink)
// The plot we are sending data to.
TimePlot TempPlot1("Temp1"), TempPlot2("Temp2"), TempPlot3("Temp3"), TempPlot4("Temp4"), TempPlot5("Temp5"), TempPlot6("Temp6"), PressurePlot("Pressure");

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
//Arduino pins
// OneWire Data wire is plugged into port 14 on the Arduino
#define ONE_WIRE_BUS 22
#define PRESSURE_SCA 18 
#define PRESSURE_SCL 19
//
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWireBus(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature oneWireSensors(&oneWireBus);
DeviceAddress addr;

int oneWireInit() {
 if (! mpr.begin(MPRLS_DEFAULT_ADDR, &Wire)) {
//  Note: use &Wire2 constructor argument for SDA2/SCL2, etc
//    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    return 0;
  } else {
//    Serial.println("Found MPRLS sensor");
    return 1;
  }
}

void setup()
{
  // MegunoPlot stuff
  Msg.Clear();
  Msg.StartLogging();
  Serial.begin(9600);
  //
  
  TempPlot1.SetTitle("Temp1");
  TempPlot1.SetXlabel("Time");
  TempPlot1.SetYlabel("Temp C");
  TempPlot1.SetXRange(1/30);
  TempPlot1.SetSeriesProperties(F("Temp1"), Plot::Red, Plot::Solid, 2, Plot::FilledCircle);

  PressurePlot.SetTitle("Pressure");
  PressurePlot.SetXlabel("Time");
  PressurePlot.SetYlabel("hPa");
  PressurePlot.SetXRange(1/30);
  PressurePlot.SetSeriesProperties("Pressure", Plot::Blue, Plot::Solid, 2, Plot::FilledCircle);
 
 
  // End Megunoplot stuff
  pinMode(PRESSURE_SCA,INPUT);
  pinMode(PRESSURE_SCL,INPUT);
  // One Wire Thermocouple Sensors 
  pinMode(ONE_WIRE_BUS,INPUT_PULLUP);  

  if(oneWireInit()) Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
  // Start up the OneWire library
  oneWireSensors.begin();
  delay(100);
}

void loop()
{

    float pressure_hPa = mpr.readPressure();
    delay(20);
    oneWireSensors.requestTemperatures(); // Send the command to get temperatures
    
  Msg.Begin(); 
    
//  Serial.print("Pressure (hPa): "); Serial.println(pressure_hPa);
//  Serial.print("Pressure (PSI): "); Serial.println(pressure_hPa / 68.947572932);
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
//  Serial.print("Requesting temperatures...");

//  Serial.println("DONE");
    Serial.printf("%4.2f, ", pressure_hPa);
  
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
  Msg.End();
    
    delay(20);

   // const char* str = &temp;
//    for (uint8_t s=0; s < oneWireSensors.getDeviceCount(); s++) {
    // get the unique address 
 //      oneWireSensors.getAddress(addr, s);
    // just look at bottom two bytes, which is pretty likely to be unique
   //    int smalladdr = (addr[6] << 8) | addr[7];    
//    Serial.print("Temperature for the device #"); Serial.print(s); 
//    Serial.print(" with ID #"); Serial.print(smalladdr);
//    Serial.print(" is: ");
      //Serial.printf("%3.3f, ", oneWireSensors.getTempCByIndex(s));
//    }      
    oneWireSensors.getAddress(addr,0);
    int smalladdr = (addr[6] << 8) | addr[7];
    TempPlot1.SendData(F("Temp1"), oneWireSensors.getTempCByIndex(0));
    oneWireSensors.getAddress(addr,1); 
    smalladdr = (addr[6] << 8) | addr[7];
    TempPlot2.SendData("Temp2", oneWireSensors.getTempCByIndex(1));
    oneWireSensors.getAddress(addr,2); 
    smalladdr = (addr[6] << 8) | addr[7];
    TempPlot3.SendData("Temp3", oneWireSensors.getTempCByIndex(2));
    oneWireSensors.getAddress(addr,3); 
    smalladdr = (addr[6] << 8) | addr[7];
    TempPlot4.SendData("Temp4", oneWireSensors.getTempCByIndex(3));
    oneWireSensors.getAddress(addr,4); 
    smalladdr = (addr[6] << 8) | addr[7];
    TempPlot5.SendData("Temp5", oneWireSensors.getTempCByIndex(4));
    oneWireSensors.getAddress(addr,5); 
    smalladdr = (addr[6] << 8) | addr[7];
    TempPlot6.SendData("Temp6", oneWireSensors.getTempCByIndex(5));

    PressurePlot.SendData("Pressure", pressure_hPa);
    delay(200);
//  
}
