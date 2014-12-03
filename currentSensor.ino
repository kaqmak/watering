#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include "plotly_streaming_cc3000wdt.h"
#include <avr/wdt.h>
#include <Adafruit_INA219.h>
#include <Wire.h>

//#define WLAN_SSID       "kaqmak"
//#define WLAN_PASS       "Saibaba8"
#define WLAN_SSID       "tju"
#define WLAN_PASS       "rasmusogelsebeth"
#define WLAN_SECURITY   WLAN_SEC_WPA2


Adafruit_INA219 ina219;
// Sign up to plotly here: https://plot.ly
// View your API key and streamtokens here: https://plot.ly/settings
#define nTraces 2
// View your tokens here: https://plot.ly/settings
// Supply as many tokens as data traces
// e.g. if you want to ploty A0 and A1 vs time, supply two tokens
char *tokens[nTraces] = {
  "xjtcxp9hpx","m5i1lryzz9"};
// arguments: username, api key, streaming token, filename
plotly graph = plotly("kaqmak", "23rkqd46jq", tokens, "CurrentSensor", nTraces);



void wifi_connect(){
  /* Initialise the module */
  Serial.println(F("\n... Initializing..."));
  if (!graph.cc3000.begin())
  {
    Serial.println(F("... Couldn't begin()! Check your wiring?"));
    Serial.println(F("Starting watchdog"));    
    wdt_enable(WDTO_8S);
    //while(1);
  }

  // Optional SSID scan
  // listSSIDResults();

  if (!graph.cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed connecting to WLAN!"));
    Serial.println(F("Starting watchdog"));
    wdt_enable(WDTO_8S); 
    //while(1);
  }

  Serial.println(F("... Connected!"));

  /* Wait for DHCP to complete */
  Serial.println(F("... Request DHCP"));
  //  while (!graph.cc3000.checkDHCP())
  //  {
  //    delay(100); // ToDo: Insert a DHCP timeout!
  //  }
  boolean flag = false;
  for (int ii = 0; ii < 500; ii++){ 
    if (!graph.cc3000.checkDHCP()){
      delay(100); // ToDo: Insert a DHCP timeout!
      flag = false;
    }
    else{
      flag = true; 
      Serial.println("DHCP succesfull");
      break;
    }
  }
  if(!flag){
    Serial.println("DHCP timeout");
    wdt_enable(WDTO_8S);
  }

}

void setup() {
  wdt_reset();
  wdt_disable();
  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  wifi_connect();

  graph.log_level = 0;
  //graph.fileopt="overwrite"; // See the "Usage" section in https://github.com/plotly/arduino-api for details
  graph.fileopt = "extend"; // Remove this if you want the graph to be overwritten
  graph.timezone = "Europe/Copenhagen";
  graph.maxpoints = 5000;
  bool success;
  success = graph.init();
  if(!success){
    Serial.println("No succes in init");
    while(true){
      wdt_enable(WDTO_8S);
    }
  }
  graph.openStream();

  //sensor part
  uint32_t currentFrequency;
  Serial.println("Hello!");
  Serial.println("Measuring voltage and current with INA219 ...");
  ina219.begin();

}

unsigned long x;
int y;

void loop() {
  wdt_enable(WDTO_8S); // restart watchdog
  wdt_reset(); // and reset
  boolean isconnected = graph.cc3000.checkConnected();
  //boolean hasDHCP = checkDHCP();
  if(isconnected){
    Serial.print("\nconstant isconnected: ");
    Serial.println(isconnected);
  }
  else{
    while(true);
  }
  wdt_reset();
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");

  //delay(2000);
  wdt_reset();
  graph.plot(millis(), current_mA, tokens[0]);
  graph.plot(millis(), loadvoltage, tokens[1]);
    //    Serial.print("Humidity: "); 
    //    Serial.print(h);
    //    Serial.print(" %\t");
    //    Serial.print("Temperature: "); 
    //    Serial.print(t);
    //    Serial.println(" *C");
  wdt_reset();
  wdt_disable();
  //delay(200);
}
