#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include "plotly_streaming_cc3000wdt.h"
#include <avr/wdt.h>

#define WLAN_SSID       "kaqmak"
#define WLAN_PASS       "Saibaba8"
#define WLAN_SECURITY   WLAN_SEC_WPA2

int inPin = 2;
int powerPin = 7;
// Sign up to plotly here: https://plot.ly
// View your API key and streamtokens here: https://plot.ly/settings
#define nTraces 2
// View your tokens here: https://plot.ly/settings
// Supply as many tokens as data traces
// e.g. if you want to ploty A0 and A1 vs time, supply two tokens
char *tokens[nTraces] = {
  "5x19el5hwa","d3lyurclnz"};
// arguments: username, api key, streaming token, filename
plotly graph = plotly("kaqmak", "23rkqd46jq", tokens, "testMoisture", nTraces);



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

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void setup() {
  wdt_reset();
  wdt_disable();
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  pinMode(inPin, INPUT);      // sets the digital pin  as input
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, LOW);
  wifi_connect();

  graph.log_level = 0;
  //graph.fileopt="overwrite"; // See the "Usage" section in https://github.com/plotly/arduino-api for details
  graph.fileopt = "extend"; // Remove this if you want the graph to be overwritten
  graph.timezone = "Europe/Copenhagen";
  graph.maxpoints = 5000;
  bool success;
//  delay(65000);
  success = graph.init();
  //  if(!success){
  //    while(true){
  //    }
  //  }
  if(!success){
    Serial.println("No succes in init");
    while(true){
      wdt_enable(WDTO_8S);
    }
  }
  graph.openStream();
}

unsigned long x;
int y;

void loop() {
  wdt_enable(WDTO_8S); // "wdt" stands for "watchdog timer"
  wdt_reset();
  boolean isconnected = graph.cc3000.checkConnected();
  //boolean hasDHCP = checkDHCP();
  if(isconnected){
    Serial.print("\nconstant isconnected: ");
     
      Serial.println(isconnected);
  }
  else{
    while(true)
    {// wdt will reset after 8 s
    }    
  }
  wdt_reset();
  digitalWrite(powerPin, HIGH);
  delay(10);
  float h = analogRead(0);
  int hdig = digitalRead(inPin);
  digitalWrite(powerPin, LOW);
  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (isnan(h)) {
    Serial.println("\nFailed to read from Moisture dims");
  } 
  else {
    Serial.print("Free ram ");
    Serial.println(freeRam());
    Serial.print("Humidity: "); 
    Serial.println(h);
    Serial.print("Digital: "); 
    Serial.println(hdig);
    wdt_reset();
    graph.plot(millis(), h, tokens[0]);
    graph.plot(millis(), hdig, tokens[1]);
    //    Serial.print("Humidity: "); 
    //    Serial.print(h);
    //    Serial.print(" %\t");
    //    Serial.print("Temperature: "); 
    //    Serial.print(t);
    //    Serial.println(" *C");
  }
  wdt_disable();
  delay(30000);
}


