#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include "plotly_streaming_cc3000wdt.h"
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <Adafruit_INA219.h>
#include <Wire.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"

#define WLAN_SSID       "kaqmak"
#define WLAN_PASS       "Saibaba8"
//#define WLAN_SSID       "DyrehavenWiFi"
//#define WLAN_PASS       "dyrehaven34"
#define WLAN_SSID       "Fermentoren"
#define WLAN_PASS       "halmtorvet29c"
#define WLAN_SECURITY   WLAN_SEC_WPA2

//#define LOGGING_FREQ_SECONDS   3600       // Seconds to wait before a new sensor reading is logged.
#define LOGGING_FREQ_SECONDS   32       // testing
#define MAX_SLEEP_ITERATIONS   LOGGING_FREQ_SECONDS / 8  // Number of times to sleep (for 8 seconds) before
                                                         // a sensor reading is taken and sent to the server.
                                                         // Don't change this unless you also change the 
                                                         // watchdog timer configuration.
#define MEASUREMENTS_BEFORE_UPLOAD 2

#define INPIN 2
#define POWERPIN 7
int sleepIterations = 0;
volatile bool watchdogActivated = false;
#define nTraces 3


struct measurementSet
  {
    byte h: 8;
    unsigned long time: 32;
    int current_mA: 12;
    int loadvoltage: 12;
  } dataParams;


//EEPROM params
int measCount = 0;

char *tokens[nTraces] = {"s1swb3mjje","sb9xc012to","q1jyh0xoy7"};
//char *tokens[nTraces] = {"x","x"}; //Skovbasses tokens
// arguments: username, api key, streaming token, filename
plotly graph = plotly("kaqmak", "23rkqd46jq", tokens, "MoistureAndCunt", nTraces);

// Define watchdog timer interrupt.
ISR(WDT_vect)
{
  // Set the watchdog activated flag.
  // Note that you shouldn't do much work inside an interrupt handler.
  watchdogActivated = true;
}

// Put the Arduino to sleep.
void sleep()
{
  // Set sleep to full power down.  Only external interrupts or 
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Turn off the ADC while asleep.
  power_adc_disable();

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point.
  
  // When awake, disable sleep mode and turn on all devices.
  sleep_disable();
  power_all_enable();
}

boolean wifi_connect(){
  /* Initialise the module */
  Serial.println(F("Turning on CC3000."));
  //wlan_start(0);

  Serial.println(F("\n... Initializing..."));
  if (!graph.cc3000.begin())
  {
    Serial.println(F("... Couldn't begin()! Check your wiring?"));
    return false;
    //Serial.println(F("Starting watchdog"));    
    //wdt_enable(WDTO_8S);
    //while(1);
  }

  // Optional SSID scan
  // listSSIDResults();

  if (!graph.cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed connecting to WLAN!"));
    //Serial.println(F("Starting watchdog"));
    return false;
   // wdt_enable(WDTO_8S); 
    //while(1);
  }

  Serial.println(F("... Connected!"));

  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  int attempts = 0;
  while (!graph.cc3000.checkDHCP())
  {
    if (attempts > 20) {
      Serial.println(F("DHCP didn't finish!"));
      return false;
    }
    attempts += 1;
    delay(1000);
  }
  // Return success, the CC3000 is enabled and connected to the network.
  return true;

}

void shutdownWiFi() {
  // Disconnect from the AP if connected.
  // This might not be strictly necessary, but I found
  // it was sometimes difficult to quickly reconnect to
  // my AP if I shut down the CC3000 without first
  // disconnecting from the network.
  if (graph.cc3000.checkConnected()) {
    graph.cc3000.disconnect();
  }

  // Wait for the CC3000 to finish disconnecting before
  // continuing.
  while (graph.cc3000.checkConnected()) {
    delay(100);
  }
  
  // Shut down the CC3000.

  Serial.println("Trying to shut down cc3000");
   wlan_stop();
  //graph.cc3000.stop();
  
  Serial.println(F("CC3000 shut down.")); 
}

// Take a sensor reading and store in EEPROM
void logSensorReading() {
  // Take a sensor reading
  digitalWrite(POWERPIN, HIGH);//turn sensor on
  delay(10);
  dataParams.h = analogRead(0);//decrease resolution to 1 byte
  digitalWrite(POWERPIN, LOW);//turn sensor off
/* jeg vil med
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  dataParams.current_mA = ina219.getCurrent_mA();
  dataParams.loadvoltage = busvoltage + (shuntvoltage / 1000);
  dataParams.time = millis();
*/
  dataParams.current_mA = analogRead(1);
  dataParams.loadvoltage = analogRead(2);
  dataParams.time = millis();

  EEPROM_writeAnything(measCount++,dataParams);
}

// fetch EEPROM and upload
void uploadSensorReading(){
  // Connect to the server and send the reading.

 //Upload
  Serial.println(F("Sending measurements "));
  graph.reconnectStream();

  // Hent EEPROM data
  for(int i=0; i<measCount; i++){
      EEPROM_readAnything(i*sizeof(measurementSet),dataParams);
      graph.plot(dataParams.time, dataParams.h, tokens[0]);//change names
      graph.plot(dataParams.time, dataParams.current_mA, tokens[1]);
      graph.plot(dataParams.time, dataParams.loadvoltage, tokens[2]);
      // Note that if you're sending a lot of data you
      // might need to tweak the delay here so the CC3000 has
      // time to finish sending all the data before shutdown.
      delay(400);
  }
  measCount = 0;



  // Close the connection to the server.
  graph.closeStream();
}

void setup() {
  wdt_reset();
  wdt_disable();
  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  pinMode(INPIN, INPUT);      // sets the digital pin  as input
  pinMode(POWERPIN, OUTPUT);
  digitalWrite(POWERPIN, LOW);

  //currentsensor
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;

  wifi_connect();
  graph.log_level = 0;
  //graph.fileopt="overwrite"; // See the "Usage" section in https://github.com/plotly/arduino-api for details
  graph.fileopt = "extend"; // Remove this if you want the graph to be overwritten
  graph.timezone = "Europe/Copenhagen";
  graph.maxpoints = 5000;
  bool success;
//  delay(65000);
  success = graph.init();

  if(!success){
    Serial.println("No succes in init. Starting watchdog");
    while(true){
      wdt_enable(WDTO_8S);
    }
  }
  graph.openStream();
  delay(1000);
  graph.closeStream();
  delay(1000);
  Serial.println(F("Stream closed. Trying wlan_stop in setup"));
  wlan_stop();
 // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet
  // around page 50, Watchdog Timer.
  noInterrupts(); 
 // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1<<WDRF);
  
  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1<<WDP0) | (1<<WDP3);
  
  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1<<WDIE);
  
  // Enable interrupts again.
  interrupts();
  
  Serial.println(F("Setup complete.")); 
  delay(100);
}


void loop() {
  Serial.println(F("inside loop()"));
  if (watchdogActivated)
  {
    Serial.println(F("inside watchdogActivated"));
    delay(100);
    watchdogActivated = false;
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;
    Serial.println(sleepIterations);
    if (sleepIterations >= MAX_SLEEP_ITERATIONS) {
      Serial.println(F("sleepIteration above MAX_SLEEP"));
      Serial.flush();
      delay(10);
      
      // Reset the number of sleep iterations.
      sleepIterations = 0;
      // Log the sensor data (waking the CC3000, etc. as needed)
    
      logSensorReading();
      Serial.println(F("after logSensorReading"));
      Serial.flush();
    }
    if(measCount >= MEASUREMENTS_BEFORE_UPLOAD)
    {
      wlan_start(0);
      // Log the sensor data (waking the CC3000, etc. as needed)
      if (wifi_connect()) {
        Serial.println(F("in Loop: wifi_connected. Trying uploadSensorReading"));
        Serial.flush();
        uploadSensorReading();
        Serial.println(F("after uploadSensorReading"));
        Serial.flush();
      }
      shutdownWiFi();

    }
  }
  delay(100);
  
  // Go to sleep!
  sleep();
}

