#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include "plotly_streaming_cc3000wdt.h"
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
//#include <Adafruit_INA219.h>
#include <Wire.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"

//#define WLAN_SSID       "FTNK-AL0001"
//#define WLAN_SSID       "kaqmak"
//#define WLAN_PASS       "Saibaba8"
//#define WLAN_SSID       "PersIphone"
//#define WLAN_PASS       "Saibaba8"

#define WLAN_SSID       "DyrehavenWiFi"
#define WLAN_PASS       "dyrehaven34"
//#define WLAN_SSID       "Fermentoren"
//#define WLAN_PASS       "halmtorvet29c"
//#define WLAN_SSID       "tju"
//#define WLAN_PASS       "rasmusogelsebeth"
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
#define WIFIPWRPIN 8
int sleepIterations = 0;
volatile bool watchdogActivated = false;
volatile bool sleep_entered;
#define nTraces 3


struct measurementSet
  {
    byte h;
    unsigned long time;
    float current_mA;
    float loadvoltage;
  } dataParams;


//EEPROM params
int measCount = 0;

char *tokens[nTraces] = {"s1swb3mjje","sb9xc012to","q1jyh0xoy7"};
//char *tokens[nTraces] = {"x","x"}; //Skovbasses tokens
// arguments: username, api key, streaming token, filename
plotly graph = plotly("kaqmak", "yv586vmfqj", tokens, "MoistureMini", nTraces);


////////////////////////////////////////
inline void configure_wdt(void)
{
    /* A 'timed' sequence is required for configuring the WDT, so we need to 
     * disable interrupts here.
     */
    cli(); 
    wdt_reset();
    MCUSR &= ~_BV(WDRF);
    /* Start the WDT Config change sequence. */
    WDTCSR |= _BV(WDCE) | _BV(WDE);
    /* Configure the prescaler and the WDT for interrupt mode only*/
    WDTCSR =  _BV(WDP0) | _BV(WDP3) | _BV(WDIE);
    sei();
}

void app_sleep_init(void)
{
    /* Setup the flag. */
    sleep_entered = false;
    configure_wdt();
}

// Put the Arduino to sleep.
void sleep(void)
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_entered = true;
    sleep_enable();
    sei();
    sleep_mode(); 
    /* Execution will resume here. */
}
// Define watchdog timer interrupt.
// Define watchdog timer interrupt.
ISR(WDT_vect)
{
    /* Check if we are in sleep mode or it is a genuine WDR. */
    Serial.println('He he');
    Serial.flush();
    if(sleep_entered == false)
    {
        /* The app has locked up, force a WDR. */
        //
        wdt_enable(WDTO_2S);
        
        while(1);
    }
    else
    {
        wdt_reset();
        /* Service the timer if necessary. */
        sleep_entered = false;
        sleep_disable();
        
        /* Inline function so OK to call from here. */
        configure_wdt();
    }
}


boolean wifi_connect(){
  /* Initialise the module */
  Serial.println(F("Turning on CC3000."));
  //wlan_start(0);

  Serial.println(F("\n... Initializing..."));
  wdt_enable(WDTO_8S);
  wdt_reset();
  if (!graph.cc3000.begin())
  {
    Serial.println(F("... Couldn't begin()!"));
    return false;
    //Serial.println(F("Starting watchdog"));    
    //wdt_enable(WDTO_8S);
    //while(1);
  }
  Serial.println(F("Initialized"));
  wdt_reset();

  // Optional SSID scan
  // listSSIDResults();
  //wdt_enable(WDTO_8S);
  //wdt_reset();
  if (!graph.cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed connecting to WLAN!"));
    //Serial.println(F("Starting watchdog"));
    //wdt_reset();
    //wdt_disable();
    return false;
   // wdt_enable(WDTO_8S); 
    //while(1);
  }
  wdt_reset();
  Serial.println(F("... Connected!"));

  /* Wait for DHCP to complete */
  //Serial.println(F("Request DHCP"));
  int attempts = 0;
  while (!graph.cc3000.checkDHCP())
  {
    wdt_reset();
    if (attempts > 20) {
      Serial.println(F("no DHCP"));
      wdt_disable();
      return false;
    }
    attempts += 1;
    delay(1000);
  }
  // Return success, the CC3000 is enabled and connected to the network.
  wdt_reset();
  wdt_disable();
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

  Serial.println("Shutting cc3000");
   wlan_stop();
  //graph.cc3000.stop();
  
  //Serial.println(F("CC3000 shut down."));
}

/*void print_measurementSet() {
  Serial.print("Time: ");
  Serial.println(dataParams.time);
  Serial.print("current_mA: ");
  Serial.println(dataParams.current_mA);
  Serial.print("loadvoltage: ");
  Serial.println(dataParams.loadvoltage);
}*/

// Take a sensor reading and store in EEPROM
void logSensorReading() {
  // Take a sensor reading
  digitalWrite(POWERPIN, HIGH);//turn sensor on
  delay(10);
  dataParams.h = (byte) analogRead(0)>>2;//decrease resolution to 1 byte
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
  //Serial.println("Saving the following to EEPROM");
  //print_measurementSet();
  EEPROM_writeAnything(measCount*sizeof(measurementSet),dataParams);
  measCount++;
}


// fetch EEPROM and upload
void uploadSensorReading(){
  // Connect to the server and send the reading.

 //Upload
  Serial.println(F("Sending measurements"));
  //graph.reconnectStream();

  // Hent EEPROM data
  for(int i=0; i<measCount; i++){
      EEPROM_readAnything(i*sizeof(measurementSet),dataParams);
      //Serial.println("Fethed this from EEPROM");
      //print_measurementSet();
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
  pinMode(WIFIPWRPIN, OUTPUT);
  digitalWrite(WIFIPWRPIN, HIGH);

  //currentsensor
  float shuntvoltage = 0;
  float busvoltage = 0;
  //float current_mA = 0;
  //float loadvoltage = 0;
  app_sleep_init();
  wifi_connect();  
  graph.log_level = 0;
  //graph.fileopt="overwrite"; // See the "Usage" section in https://github.com/plotly/arduino-api for details
  graph.fileopt = "extend"; // Remove this if you want the graph to be overwritten
  graph.timezone = "Europe/Copenhagen";
  graph.maxpoints = 5000;
  graph.convertTimestamp = true; // tell plotly that you're stamping your data with a millisecond counter and that you want plotly to convert it into a date-formatted graph
  bool success;
//  delay(65000);
  success = graph.init();

  if(!success){
    Serial.println(F("No succes in init"));
    wdt_enable(WDTO_1S);
    while(true);
  }
  graph.openStream();
  delay(1000);
  graph.closeStream();
  delay(1000);
  //Serial.println(F("Stream closed. Trying wlan_stop in setup"));
  wlan_stop();
  digitalWrite(WIFIPWRPIN, LOW);
 //initialize sleep parameters
  
 
}


void loop() {
  //Serial.println(F("inside loop()"));

    //Serial.println(F("inside watchdogActivated"));
    delay(100);
    //watchdogActivated = false;
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;
    Serial.println(sleepIterations);
    if (sleepIterations >= MAX_SLEEP_ITERATIONS) {
      //Serial.println(F("sleepIteration above MAX_SLEEP"));
      //Serial.flush();
      delay(10);

      // Reset the number of sleep iterations.
      sleepIterations = 0;
      // Log the sensor data (waking the CC3000, etc. as needed)
      //wdt_enable(WDTO_8S); Skal denne med?
      logSensorReading();
      //Serial.println(F("after logSensorReading"));
      Serial.flush();
    }
    if(measCount >= MEASUREMENTS_BEFORE_UPLOAD)
    {
      wdt_enable(WDTO_8S);
      digitalWrite(WIFIPWRPIN, HIGH);//turn on power to the CC3000
      delay(500);
      wlan_start(0);
      wdt_reset();
      wdt_disable();
      //configure_wdt();
      delay(500);
      // Log the sensor data (waking the CC3000, etc. as needed)
      if (wifi_connect()) {
        //Serial.println(F("in Loop: wifi_connected. Trying uploadSensorReading"));
        Serial.flush();
        uploadSensorReading();
        //Serial.println(F("after uploadSensorReading"));
        //Serial.flush();
      }else{
        asm volatile ("  jmp 0");
      }
      wdt_enable(WDTO_8S);
      shutdownWiFi();
      delay(400);
      digitalWrite(WIFIPWRPIN, LOW);//turn off power to the CC3000
      Serial.print("Efter wifipin low");
      //wdt_reset();
      //wdt_disable();
      app_sleep_init();//needed to switch back to interrupt mode
    }

  delay(100);

  // Go to sleep!
  sleep();
}

