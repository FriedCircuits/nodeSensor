/*------------------------------------------------------------------------------
* Author       William Garrido
* Created      07-30-2016
* Purpose      Livingroom Node and LaCrosse Gateway
* Git:         https://github.com/FriedCircuits/nodeSensor           
* Blog:        http://mobilewill.us
* License:     CC-BY-SA
* Referance:   MQTT: https://home-assistant.io/blog/2015/10/11/measure-temperature-with-esp8266-and-report-to-mqtt/
*              LaCrosse: http://arduino.cc/forum/index.php/topic,142871.msg1106336.html#msg1106336
*              RF Timing
*              __           ___       ___    ___
*                |         |  |      |  |   |  |
*                |_________|  |______|  |___|  |
*                |  Sync      |    1    |  0   |
*                |  8320us    | 4500us  | 2530us
-------------------------------------------------------------------------------
* *History*
*
* 
*          
------------------------------------------------------------------------------*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "DHT.h"

//Pin Mapping
#define LDR_PIN A0
#define DHT_PIN D7
#define REED_PIN D2
#define RF_PIN D6
#define LED_PIN D0

//How often to report sensor readings (except for La crossee gateway)
#define UPDATE_DELAY 60000

#define DHTTYPE DHT11

DHT dht(DHT_PIN, DHTTYPE);

//WiFi Setup
const char* ssid = "SSID";
const char* password = "PASSWORD";
const char* host = "OTA-ESP8266";

//MQTT Setup
#define MQTT_SERVER "mqtt.local"
//Topipcs
#define HUMIDITY_TOPIC "esp/livingroom/h"
#define TEMP_TOPIC "esp/livingroom/t"
#define HEATINDEX_TOPIC "esp/livingroom/hi"
#define LIGHT_TOPIC "esp/livingroom/l"
#define REED_TOPIC "esp/livingroom/reed"
#define LACROSSE_TOPIC_TEMP "esp/outside/t"
#define LACROSSE_TOPIC_BAT "esp/outside/bat"
#define LACROSSE_TOPIC_CH "esp/outside/ch"

//Sensor variables and tracking of last message for timing
long lastMsg = 0;
float temp = 0.0;
float hum = 0.0;
float heatindex = 0.0;
bool reed = 0;
bool reedOld = 0;
uint16_t light = 0;

//Tracking if first boot so we can do stuff once
bool firstboot = 1;

//WiFi-MQTT client objects
WiFiClient espClient;
PubSubClient client(espClient);


//Defines for 433Mhz radio from LaCrosse
#define DataBits0 4                                       // Number of data0 bits to expect
#define DataBits1 32                                      // Number of data1 bits to expect
#define allDataBits 36                                    // Number of data sum 0+1 bits to expect
// isrFlags bit numbers
#define F_HAVE_DATA 1                                     // 0=Nothing in read buffer, 1=Data in read buffer
#define F_GOOD_DATA 2                                     // 0=Unverified data, 1=Verified (2 consecutive matching reads)
#define F_CARRY_BIT 3                                     // Bit used to carry over bit shift from one long to the other
#define F_STATE 7                                         // 0=Sync mode, 1=Data mode

// Constants
const unsigned long sync_MIN = 4300;                      // Minimum Sync time in micro seconds
const unsigned long sync_MAX = 4700;

const unsigned long bit1_MIN = 2300;
const unsigned long bit1_MAX = 2700;

const unsigned long bit0_MIN = 1330;
const unsigned long bit0_MAX = 1730;

const unsigned long glitch_Length = 300;                  // Anything below this value is a glitch and will be ignored.

// Interrupt variables
unsigned long fall_Time = 0;                              // Placeholder for microsecond time when last falling edge occured.
unsigned long rise_Time = 0;                              // Placeholder for microsecond time when last rising edge occured.
byte bit_Count = 0;                                       // Bit counter for received bits.
unsigned long build_Buffer[] = {
  0,0};                     // Placeholder last data packet being received.
volatile unsigned long read_Buffer[] = {
  0,0};             // Placeholder last full data packet read.
volatile byte isrFlags = 0;                               // Various flag bits

//If no parameters then toggle LED otherwise blink specified times at specified speed default: 50ms delay)
//Param 1 how many flashes
//Param 2 delay between flashes
void blink(uint8_t flash = 0, uint8_t spd = 50){
  bool currentLED = digitalRead(LED_PIN);
  if(flash > 0){
    for(int i = 0; i <= flash; i++){
      //digitalWrite(LED_PIN, LOW);
      for(int b = 0; b <= spd; b++){
        digitalWrite(LED_PIN, LOW);
        delayMicroseconds(5);
        delay(0);
      }
      //delay(spd);    
      digitalWrite(LED_PIN, HIGH);
      delay(spd);
    }
    //digitalWrite(LED_PIN, currentLED);
  }
  else {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}


//Read data on change from 433Mhz Radio
void PinChangeISR0(){                                     // Pin 2 (Interrupt 0) service routine
  unsigned long Time = micros();                          // Get current time
  if (digitalRead(RF_PIN) == LOW) {
    // Falling edge
    if (Time > (rise_Time + glitch_Length)) {
      // Not a glitch
      Time = micros() - fall_Time;                        // Subtract last falling edge to get pulse time.
      if (bitRead(build_Buffer[1],31) == 1)
        bitSet(isrFlags, F_CARRY_BIT);
      else
        bitClear(isrFlags, F_CARRY_BIT);

      if (bitRead(isrFlags, F_STATE) == 1) {
        // Looking for Data
        if ((Time > bit0_MIN) && (Time < bit0_MAX)) {
          // 0 bit
          build_Buffer[1] = build_Buffer[1] << 1;
          build_Buffer[0] = build_Buffer[0] << 1;
          if (bitRead(isrFlags,F_CARRY_BIT) == 1)
            bitSet(build_Buffer[0],0);
          bit_Count++;
        }
        else if ((Time > bit1_MIN) && (Time < bit1_MAX)) {
          // 1 bit
          build_Buffer[1] = build_Buffer[1] << 1;
          bitSet(build_Buffer[1],0);
          build_Buffer[0] = build_Buffer[0] << 1;
          if (bitRead(isrFlags,F_CARRY_BIT) == 1)
            bitSet(build_Buffer[0],0);
          bit_Count++;
        }
        else {
          // Not a 0 or 1 bit so restart data build and check if it's a sync?
          bit_Count = 0;
          build_Buffer[0] = 0;
          build_Buffer[1] = 0;
          bitClear(isrFlags, F_GOOD_DATA);                // Signal data reads dont' match
          bitClear(isrFlags, F_STATE);                    // Set looking for Sync mode
          if ((Time > sync_MIN) && (Time < sync_MAX)) {
            // Sync length okay
            bitSet(isrFlags, F_STATE);                    // Set data mode
          }
        }
        if (bit_Count >= allDataBits) {
          // All bits arrived
          bitClear(isrFlags, F_GOOD_DATA);                // Assume data reads don't match
          if (build_Buffer[0] == read_Buffer[0]) {
            if (build_Buffer[1] == read_Buffer[1]) 
              bitSet(isrFlags, F_GOOD_DATA);              // Set data reads match
          }
          read_Buffer[0] = build_Buffer[0];
          read_Buffer[1] = build_Buffer[1];
          bitSet(isrFlags, F_HAVE_DATA);                  // Set data available
          bitClear(isrFlags, F_STATE);                    // Set looking for Sync mode
          //blink(); // Used for debugging
          build_Buffer[0] = 0;
          build_Buffer[1] = 0;
          bit_Count = 0;
        }
      }
      else {
        // Looking for sync
        if ((Time > sync_MIN) && (Time < sync_MAX)) {
          // Sync length okay
          build_Buffer[0] = 0;
          build_Buffer[1] = 0;
          bit_Count = 0;
          bitSet(isrFlags, F_STATE);                      // Set data mode
          //blink(); // Used for debugging
        }
      }
      fall_Time = micros();                               // Store fall time
    }
  }
  else {
    // Rising edge
    if (Time > (fall_Time + glitch_Length)) {
      // Not a glitch
      rise_Time = Time;                                   // Store rise time
    }
  }
}


//Init pins
void setup_pins() {
  //LED Output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  //REED Input
  pinMode(REED_PIN, INPUT_PULLUP);

  //433Mhz 
  pinMode(RF_PIN, INPUT);  
}

void setup_OTA(){
  ArduinoOTA.setHostname(host);
  ArduinoOTA.onStart([]() { blink(); });
  ArduinoOTA.onEnd([]() { blink(); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

   /* setup the OTA server */
   ArduinoOTA.begin();
   Serial.println("OTA Ready");
}

void setup_wifi() {
  blink();
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.hostname(host);
  WiFi.begin(ssid, password);
  unsigned long lastCheck = millis(); //Track how long we have been waiting for WiFi connection
  bool connected = false;

  while (WiFi.status() != WL_CONNECTED) {
    for (int i = 0; i < 500; i++) {
      delay(1);
    }
    Serial.print(".");
    if (millis() - lastCheck >= 10000) {
      connected = false;
      break;
    }
    connected = true;
  }
  Serial.println("");
  if (!connected) {
    Serial.println("WiFi not connected");
    while(1){blink(10,10);}
  } else {
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
  blink();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("Livingroom")){
      Serial.println("connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup_mqtt(){
  client.setServer(MQTT_SERVER, 1883);
}

void setup() {
  Serial.begin(115200);
  setup_pins();
  setup_wifi();
  setup_OTA();
  setup_mqtt();
  dht.begin();
  attachInterrupt(RF_PIN,PinChangeISR0,CHANGE);
}

void dec2binLong(unsigned long myNum, byte NumberOfBits) {
  if (NumberOfBits <= 32){
    myNum = myNum << (32 - NumberOfBits);
    for (int i=0; i<NumberOfBits; i++) {
      if (bitRead(myNum,31) == 1){ 
        #if SERIAL
          Serial.print("1");
        #endif
      }
      else{ 
        #if SERIAL
          Serial.print("0");
        #endif 
      }
      myNum = myNum << 1;
    }
  }
  #if SERIAL
   serialFlush();
  #endif
}

//Publish sensor data if time has passed
//Check is reed switch has changed
//If we have complete data from 433Mhz radio process and publish
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  ArduinoOTA.handle();

  long now = millis();
  if(((now - lastMsg)  > UPDATE_DELAY) || firstboot){
      Serial.println("Sending data");
      hum = dht.readHumidity();
      temp = dht.readTemperature(true);
      light = analogRead(LDR_PIN);
      heatindex = dht.computeHeatIndex(temp, hum);
      client.publish(HUMIDITY_TOPIC, String(hum).c_str(), true);
      client.publish(TEMP_TOPIC, String(temp).c_str(), true);
      client.publish(LIGHT_TOPIC, String(light).c_str(), true);
      client.publish(HEATINDEX_TOPIC, String(heatindex).c_str(), true);
      lastMsg = now;
  }
  
  reed = digitalRead(REED_PIN);
  if((reed != reedOld) || firstboot){
    client.publish(REED_TOPIC, String(reed).c_str(), true);
    blink(5);
    reedOld=reed;
  }
  if(firstboot) firstboot = false;

  unsigned long myData0 = 0;
  unsigned long myData1 = 0;
  if (bitRead(isrFlags,F_GOOD_DATA) == 1) 
  {
    // We have at least 2 consecutive matching reads
    myData0 = read_Buffer[0]; // Read the data spread over 2x 32 variables
    myData1 = read_Buffer[1];
    bitClear(isrFlags,F_HAVE_DATA); // Flag we have read the data
    dec2binLong(myData0,DataBits0);
    dec2binLong(myData1,DataBits1);

    Serial.print(" - Battery=");
    byte H = (myData1 >> 26) & 0x3;   // Get Battery
    Serial.print(H);
    client.publish(LACROSSE_TOPIC_BAT, String(H).c_str(), true);
    
    Serial.print(" Channel=");
    H = ((myData1 >> 24) & 0x3) + 1;        // Get Channel
    Serial.print(H);
    client.publish(LACROSSE_TOPIC_CH, String(H).c_str(), true);
    
    Serial.print(" Temperature=");
    byte ML = (myData1 >> 12) & 0xF0; // Get MMMM
//     Serial.print(" (M=");
//     Serial.print(ML);
    H = (myData1 >> 12) & 0xF;        // Get LLLL
//     Serial.print(" L=");
//     Serial.print(H);
    ML = ML | H;                      // OR MMMM & LLLL nibbles together
    H = (myData1 >> 20) & 0xF;        // Get HHHH
//     Serial.print(" H=");
//     Serial.print(H);
//     Serial.print(" T= ");
    byte HH = 0;
    if((myData1 >> 23) & 0x1 == 1) //23 bit
         HH = 0xF;
    int Temperature = (H << 8)|(HH << 12) | ML;  // Combine HHHH MMMMLLLL
//     Serial.print( Temperature);
//     Serial.print(") ");
    // Temperature = Temperature*3; //F // Remove Constant offset
    Serial.print(Temperature/10.0,1);   
    Serial.println("C");

    client.publish(LACROSSE_TOPIC_TEMP, String((1.8 * (Temperature/10.0)) + 32).c_str(), true);
    blink(5,5);
  }
}
