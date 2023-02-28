
/*This sketch requires two external libraries:
  IRremoteEsp8266 (2.8.4) by David Conran
  ThingSpeak (2.0.0) by MathWorks
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <IRac.h>
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRtext.h>
#include <IRutils.h>
#include <Servo.h>
#include <assert.h>

#include "ThingSpeak.h"
#include "secrets.h"

#define DEBUG 1  // comment out this line to remove debug messages from the code

#ifdef DEBUG
// Debug messages included
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugf(y, z) Serial.printf(y, z)
#else
// Debug messages removed (Handy to save memory and clock cycles when no serial com is established)
#define debug(...)
#define debugln(...)
#define debugf(...)
#endif

//  Fixed by hardware
const uint16_t ledPin = D4;
const uint16_t relayPin = D1;

// User changable
const uint16_t servoPin = D5;
const uint16_t kRecvPin = D7;

// Set the smallest sized "UNKNOWN" message packets we actually care about.
// This value helps reduce the false-positive detection rate of IR background
// noise as real messages. The chances of background IR noise getting detected
// as a message increases with the length of the kTimeout value. (See above)
// The downside of setting this message too large is you can miss some valid
// short messages for protocols that this library doesn't yet decode.
//
// Set higher if you get lots of random short UNKNOWN messages when nothing
// should be sending a message.
// Set lower if you are sure your setup is working, but it doesn't see messages
// from your device. (e.g. Other IR remotes work.)
// NOTE: Set this value very high to effectively turn off UNKNOWN detection.
const uint16_t kMinUnknownSize = 12;

// How much percentage lee way do we give to incoming signals in order to match
// it?
// e.g. +/- 25% (default) to an expected value of 500 would mean matching a
//      value between 375 & 625 inclusive.
// Note: Default is 25(%). Going to a value >= 50(%) will cause some protocols
//       to no longer match correctly. In normal situations you probably do not
//       need to adjust this value. Typically that's when the library detects
//       your remote's message some of the time, but not all of the time.
const uint8_t kTolerancePercentage = kTolerance;  // kTolerance is normally 25%

// Use turn on the save buffer feature for more complete capture coverage.
IRrecv irrecv(kRecvPin);
decode_results results;  // Somewhere to store the results

Servo s;
int angleCurrent = 90;  // middle position
int angleClosed = 80;   // safe starting values in case EEPROM fails
int angleOpen = 100;    // safe starting values in case EEPROM fails

/***************************
   Personal Connection Settings imported from "secrets.h"
 **************************/

const char *WIFI_SSID = SECRET_SSID;
String WIFI_PWD = SECRET_PASS;
const char *READ_API_KEY = SECRET_READ_APIKEY;  // Your own thingspeak api_key
const unsigned long CHANNEL_ID = SECRET_CH_ID;
const unsigned int SENSOR_FIELD = SENSOR_READ_FIELD;

/**************************
   WIFI Settings

  Included in ThingSpeak.h
  const char *host = "api.thingspeak.com";                  //IP address of the thingspeak server
  const int httpPort = 80;
 ************************/
WiFiClient client;

/**************************
   IR control values

   Hexa decimal values of buttons from an NEC encoded RGB-light remote
 ************************/
const uint64_t IR_UP = 0xF700FF;
const uint64_t IR_DOWN = 0xF7807F;
const uint64_t IR_ON = 0xF7C03F;
const uint64_t IR_OFF = 0xF740BF;
const uint64_t IR_RED = 0XF720DF;    // RED
const uint64_t IR_GREEN = 0XF7A05F;  // GREEN
const uint64_t IR_BLUE = 0XF7609F;   // BLUE
const uint64_t IR_WHITE = 0xF7E01F;  // WHITE
const uint64_t IR_P5 = 0xF7D02F;     // FLASH
const uint64_t IR_P1 = 0xF7F00F;     // STROBE
const uint64_t IR_M1 = 0xF7C837;     // FADE
const uint64_t IR_M5 = 0xF7E817;     // SMOOTH

/* numbers 0 - 9 as on a keypad, they can be used to type numbers to the device using the colors on the RGB remote
|7|8|9|
|4|5|6|
|1|2|3|
| |0| |
#define IR_0 0XF7A857
#define IR_1 0XF708F7
#define IR_2 0XF78877
#define IR_3 0XF748B7
#define IR_4 0XF730CF
#define IR_5 0XF7B04F
#define IR_6 0XF7708F
#define IR_7 0XF710EF
#define IR_8 0XF7906F
#define IR_9 0XF750AF
*/
const uint64_t PINNUMBERS[10] =
    {
        0XF7A857,
        0XF708F7,
        0XF78877,
        0XF748B7,
        0XF730CF,
        0XF7B04F,
        0XF7708F,
        0XF710EF,
        0XF7906F,
        0XF750AF,
};

/**************************
   Function prototypes
 ************************/
void initWifi();
void disableWifi();
void ledOn();
void ledOff();
void toggleLed();
void getData();
void changeEnvControl();
void changeLimits();
void relayOff();
void relayOn();
void closeAV();
void openAV();

/**************************
   Globals
 ************************/

// Timing values
unsigned long ledLastMillis = 0;
const int LED_BLINK_INTERVAL_MS = 250;  // ms
unsigned long dataLastMillis = 0;
const int GETDATA_INTERVAL_SEC = 15;  // S
unsigned long relayTime = 0;

// boolean flags
bool Blink = false;
bool envControl = false;
bool Wifienabled = false;

// pin states
uint8_t ledState = LOW;  // Note: The built-in led will light up when LOW and dim when HIGH
uint8_t relayState = LOW;

void setup() {
  pinMode(relayPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  ledOff();
  // servo standard attach initializes at default angle 1500us => 90deg
  s.attach(servoPin);
  Serial.begin(115200);

  // reserve 2 bytes to hold the permanent angle limit values of the ventilation lever.
  EEPROM.begin(2);  // keep in mind these bytes could be overwritten to unknown values when flashing new firmware

  while (!Serial)  // Wait for the serial connection to be establised.
    delay(50);
  // Perform a low level sanity checks that the compiler performs bit field
  // packing as we expect and Endianness is as we expect.
  assert(irutils::lowLevelSanityCheck() == 0);
  debugf("\n" D_STR_IRRECVDUMP_STARTUP "\n", kRecvPin);
#if DECODE_HASH
  // Ignore messages with less than minimum on or off pulses.
  irrecv.setUnknownThreshold(kMinUnknownSize);
#endif                                        // DECODE_HASH
  irrecv.setTolerance(kTolerancePercentage);  // Override the default tolerance.
  irrecv.enableIRIn();                        // Start the receiver // call irrecv.disableIRIn() to disable timers and detach interrupt on kRecvPin
  // ESP.eraseConfig();
  WiFi.softAPdisconnect(true);  // Switch off software enabled accesspoint
  // WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // !!! Setting the min and max angle limit for the servo to avoid DESTROYING the ventilation unit or the 3d-printed frame !!!
  // make sure the lever arm is installed on the servo while the servo angle is in the middle positon
  angleClosed = EEPROM.read(0);  // read previously saved closed angle limit value of ventilation lever from EEPROM
  angleOpen = EEPROM.read(1);    // read previously saved opened angle limit value of ventilation lever from EEPROM
  // the limit angles can be adjusted with the "changeLimits()" function which can be called from the main loop using the remote
}

void loop() {
  if (Wifienabled && !WiFi.isConnected()) {
    WiFi.reconnect();
  }

  // Check if the IR code has been received.
  if (irrecv.decode(&results)) {
    if (results.decode_type == NEC) {
      // debugln(results.value);
      switch (results.value) {
        case IR_ON:
          initWifi();
          break;
        case IR_OFF:
          disableWifi();
          break;
        case IR_GREEN:
          openAV();
          break;
        case IR_RED:
          closeAV();
          break;
        case IR_UP:
          relayOn();
          angleCurrent += 5;
          angleCurrent = constrain(angleCurrent, angleClosed, angleOpen);
          s.write(angleCurrent);
          relayTime = millis();
          break;
        case IR_DOWN:
          relayOn();
          angleCurrent -= 5;
          angleCurrent = constrain(angleCurrent, angleClosed, angleOpen);
          s.write(angleCurrent);
          relayTime = millis();
          break;
        case IR_WHITE:
          // not implemented yet
          changeEnvControl();
          break;
        case IR_BLUE:
          changeLimits();
          break;
        default:
          break;
      }
    } else {
      debugln("Unknown.");
    }
    irrecv.resume();  // Receive the next value
  }

  if (relayState && millis() - relayTime > 3000) {  // Turn the relay off when the servo is not in use
    relayOff();
  }

  // If an error occurs, Blink will be set to "true" and this statement will make the onboard LED blink
  if (Blink && millis() - ledLastMillis > LED_BLINK_INTERVAL_MS) {
    toggleLed();
    ledLastMillis = millis();
  }
}

void initWifi() {
  irrecv.resume();
  if (Wifienabled) {
    debugln("skip initwifi.");
    return;
  }

  WiFi.disconnect();               // Disconnect any previous connection
  delay(100);                      // Delay for disconnecting
  WiFi.hostname("Wemos_D1_mini");  // Give you device a name on your network
  WiFi.mode(WIFI_STA);             // Set the device as station, to connect to an access point

  // In this case we use Thingspeak as our IoT server to get our sensor data from.
  // If you are not using Thingspeak for accessing data from your sensors you
  // may remove all thingspeak functions from this code.
  ThingSpeak.begin(client);  // Initialize the ThingSpeak library.

  debug("Attempting to connect to SSID: ");
  debugln(WIFI_SSID);

  if (WIFI_PWD == "") {
    Blink = true;
    debugln("No password!");
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PWD);
    unsigned long connectingTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      toggleLed();
      debug(".");
      if (WiFi.status() == WL_WRONG_PASSWORD) {
        Blink = true;
        Wifienabled = false;
        debugln("Wrong password.");
        break;
      }
      if (millis() - connectingTime > 10000) {
        disableWifi();
        Blink = true;
        Wifienabled = false;
        debugln("Connecting timeout.");
        break;
      }
      yield();
    }
    if (WiFi.status() == WL_CONNECTED) {
      Blink = false;
      Wifienabled = true;
      debugln("Connected.");
    }
  }
}

void disableWifi() {
  irrecv.resume();
  Blink = false;
  ledOff();
  // adc_power_release();
  WiFi.disconnect();
  delay(500);
  // WiFi.mode(WIFI_OFF);  // Switch WiFi off
  debugln("Wifi disabled");
  Wifienabled = false;
}

// This function allows the user to set treshold angles, which the servo is not able to move beyond afterwards
// unless overwritten again. The angle values will be saved in non volatile memory of the controller and will
// be reused when the controller is repowered.
void changeLimits() {
  irrecv.resume();
  ledOn();
  bool adjustApproved = false;
  bool scanLoop = true;
  unsigned long loopTime = millis();
  while (scanLoop) {  // bail out loop to avoid accidental limit adjustment activations
    if (irrecv.decode(&results)) {
      if (results.decode_type != NEC) {
        goto end1;
      }
      // press IR_M5 (Smooth) button within 2 seconds after starting the calibration function to tell the controller
      // you really intend to adjust the values of min and max angle. This is basically a safety feature to
      // avoid changing the values by accident.
      if (results.value == IR_M5) {
        adjustApproved = true;
        scanLoop = false;
        debugln("approved");

      } else {
        scanLoop = false;
        debugln("cancelled");
      }
    end1:
      irrecv.resume();
    }
    yield();  // Feed the WDT
    if (millis() - loopTime > 2000) {
      scanLoop = false;
    }
  }
  if (adjustApproved) {  // Proceed when user intends to adjust the lever limits
    relayOn();
    angleCurrent = 90;
    s.write(angleCurrent);  // move the servo to the middle position to indicate that it is ready for adjustment
    scanLoop = true;
    while (scanLoop) {
      if (irrecv.decode(&results)) {
        if (results.decode_type != NEC) {
          goto end2;
        }
        switch (results.value) {
          case IR_UP:  // User confirms position and angle will be saved as max open angle
            angleOpen = angleCurrent;
            EEPROM.write(1, angleOpen);  // save value to non volatile memory
            if (!EEPROM.commit()) {
              Blink = true;
            }
            scanLoop = false;
            break;
          case IR_DOWN:  // User confirms position and angle will be saved as min open angle
            angleClosed = angleCurrent;
            EEPROM.write(0, angleClosed);  // save value to non volatile memory
            if (!EEPROM.commit()) {
              Blink = true;
            }
            scanLoop = false;
            break;
          case IR_OFF:  // User cancels the calibration process
            scanLoop = false;
            break;
          case IR_P5:  // Move up by 5 degrees
            angleCurrent += 5;
            angleCurrent = constrain(angleCurrent, 0, 180);
            s.write(angleCurrent);
            break;
          case IR_P1:  // Move up by 1 degree
            angleCurrent += 1;
            angleCurrent = constrain(angleCurrent, 0, 180);
            s.write(angleCurrent);
            break;
          case IR_M1:  // Move down by 1 degree
            angleCurrent -= 1;
            angleCurrent = constrain(angleCurrent, 0, 180);
            s.write(angleCurrent);
            break;
          case IR_M5:  // Move down by 5 degrees
            angleCurrent -= 5;
            angleCurrent = constrain(angleCurrent, 0, 180);
            s.write(angleCurrent);
            break;
          default:
            break;
        }
      end2:
        irrecv.resume();  // Receive the next value
      }
      yield();  // Feed the WDT
    }
    relayOff();
  }
  ledOff();
}

// Set a flag "envControl" for the main loop to tell if
// automatic climate control should be enabled or disabled
// Note: The automatic control loop is not implemented yet
void changeEnvControl() {
  irrecv.resume();
  ledOn();
  bool scanLoop = true;
  while (scanLoop) {
    if (irrecv.decode(&results)) {
      if (results.decode_type != NEC) {
        goto end;
      }
      if (results.value == IR_ON) {
        envControl = true;
        scanLoop = false;
        debugln("ON");
        goto end;
      }
      if (results.value == IR_OFF) {
        envControl = false;
        scanLoop = false;
        debugln("OFF");
        goto end;
      }
    end:
      irrecv.resume();
    }
    yield();  // Feed the WDT
  }
  ledOff();
}

// this is an unused test function, but contains methods to getting data and timestamps from uploaded sensor data at a thingspeak channel for control feedback
// you could also directly communicate with a microcontroller that collects sensor readings to be independent of an internet connection and external server
void getData() {
  irrecv.resume();
  if (!WiFi.isConnected()) {
    debugln("Enable Wifi first!");
    return;
  }
  if (millis() - dataLastMillis < GETDATA_INTERVAL_SEC * 1000L) {  // avoid spamming the server
    debug("Please wait ");
    debug(GETDATA_INTERVAL_SEC - (millis() - dataLastMillis) / 1000);
    debugln(" s.");
    return;
  }
  float Temp = ThingSpeak.readFloatField(CHANNEL_ID, SENSOR_FIELD, READ_API_KEY);  // Get the most recent sensor measurement from your channel
  int stat = ThingSpeak.getLastReadStatus();                                       // get the status to see if the download was successful
  if (stat != 200) {                                                               // 200 means success
    debugln("Could not read thingspeak.");
    debug("statuscode: ");
    debugln(stat);
  } else {
    debugln(Temp);
    // Get the timestamp from when the data point was uploaded to the server.
    // We can use this value to see if the data point is actually a new one
    debugln(ThingSpeak.readCreatedAt(CHANNEL_ID, READ_API_KEY));
  }
  dataLastMillis = millis();
}

void ledOn() {
  ledState = !HIGH;
  digitalWrite(ledPin, ledState);
}

void ledOff() {
  ledState = !LOW;
  digitalWrite(ledPin, ledState);
}

void toggleLed() {
  irrecv.resume();
  ledState = !ledState;
  digitalWrite(ledPin, ledState);
  // debugln(!ledState);
}

void relayOn() {
  // irrecv.resume();
  relayState = HIGH;
  digitalWrite(relayPin, relayState);
  relayTime = millis();
}

void relayOff() {
  // irrecv.resume();
  relayState = LOW;
  digitalWrite(relayPin, relayState);
}

void openAV() {
  irrecv.resume();
  angleCurrent = angleOpen;
  relayOn();
  delay(100);
  s.write(angleCurrent);
}

void closeAV() {
  irrecv.resume();
  angleCurrent = angleClosed;
  relayOn();
  delay(100);
  s.write(angleCurrent);
}
