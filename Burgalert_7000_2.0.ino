#include "Adafruit_FONA.h"
#include "LowPower.h"
#include <SoftwareSerial.h>


#define FONA_PWRKEY 6
#define FONA_RST 7
#define FONA_TX 10
#define FONA_RX 11

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();


uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
char imei[16] = {0}; // Use this for device ID
char replybuffer[255]; // Large buffer for replies
uint8_t type;
uint8_t counter = 0;
float latitude, longitude, speed_kph, heading, altitude, second;
uint16_t year;
uint8_t month, day, hour, minute;
char URL[100]; // Make sure this is long enough for your request URL
const int wakeUpPin = 2; // Use pin 2 to wake up the Uno/Mega

const char * phone_number = "15097240325"; // Include country code, only numbers
char * text_message = "Test 12/14/21 "; // Change to suit your needs


void setup()
{
  Serial.begin(9600);
  Serial.println(F("___Begining of the program, Void Setup, line 30___"));
  pinMode(wakeUpPin, INPUT); // For the interrupt wake-up to work
  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH); // Default state
  pinMode(FONA_PWRKEY, OUTPUT);
  //Serial.println(F("___End of the Void Setup now to powerOn True, line 36___"));
  //  powerOn(true); // Power on the module
  //Serial.println(F("___Back up to Void Setup, line 38___"));
  moduleSetup(); // Establish first-time serial comm and print IMEI
  //Serial.println(F("___Back up to Void Setup, line 40___"));
  //  fona.setNetworkSettings(F("hologram")); // For Hologram SIM card, change appropriately
  fona.setFunctionality(1); // AT+CFUN=1  //From Other Program
  //Serial.println(F("___End of Void Setup, line 43___"));
}

void loop()
{
  // Our goal is to wake up whenever the motion detector tells us to,
  // then go back to sleep after sending a message or HTTP request.
  Serial.println(F("___This is the begining of Void Loop, line 50___"));

  // Disable external pin interrupt until we finishd doing stuff, then sleep afterward
  detachInterrupt(digitalPinToInterrupt(wakeUpPin));

  //  powerOn(true); // Power on the module
  //  moduleSetup(); // Establish first-time serial comm and print IMEI

  // First connect to the cell network and verify connection
  // If unsuccessful, keep retrying every 2s until a connection is made
  Serial.println(F("___This is middle of Void Loop just before cell network connection verificaiton, line 60___"));

  //  while (!netStatus())
  //  {
  //    Serial.println(F("Failed to connect to cell network, retrying..."));
  //    delay(2000); // Retry every 2s
  //  }

  Serial.println(F("Connected to cell network!"));

  Serial.println(F("___This is middle of Void Loop just before GPS Location, line 68___"));


//  acquireGPSLoop()

  //  sendMessage( phone_number,  text_message)



  Serial.println(F("___This is Void Loop, line 76___"));
  sleepDevice(); // Put the MCU and SIM7000 shield to sleep until more motion is detected!
}

void acquireGPSLoop() {
  //   check: loop until gpsLocation acquired
  // while (!gpsLocation())
  // {
  //   Serial.println(F("Failed to aquire GPS Location, retrying..."));
  //   delay(2000); // Retry every 2s
  // }
  Serial.println(F("Connected to aquire GPS Location!"));
}

void sendMessage(String phone_number, String text_message) {
  // Send a text to your phone!
  if (!fona.sendSMS(phone_number, text_message))
  {
    Serial.println(F("Failed to send text!"));
  }
  //  else
  {
    Serial.println(F("Sent text alert!"));
  }
}

// Power on/off the module
void powerOn(bool state)
{
  if (state) {
    Serial.println(F("___This is the start of Void powerOn, if State, line 84___"));
    Serial.println(F("Turning on SIM7000..."));
    digitalWrite(FONA_PWRKEY, LOW);
    delay(100); // Turn on module
    digitalWrite(FONA_PWRKEY, HIGH);
    delay(4500); // Give enough time for the module to boot up before communicating with it
    Serial.println(F("___This is the end of Void powerOn, If State, line 90___"));
  }
  else
  {
    Serial.println(F("___This is Void powerOn, Else, line 94___"));
    Serial.println(F("Turning off SIM7000..."));
    fona.powerDown(); // Turn off module
  }
}

void moduleSetup()
{
  Serial.println(F("___This is the top of Void moduleSetup, line 101___"));
  fonaSS.begin(115200); // Default SIM7000 shield baud rate

  Serial.println(F("Configuring to 9600 baud"));
  fonaSS.println("AT+IPR=9600"); // Set baud rate
  delay(100); // Short pause to let the command run
  fonaSS.begin(9600);
  if (! fona.begin(fonaSS))
  {
    Serial.println(F("Couldn't find FONA"));
    while (1); // Don't proceed if it couldn't find the device
  }

  type = fona.type();
  Serial.println(F("___This is Void moduleSetup, Type, line 115___"));
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type)
  {
    case SIM7000:
      Serial.println(F("___This is Void moduleSetup, Case, line 121___"));
      Serial.println(F("SIM7000G (Global)")); break;
    default:
      Serial.println(F("???")); break;
  }

  // Print module IMEI number.
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0)
  {
    Serial.println(F("___This is Void moduleSetup, IMEI, line 131___"));
    Serial.print("Module IMEI: "); Serial.println(imei);
  }

  {
    Serial.print("Turning GPS ON");
    Serial.println();
    delay(100);
    if (!fona.enableGPS(true))
      Serial.println(F("Failed to turn on"));
    Serial.print("Now turning GPRS ON");
    Serial.println();
    delay(100);
    if (!fona.enableGPRS(true))
      Serial.println(F("Failed to turn on"));
    Serial.println();
  }

}

bool netStatus()
{
  int n = fona.getNetworkStatus();

  Serial.print(F("Network status ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));

  if (!(n == 1 || n == 5)) return false;
  else return true;
}





void sleepDevice()
{
  Serial.println(F("___This is the start of Void sleepDevice, line 174___"));
  Serial.println(F("Turning off SIM7000..."));
  powerOn(false); // Turn off the SIM7000 module

  // After everything's said and done, attach the interrupt
  // Set the PIR sensor jumper to "L" for normal operation. This
  // means the output will go from LOW to HIGH when motion is detected!
  attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, RISING); // When the PIR sensor pin goes low to high, MCU wakes up!
  Serial.println(F("___This is the middle of Void sleepDevice, line 182___"));
  Serial.println(F("Sleeping MCU..."));
  delay(100); // Needed for the serial print above

  // Power down MCU with ADC and BOD module disabled
  // Wake up when wake up pin is low
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void wakeUp()
{
  // Just a handler for the interrupt
  // This runs right after the MCU wakes up, then it goes back to loop()
}
