// Code by Aabhas Senapati for the Desert Botanical Temperature Sensor Display Project, using Adafruit ESP32 S2 Feather TFT board with a adalogger feather wing, and op-amp circuits for 10 kOhm thermistors.
// Last Edited on 18-08-25
// Github Project Page: https://github.com/aabhassenapati/temperature-display-logger.git

// Data recorded on sheet: https://docs.google.com/spreadsheets/d/1vcmvVcORiZuO4vOsFyO3KGQkez6GWRYInc9CoZGzT1s/edit?usp=sharing
// Calibration data on sheet: https://docs.google.com/spreadsheets/d/1j9i1ZkVB2AnTspQb2jjDu-8glNMxmgINlzlttA89imA/edit?usp=sharing
// Circuit Simulation on: https://tinyurl.com/27u8j87o


// Importing required libraries for the code, install the necesarry libraries through Tools -> Manage libraries

#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP_Google_Sheet_Client.h>
#include "Adafruit_MAX1704X.h"
#include "Adafruit_LC709203F.h"
#include <Adafruit_NeoPixel.h>
#include "Adafruit_TestBed.h"
#include <Adafruit_ST7789.h> 
#include <Fonts/FreeSans12pt7b.h>
#include "RTClib.h"

// For SD/SD_MMC mounting helper
//#include <GS_SDHelper.h>


// Instantiating objects for the required functions and libraries
RTC_PCF8523 rtc;
Adafruit_BME280 bme; // I2C
bool bmefound = false;
extern Adafruit_TestBed TB;
Adafruit_LC709203F lc_bat;
Adafruit_MAX17048 max_bat;
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);



//Declaring variables
  double rf = 11; //kOhm
  double rn = 10; //kOhm
  double vp = 1.01;
  double vin;
  double vn;
  double vout;
  double resistance;
  bool maxfound = false;
  bool lcfound = false;

//Credentials used below can be found on the shared google doc: https://docs.google.com/document/d/1VEV-u6rsc7VFb_hW2kQp7uUem-LySTKIvIandwphV1Q/edit?usp=sharing

// Replace with the credentials from your network
 #define WIFI_SSID ""
 #define WIFI_PASSWORD ""

// Follow steps from https://randomnerdtutorials.com/esp32-datalogging-google-sheets/#demonstration, to create and obtain credentials for Google Service Account, and fill in the required details below.

// Google Project ID
#define PROJECT_ID ""

// Service Account's client email
#define CLIENT_EMAIL ""

// Service Account's private key
const char PRIVATE_KEY[] PROGMEM = "";

// The ID of the spreadsheet where you'll publish the data
const char spreadsheetId[] = "";

// Timer variables
unsigned int sheetlastTime = 0;  
unsigned int sheettimerDelay = 30000; // time for updating a new value to the sheet, currently set to 30 seconds to record a new value.
unsigned int displaylastTime = 0;  
unsigned int displaytimerDelay = 100; // real-display display update time .1 seconds.

// Token Callback function
void tokenStatusCallback(TokenInfo info);

// Variables to hold sensor readings
float airtemp;
float relhum;
float atmpres;
double planttemp1 = 0;
double planttemp2 = 0;
double planttemp3 = 0;
double planttemp4 = 0;
double planttemp5 = 0;
double planttemp6 = 0;

//variables to check Wifi and Internet connectivity status
bool wifiConnectING = false;
unsigned long lastWifiAttemptMillis = 0;
const long wifiReconnectInterval = 10000; // Try reconnecting every 10 seconds
bool internetConnected = false;
bool internetReConnected = false;

// Function to connect to Wifi
void connectToWiFi() {
  Serial.print("Connecting to WiFi ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifiConnectING = true;
  lastWifiAttemptMillis = millis();
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();
}

// Function to check Wifi connectivity.
void checkWifiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    if (!wifiConnectING) {
      connectToWiFi(); // Start connection if not already trying
    } else if (millis() - lastWifiAttemptMillis > wifiReconnectInterval) {
      Serial.println("Still trying to connect... Retrying.");
      connectToWiFi(); // Retry connection
    }
  } else if (wifiConnectING) {
    wifiConnectING = false;
    Serial.print("Connected to WiFi as: ");
    Serial.println(WiFi.localIP());
  }
}

// Function to check internet connection beyond just WiFi
bool checkInternetConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected");
    return false;
  }
  // Try a simple HTTP request
  WiFiClient client;
  if (!client.connect("8.8.8.8", 53)) {
    Serial.println("Internet connection failed");
    return false;
  }
  
  client.stop();
  Serial.println("Internet connection OK");
  return true;
}

// Function to display the Token status for Gsheets API
void tokenStatusCallback(TokenInfo info){
    if (info.status == token_status_error){
        GSheet.printf("Token info: type = %s, status = %s\n", GSheet.getTokenType(info).c_str(), GSheet.getTokenStatus(info).c_str());
        GSheet.printf("Token error: %s\n", GSheet.getTokenError(info).c_str());
    }
    else{
        GSheet.printf("Token info: type = %s, status = %s\n", GSheet.getTokenType(info).c_str(), GSheet.getTokenStatus(info).c_str());
    }
}

// Function to convert voltage readings on a ADC channel into temperature values, by converting from voltage to resistance using opamp-circuit and corresponsing conversions, and then applying resistnace to temperature calibration to obtain temperature.
double voltagetoresistancetotemp(double analogreadmv){
  //Declaring internal variables based on the thermistor circuit.
  double rf = 11; //kOhm 
  double rn = 10; //kOhm
  double vp = 1.01; //V
  double vin;
  double vn;
  double vout;
  double resistance;
    //conversion for the opamp circuit to find out resistance from the output voltage. link to circuit : https://tinyurl.com/2cm8oouf
    vout = (analogreadmv-21.00)/1000;//adjusting for offset in ADC
    vn = (rn/rf)*(((1+(rf/rn))*vp)-vout);//using opamp law, vin = vn; //Volts
    resistance = 1000*((10.3*vn)/(3.3-vn));//kOhm //voltage divider conversion to find resistance
    //applying resistance to temeprature calibration conversion using steinhart hart fit from : https://docs.google.com/spreadsheets/d/1j9i1ZkVB2AnTspQb2jjDu-8glNMxmgINlzlttA89imA/edit?usp=sharing
    double x = log(resistance);
    double term1 = -306.0;
    double term2 = 209.0 * x;
    double term3 = -31.8 * x * x;
    double term4 = 1.41 * x * x * x;
    double planttemp = term1 + term2 + term3 + term4;
    return planttemp;
}

// Setup function that initializes the different peripheral and pins used by the microcontroller.
void setup(){
    // Initalize Serial communication
    Serial.begin(115200);
    Serial.println();
    Serial.println();

    // turn on the TFT / I2C power supply
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    //Configure time
    rtc.begin();
    rtc.start();

    //Input Pins for Thermistors, check out the pinout on https://learn.adafruit.com/adafruit-esp32-s2-tft-feather/pinouts
    pinMode(5, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);


    // Initalize the LCD display
    TB.begin();
    display.init(135, 240);// Init ST7789 240x135
    display.setRotation(3);
    canvas.setFont(&FreeSans12pt7b);
    canvas.setTextColor(ST77XX_WHITE); 

   // Check for the battery status monitoring  IC
   if (lc_bat.begin()) {
    Serial.println("Found LC709203F");
    Serial.print("Version: 0x"); Serial.println(lc_bat.getICversion(), HEX);
    lc_bat.setPackSize(LC709203F_APA_500MAH);
    lcfound = true;
  }
  else {
    Serial.println(F("Couldnt find Adafruit LC709203F?\nChecking for Adafruit MAX1704X.."));
    delay(200);
    if (!max_bat.begin()) {
      Serial.println(F("Couldnt find Adafruit MAX1704X?\nMake sure a battery is plugged in!"));
      //while (1) delay(10);
    }
    Serial.print(F("Found MAX17048"));
    Serial.print(F(" with Chip ID: 0x")); 
    Serial.println(max_bat.getChipID(), HEX);
    maxfound = true;
  } 

  // Check for presence of the BME 280 sensor used for temperature, RH and atmospheric pressure measurements connected via I2C
  if (TB.scanI2CBus(0x77)) {
    Serial.println("BME280 address");

    unsigned status = bme.begin();  
    if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      return;
    }
    Serial.println("BME280 found OK");
    bmefound = true;
  }
    GSheet.printf("ESP Google Sheet Client v%s\n\n", ESP_GOOGLE_SHEET_CLIENT_VERSION);

    // Initiate Connection to Wi-Fi
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
    Serial.print("Connecting to Wi-Fi");
    //Wait 60 seconds after resetting the board to connect to Wifi.
    while ((WiFi.status() != WL_CONNECTED)&&(millis()<=60000))  {
      Serial.print(".");
      delay(1000);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    // Checks Internet Connection
    internetConnected = checkInternetConnection();
    
    // Initiates the Google Sheets logging

    // Set the callback for Google API access token generation status (for debug only)
    GSheet.setTokenCallback(tokenStatusCallback);

    // Set the seconds to refresh the auth token before expire (60 to 3540, default is 300 seconds)
    GSheet.setPrerefreshSeconds(10 * 60);

    // Begin the access token generation for Google API authentication
    GSheet.begin(CLIENT_EMAIL, PROJECT_ID, PRIVATE_KEY);
}

// Main program that is executed over time
void loop(){

    // Checks Wifi and Internet Connection, and determines if signal was lost before and if gsheets authentication needs to be run again.
    checkWifiConnection();
    if(!internetConnected){
      internetConnected = checkInternetConnection(); //Check internet connection
      if(internetConnected){
        internetReConnected = true;
      }
      else
      {
        internetReConnected = false;
      }
    }
    if(internetReConnected)
    {
          // Set the callback for Google API access token generation status (for debug only)
    GSheet.setTokenCallback(tokenStatusCallback);

    // Set the seconds to refresh the auth token before expire (60 to 3540, default is 300 seconds)
    GSheet.setPrerefreshSeconds(10 * 60);

    // Begin the access token generation for Google API authentication
    GSheet.begin(CLIENT_EMAIL, PROJECT_ID, PRIVATE_KEY);
    }

    //This codechunk below looks for the sensor values every .1 seconds and updates it to the display in real-time
    if ((millis() - displaylastTime > displaytimerDelay)) {
    displaylastTime = millis();
    Serial.println("**********************");
    TB.printI2CBusScan();
    canvas.fillScreen(ST77XX_BLACK);
    canvas.setCursor(0, 25);
    canvas.setTextColor(ST77XX_RED);
    canvas.print("Air Temp: ");
    canvas.print(bme.readTemperature());
    canvas.println(" °C");
    canvas.setTextColor(ST77XX_GREEN); 
    canvas.print("Plant Temp: ");

    //conversion for the opamp circuit to find out resistance from the output voltage. link to circuit : https://tinyurl.com/2cm8oouf
    double analogreadmv1 = analogReadMilliVolts(5);
    double analogreadmv2 = analogReadMilliVolts(A1);
    double analogreadmv3 = analogReadMilliVolts(A2);
    double analogreadmv4 = analogReadMilliVolts(A3);
    double analogreadmv5 = analogReadMilliVolts(A4);
    double analogreadmv6 = analogReadMilliVolts(A5);

    planttemp1 = voltagetoresistancetotemp(analogreadmv1);
    planttemp2 = voltagetoresistancetotemp(analogreadmv2);
    planttemp3 = voltagetoresistancetotemp(analogreadmv3);
    planttemp4 = voltagetoresistancetotemp(analogreadmv4);
    planttemp5 = voltagetoresistancetotemp(analogreadmv5);
    planttemp6 = voltagetoresistancetotemp(analogreadmv6);
    canvas.print(planttemp1,1); 
    canvas.println(" °C"); 
    canvas.setTextColor(ST77XX_YELLOW);
    // canvas.print("Rel. Humidity: ");
    // canvas.print(bme.readHumidity());
    // canvas.println(" %");
    canvas.print("Battery: ");
    canvas.setTextColor(ST77XX_WHITE);
    if (lcfound == true) {
      canvas.print(lc_bat.cellVoltage(), 1);
      canvas.print(" V  /  ");
      canvas.print(lc_bat.cellPercent(), 0);
      canvas.println("%");
    }
    else {
      canvas.print(max_bat.cellVoltage(), 1);
      canvas.print(" V  /  ");
      canvas.print(max_bat.cellPercent(), 0);
      canvas.println("%");
    }
    canvas.setTextColor(ST77XX_BLUE); 
    canvas.print("I2C: ");
    canvas.setTextColor(ST77XX_WHITE);
    for (uint8_t a=0x01; a<=0x7F; a++) {
      if (TB.scanI2CBus(a, 0))  {
        canvas.print("0x");
        canvas.print(a, HEX);
        canvas.print(", ");
      }
    }
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);
  }

  // This code chunk below looks for internet connection, and then updates the current reading to google sheets every 30 seconds, if connection is lost, it does not log those datapoints on gsheets.
  if(internetConnected){
  bool ready = GSheet.ready(); // Call ready() repeatedly in loop for authentication checking and processing
    if ((ready && (millis() - sheetlastTime > sheettimerDelay))){
        sheetlastTime = millis();

        FirebaseJson response;

        Serial.println("\nAppend spreadsheet values...");
        Serial.println("----------------------------");

        FirebaseJson valueRange;

        // New BME280 sensor readings
        airtemp = bme.readTemperature();
        //temp = 1.8*bme.readTemperature() + 32;
        relhum = bme.readHumidity();
        atmpres = bme.readPressure()/100.0F;
        // Get timestamp
        DateTime now = rtc.now();

        valueRange.add("majorDimension", "COLUMNS");
        valueRange.set("values/[0]/[0]", now.unixtime());
        valueRange.set("values/[1]/[0]", airtemp);
        valueRange.set("values/[2]/[0]", relhum);
        valueRange.set("values/[3]/[0]", atmpres);
        valueRange.set("values/[4]/[0]", planttemp1);
        valueRange.set("values/[5]/[0]", planttemp2);
        valueRange.set("values/[6]/[0]", planttemp3);
        valueRange.set("values/[7]/[0]", planttemp4);
        valueRange.set("values/[8]/[0]", planttemp5);
        valueRange.set("values/[9]/[0]", planttemp6);
        valueRange.set("values/[10]/[0]", max_bat.cellVoltage());
        


        // For Google Sheet API ref doc, go to https://developers.google.com/sheets/api/reference/rest/v4/spreadsheets.values/append
        // Append values to the spreadsheet
        bool success = GSheet.values.append(&response /* returned response */, spreadsheetId /* spreadsheet Id to append */, "Sheet1!A1" /* range to append */, &valueRange /* data range to append */);
        if (success){
            response.toString(Serial, true);
            valueRange.clear();
        }
        else{
            Serial.println(GSheet.errorReason());
        }
        Serial.println();
        Serial.println(ESP.getFreeHeap());
        internetConnected = checkInternetConnection();
    }
  }

  return;
}
