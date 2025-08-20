// Code by Aabhas Senapati for the Desert Botanical Temperature Sensor Display Project, using Adafruit ESP32 S2 Feather TFT board with a adalogger feather wing, and op-amp circuits for 10 kOhm thermistors.
// Last Edited on 19-08-25 - Added combined SD Card and Google Sheets logging with 5-minute averaging every 30 seconds, and turning off dispplay in the night.
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
#include "Adafruit_TestBed.h"
#include <Adafruit_ST7789.h> 
#include <Fonts/FreeSans12pt7b.h>
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>
#include <math.h>

// For SD/SD_MMC mounting helper
//#include <GS_SDHelper.h>

// SD Card pin - Updated for ESP32-S2 TFT Feather with Adalogger FeatherWing
#define SD_CS_PIN 10  // ESP32-S2 default CS pin for SD card

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

// Timer variables for display
unsigned int displaylastTime = 0;  
unsigned int displaytimerDelay = 100; // real-display display update time .1 seconds.

// Combined logging system variables
unsigned int measurementLastTime = 0;  
unsigned int measurementInterval = 30000; // Measure every 30 seconds
unsigned int logLastTime = 0;  
unsigned int logInterval = 300000; // Log every 5 minutes (300,000 ms)

// Averaging variables (shared for both logging systems)
const int MEASUREMENTS_PER_LOG = 10; // 5 minutes / 30 seconds = 10 measurements
int measurementCount = 0;

// Arrays to store individual measurements for standard deviation calculation
double airtempMeasurements[MEASUREMENTS_PER_LOG];
double relhumMeasurements[MEASUREMENTS_PER_LOG];
double atmpresMeasurements[MEASUREMENTS_PER_LOG];
double planttemp1Measurements[MEASUREMENTS_PER_LOG];
double planttemp2Measurements[MEASUREMENTS_PER_LOG];
double planttemp3Measurements[MEASUREMENTS_PER_LOG];
double planttemp4Measurements[MEASUREMENTS_PER_LOG];
double planttemp5Measurements[MEASUREMENTS_PER_LOG];
double planttemp6Measurements[MEASUREMENTS_PER_LOG];
double batteryVoltageMeasurements[MEASUREMENTS_PER_LOG];

// Sum variables for averaging (shared for both logging systems)
double airtempSum = 0;
double relhumSum = 0;
double atmpresSum = 0;
double planttemp1Sum = 0;
double planttemp2Sum = 0;
double planttemp3Sum = 0;
double planttemp4Sum = 0;
double planttemp5Sum = 0;
double planttemp6Sum = 0;
double batteryVoltageSum = 0;

// SD Card configuration
File logfile;  // Keep file handle open
bool sdCardAvailable = false;

// Create unique filename 
char filename[20];

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

// Function to calculate standard deviation
double calculateStandardDeviation(double measurements[], int count, double mean) {
  if (count <= 1) return 0.0;
  
  double sumSquaredDifferences = 0.0;
  for (int i = 0; i < count; i++) {
    double difference = measurements[i] - mean;
    sumSquaredDifferences += difference * difference;
  }
  
  // Calculate sample standard deviation (n-1 in denominator)
  double variance = sumSquaredDifferences / (count - 1);
  return sqrt(variance);
}

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

// Function to initialize SD card
bool initializeSDCard() {
  delay(2000);
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    Serial.print("Tried CS pin: ");
    Serial.println(SD_CS_PIN);
    return false;
  }
  Serial.println("SD card initialized successfully.");
  

  strcpy(filename, "/TEMPLOG00.CSV");
  for (uint8_t i = 0; i < 100; i++) {
    filename[8] = '0' + i/10;
    filename[9] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  
  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create ");
    Serial.println(filename);
    return false;
  }
  
  Serial.print("Writing to ");
  Serial.println(filename);
  
  // Write CSV header and flush immediately
  logfile.println("Timestamp,AirTemp_C,AirTemp_StdDev,Humidity_%,Humidity_StdDev,Pressure_hPa,Pressure_StdDev,PlantTemp1_C,PlantTemp1_StdDev,PlantTemp2_C,PlantTemp2_StdDev,PlantTemp3_C,PlantTemp3_StdDev,PlantTemp4_C,PlantTemp4_StdDev,PlantTemp5_C,PlantTemp5_StdDev,PlantTemp6_C,PlantTemp6_StdDev,BatteryVoltage_V,BatteryVoltage_StdDev");
  logfile.flush(); // Make sure header is written
  
  Serial.println("CSV header written to SD card with standard deviation columns.");
  return true;
}

// Function to log data to SD card with standard deviations
bool logToSDCard(DateTime timestamp, double avgAirtemp, double stdAirtemp, double avgRelhum, double stdRelhum, 
                 double avgAtmpres, double stdAtmpres, double avgPlanttemp1, double stdPlanttemp1, 
                 double avgPlanttemp2, double stdPlanttemp2, double avgPlanttemp3, double stdPlanttemp3, 
                 double avgPlanttemp4, double stdPlanttemp4, double avgPlanttemp5, double stdPlanttemp5, 
                 double avgPlanttemp6, double stdPlanttemp6, double avgBatteryVoltage, double stdBatteryVoltage) {
  
  sdCardAvailable = SD.exists(filename);
  if (!sdCardAvailable || !logfile) return false;
  
  logfile.print(timestamp.unixtime());
  logfile.print(",");
  logfile.print(avgAirtemp, 2);
  logfile.print(",");
  logfile.print(stdAirtemp, 3);
  logfile.print(",");
  logfile.print(avgRelhum, 2);
  logfile.print(",");
  logfile.print(stdRelhum, 3);
  logfile.print(",");
  logfile.print(avgAtmpres, 2);
  logfile.print(",");
  logfile.print(stdAtmpres, 3);
  logfile.print(",");
  logfile.print(avgPlanttemp1, 2);
  logfile.print(",");
  logfile.print(stdPlanttemp1, 3);
  logfile.print(",");
  logfile.print(avgPlanttemp2, 2);
  logfile.print(",");
  logfile.print(stdPlanttemp2, 3);
  logfile.print(",");
  logfile.print(avgPlanttemp3, 2);
  logfile.print(",");
  logfile.print(stdPlanttemp3, 3);
  logfile.print(",");
  logfile.print(avgPlanttemp4, 2);
  logfile.print(",");
  logfile.print(stdPlanttemp4, 3);
  logfile.print(",");
  logfile.print(avgPlanttemp5, 2);
  logfile.print(",");
  logfile.print(stdPlanttemp5, 3);
  logfile.print(",");
  logfile.print(avgPlanttemp6, 2);
  logfile.print(",");
  logfile.print(stdPlanttemp6, 3);
  logfile.print(",");
  logfile.print(avgBatteryVoltage, 3);
  logfile.print(",");
  logfile.println(stdBatteryVoltage, 4);
  
  logfile.flush();
  
  Serial.println("Data logged to SD card successfully");
  return true;
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
    
    // Check for division by zero
    if ((3.3-vn) <= 0) {
      Serial.println("Warning: Division by zero in resistance calculation");
      return NAN;
    }
    
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

// Function to check if display should be off (7:30 PM to 4:30 AM)
bool shouldDisplayBeOff() {
  DateTime now = rtc.now();
  int currentHour = now.hour();
  int currentMinute = now.minute();
  
  // Convert current time to minutes since midnight for easier comparison
  int currentTimeMinutes = currentHour * 60 + currentMinute;
  
  // 7:30 PM = 19:30 = 19*60 + 30 = 1170 minutes
  // 4:30 AM = 4:30 = 4*60 + 30 = 270 minutes
  int displayOffStart = 19 * 60 + 30;  // 7:30 PM
  int displayOffEnd = 4 * 60 + 30;     // 4:30 AM
  
  // Check if current time is in the sleep period
  // Since this crosses midnight, we need to handle it specially
  if (displayOffStart > displayOffEnd) {
    // Sleep period crosses midnight (19:30 to 04:30 next day)
    return (currentTimeMinutes >= displayOffStart || currentTimeMinutes < displayOffEnd);
  } else {
    // Sleep period doesn't cross midnight (shouldn't happen with 19:30-04:30)
    return (currentTimeMinutes >= displayOffStart && currentTimeMinutes < displayOffEnd);
  }
}
void handleCombinedLogging() {
  // Take measurements every 30 seconds
  if (millis() - measurementLastTime > measurementInterval) {
    measurementLastTime = millis();
    
    // Take new sensor readings
    double currentAirtemp = bme.readTemperature();
    double currentRelhum = bme.readHumidity();
    double currentAtmpres = bme.readPressure()/100.0;
    double currentBatteryVoltage;
    
    // Get battery voltage from the appropriate sensor
    if (lcfound) {
      currentBatteryVoltage = lc_bat.cellVoltage();
    } else {
      currentBatteryVoltage = max_bat.cellVoltage();
    }
    
    // Store individual measurements in arrays for standard deviation calculation
    if (measurementCount < MEASUREMENTS_PER_LOG) {
      airtempMeasurements[measurementCount] = currentAirtemp;
      relhumMeasurements[measurementCount] = currentRelhum;
      atmpresMeasurements[measurementCount] = currentAtmpres;
      planttemp1Measurements[measurementCount] = planttemp1;  // These are updated in display loop
      planttemp2Measurements[measurementCount] = planttemp2;
      planttemp3Measurements[measurementCount] = planttemp3;
      planttemp4Measurements[measurementCount] = planttemp4;
      planttemp5Measurements[measurementCount] = planttemp5;
      planttemp6Measurements[measurementCount] = planttemp6;
      batteryVoltageMeasurements[measurementCount] = currentBatteryVoltage;
    }
    
    // Add to running sums
    airtempSum += currentAirtemp;
    relhumSum += currentRelhum;
    atmpresSum += currentAtmpres;
    planttemp1Sum += planttemp1;
    planttemp2Sum += planttemp2;
    planttemp3Sum += planttemp3;
    planttemp4Sum += planttemp4;
    planttemp5Sum += planttemp5;
    planttemp6Sum += planttemp6;
    batteryVoltageSum += currentBatteryVoltage;
    
    measurementCount++;
    
    Serial.print("Measurement ");
    Serial.print(measurementCount);
    Serial.print("/");
    Serial.print(MEASUREMENTS_PER_LOG);
    Serial.print(" - Temp: ");
    Serial.print(currentAirtemp);
    Serial.print("°C, Humidity: ");
    Serial.print(currentRelhum);
    Serial.println("%");
  }
  
  // Log averaged values every 5 minutes to both systems
  if (measurementCount >= MEASUREMENTS_PER_LOG && (millis() - logLastTime > logInterval)) {
    logLastTime = millis();
    
    // Calculate averages
    double avgAirtemp = airtempSum / measurementCount;
    double avgRelhum = relhumSum / measurementCount;
    double avgAtmpres = atmpresSum / measurementCount;
    double avgPlanttemp1 = planttemp1Sum / measurementCount;
    double avgPlanttemp2 = planttemp2Sum / measurementCount;
    double avgPlanttemp3 = planttemp3Sum / measurementCount;
    double avgPlanttemp4 = planttemp4Sum / measurementCount;
    double avgPlanttemp5 = planttemp5Sum / measurementCount;
    double avgPlanttemp6 = planttemp6Sum / measurementCount;
    double avgBatteryVoltage = batteryVoltageSum / measurementCount;
    
    // Calculate standard deviations
    double stdAirtemp = calculateStandardDeviation(airtempMeasurements, measurementCount, avgAirtemp);
    double stdRelhum = calculateStandardDeviation(relhumMeasurements, measurementCount, avgRelhum);
    double stdAtmpres = calculateStandardDeviation(atmpresMeasurements, measurementCount, avgAtmpres);
    double stdPlanttemp1 = calculateStandardDeviation(planttemp1Measurements, measurementCount, avgPlanttemp1);
    double stdPlanttemp2 = calculateStandardDeviation(planttemp2Measurements, measurementCount, avgPlanttemp2);
    double stdPlanttemp3 = calculateStandardDeviation(planttemp3Measurements, measurementCount, avgPlanttemp3);
    double stdPlanttemp4 = calculateStandardDeviation(planttemp4Measurements, measurementCount, avgPlanttemp4);
    double stdPlanttemp5 = calculateStandardDeviation(planttemp5Measurements, measurementCount, avgPlanttemp5);
    double stdPlanttemp6 = calculateStandardDeviation(planttemp6Measurements, measurementCount, avgPlanttemp6);
    double stdBatteryVoltage = calculateStandardDeviation(batteryVoltageMeasurements, measurementCount, avgBatteryVoltage);
    
    // Get timestamp
    DateTime now = rtc.now();
    
    Serial.println("\n==============================");
    Serial.println("LOGGING 5-MINUTE AVERAGES WITH STANDARD DEVIATIONS");
    Serial.print("Measurements averaged: ");
    Serial.println(measurementCount);
    Serial.print("Air Temp: ");
    Serial.print(avgAirtemp, 2);
    Serial.print(" ± ");
    Serial.print(stdAirtemp, 3);
    Serial.println("°C");
    Serial.print("Humidity: ");
    Serial.print(avgRelhum, 2);
    Serial.print(" ± ");
    Serial.print(stdRelhum, 3);
    Serial.println("%");
    Serial.println("==============================");
    
    // Attempt to log to both systems
    bool googleSheetsSuccess = false;
    bool sdCardSuccess = false;
    
    // 1. Try Google Sheets logging (if internet connected)
    if (internetConnected) {
      bool ready = GSheet.ready();
      if (ready) {
        FirebaseJson response;
        FirebaseJson valueRange;

        // Set up the values for Google Sheets (keeping original format for compatibility)
        airtemp = avgAirtemp;
        relhum = avgRelhum;
        atmpres = avgAtmpres;

        valueRange.add("majorDimension", "COLUMNS");
        valueRange.set("values/[0]/[0]", now.unixtime());
        valueRange.set("values/[1]/[0]", avgAirtemp);
        valueRange.set("values/[2]/[0]", stdAirtemp);
        valueRange.set("values/[3]/[0]", avgRelhum);
        valueRange.set("values/[4]/[0]", stdRelhum);
        valueRange.set("values/[5]/[0]", avgAtmpres);
        valueRange.set("values/[6]/[0]", stdAtmpres);
        valueRange.set("values/[7]/[0]", avgPlanttemp1);
        valueRange.set("values/[8]/[0]", stdPlanttemp1);
        valueRange.set("values/[9]/[0]", avgPlanttemp2);
        valueRange.set("values/[10]/[0]", stdPlanttemp2);
        valueRange.set("values/[11]/[0]", avgPlanttemp3);
        valueRange.set("values/[12]/[0]", stdPlanttemp3);
        valueRange.set("values/[13]/[0]", avgPlanttemp4);
        valueRange.set("values/[14]/[0]", stdPlanttemp4);
        valueRange.set("values/[15]/[0]", avgPlanttemp5);
        valueRange.set("values/[16]/[0]", stdPlanttemp5);
        valueRange.set("values/[17]/[0]", avgPlanttemp6);
        valueRange.set("values/[18]/[0]", stdPlanttemp6);
        valueRange.set("values/[19]/[0]", avgBatteryVoltage);
        valueRange.set("values/[20]/[0]", stdBatteryVoltage);

        // Append values to Google Sheets
        bool success = GSheet.values.append(&response, spreadsheetId, "Data_raw_new!A1", &valueRange);
        if (success) {
          response.toString(Serial, true);
          valueRange.clear();
          googleSheetsSuccess = true;
          Serial.println("✓ Data logged to Google Sheets successfully (with standard deviations)");
        } else {
          Serial.print("✗ Google Sheets error: ");
          Serial.println(GSheet.errorReason());
        }
        
        internetConnected = checkInternetConnection();
      } else {
        Serial.println("✗ Google Sheets not ready");
      }
    } else {
      Serial.println("⚠ No internet connection - skipping Google Sheets");
    }
    
    // 2. Try SD Card logging with standard deviations
    sdCardSuccess = logToSDCard(now, avgAirtemp, stdAirtemp, avgRelhum, stdRelhum, avgAtmpres, stdAtmpres, 
                               avgPlanttemp1, stdPlanttemp1, avgPlanttemp2, stdPlanttemp2, avgPlanttemp3, stdPlanttemp3,
                               avgPlanttemp4, stdPlanttemp4, avgPlanttemp5, stdPlanttemp5, avgPlanttemp6, stdPlanttemp6, 
                               avgBatteryVoltage, stdBatteryVoltage);
    
    // Summary of logging results
    Serial.println("------------------------------");
    Serial.print("Google Sheets: ");
    Serial.println(googleSheetsSuccess ? "SUCCESS" : "FAILED");
    Serial.print("SD Card: ");
    Serial.println(sdCardSuccess ? "SUCCESS" : "FAILED");
    Serial.println("==============================\n");
    
    // Always reset everything regardless of logging success (skip failed datapoints)
    airtempSum = 0;
    relhumSum = 0;
    atmpresSum = 0;
    planttemp1Sum = 0;
    planttemp2Sum = 0;
    planttemp3Sum = 0;
    planttemp4Sum = 0;
    planttemp5Sum = 0;
    planttemp6Sum = 0;
    batteryVoltageSum = 0;
    measurementCount = 0;
    
    // Clear measurement arrays
    memset(airtempMeasurements, 0, sizeof(airtempMeasurements));
    memset(relhumMeasurements, 0, sizeof(relhumMeasurements));
    memset(atmpresMeasurements, 0, sizeof(atmpresMeasurements));
    memset(planttemp1Measurements, 0, sizeof(planttemp1Measurements));
    memset(planttemp2Measurements, 0, sizeof(planttemp2Measurements));
    memset(planttemp3Measurements, 0, sizeof(planttemp3Measurements));
    memset(planttemp4Measurements, 0, sizeof(planttemp4Measurements));
    memset(planttemp5Measurements, 0, sizeof(planttemp5Measurements));
    memset(planttemp6Measurements, 0, sizeof(planttemp6Measurements));
    memset(batteryVoltageMeasurements, 0, sizeof(batteryVoltageMeasurements));
    
    if (!googleSheetsSuccess && !sdCardSuccess) {
      Serial.println("⚠ Both logging methods failed - datapoint skipped");
    }
    
    Serial.println(ESP.getFreeHeap());
  }
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

  // Initialize SD card with improved error handling
  Serial.println("Attempting to initialize SD card...");
  sdCardAvailable = initializeSDCard();
  if (!sdCardAvailable) {
    Serial.println("SD card not available - will only use Google Sheets logging");
    Serial.println("Check SD card connection and ensure it's properly formatted");
  }

  // Initalize the LCD display
  TB.begin();
  display.init(135, 240);// Init ST7789 240x135
  display.setRotation(3);
  canvas.setFont(&FreeSans12pt7b);
  canvas.setTextColor(ST77XX_WHITE); 

  // Check for the battery status monitoring IC
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
    Serial.println("BME280 address detected");

    unsigned status = bme.begin();  
    if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      bmefound = false; // Ensure it's set to false if initialization fails
    } else {
      Serial.println("BME280 found and initialized OK");
      bmefound = true;
    }
  } else {
    Serial.println("BME280 not detected on I2C bus");
    bmefound = false;
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
    internetReConnected = false; // Reset the flag after handling reconnection
  }

  //This codechunk below looks for the sensor values every .1 seconds and updates it to the display in real-time
  if ((millis() - displaylastTime > displaytimerDelay)) {
    displaylastTime = millis();
    Serial.println("**********************");
    TB.printI2CBusScan();
    
    // Check if display should be off (7:30 PM to 4:30 AM)
    bool displayShouldBeOff = shouldDisplayBeOff();
    
    if (displayShouldBeOff) {
      // Turn off display backlight and clear screen
      pinMode(TFT_BACKLITE, OUTPUT);
      digitalWrite(TFT_BACKLITE, LOW);
      canvas.fillScreen(ST77XX_BLACK);
      display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
      Serial.println("Display OFF - Sleep hours (7:30 PM - 4:30 AM)");
    } else {
      // Normal display operation
      canvas.fillScreen(ST77XX_BLACK);
      canvas.setCursor(0, 25);
      canvas.setTextColor(ST77XX_RED);
      canvas.print("Air Temp: ");
      if (bmefound) {
        float tempReading = bme.readTemperature();
        if (!isnan(tempReading)) {
          canvas.print(tempReading, 1);
        } else {
          canvas.print("ERR");
        }
      } else {
        canvas.print("N/A");
      }
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
      
      // Add SD card status indicator to display
      canvas.setTextColor(ST77XX_CYAN);
      canvas.print("SD: ");
      canvas.setTextColor(sdCardAvailable ? ST77XX_GREEN : ST77XX_RED);
      canvas.println(sdCardAvailable ? "OK" : "FAIL");
      
      display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
      pinMode(TFT_BACKLITE, OUTPUT);
      digitalWrite(TFT_BACKLITE, HIGH);
    }
    
    // Always read thermistor values even when display is off (needed for logging)
    if (displayShouldBeOff) {
      // Read thermistor values even when display is off for logging purposes
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
    }
  }

  // Handle combined logging with standard deviation calculation
  handleCombinedLogging();

  return;
}