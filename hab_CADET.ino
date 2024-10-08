// Uses GPS and SD breakout board
// GPS generates data at a specified rate
// and data is written to SD card

// GPSport library gives more accurate readings than Adafruit GPS
#include <NMEAGPS.h>
#include <GPSport.h>
// Libaries to write to SD card
#include <SPI.h>
#include <SD.h>
// Read temperature sensors
#include <OneWire.h>
#include <DallasTemperature.h>
// power sensor library
#include <Adafruit_INA260.h>
// barometric pressure sensor
#include <Wire.h>
#include "BlueDot_BME280_TSL2591.h"
BlueDot_BME280_TSL2591 bme280;
bool bmeFound = false;

// GPS configuration
NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values

// detects voltage, current, and power
Adafruit_INA260 powerSensor = Adafruit_INA260();
bool powerSensorFound = false;

// set sample time in milliseconds
const int readTime = 1000;

// keeps track of burn wire burn times
int burnCount = 0;
const int burnCycles = 3;
const int burnTime = 4000;
const int burnDelayTime = 2000;
uint32_t burnTimer = millis();
uint32_t burnDelayTimer = millis();
const float burnAltitudeLimit = 600;
// const float burnAltitudeLimit = 20;
// keeps track of if we have used burn cycle
// to release from the balloon or not
// (does not include burn cycle because of button press)
bool burnReleaseComplete = false;
bool releaseBurnInProgress = false;
bool readOnce = false;

// geofence
const bool useGeofence = true; // turn on if using geofence to drop payload
bool outsideGeofence = false;
struct Geofence {
  // p1 must always be upper point for algorithm to know
  // what is inside or outside fence
  float p1Lat;
  float p1Lon;
  float p2Lat;
  float p2Lon;
};

// Geofence geofenceLower = {43.0869, -83.29721, 43.08184, -83.30335};
Geofence geofenceLower = {43.011145, -82.781035, 41.728951, -83.876630};
Geofence geofenceUpper = {43.854645, -82.983770, 43.011145, -82.781035};

// Geofence geofences[] = {geofenceLower, geofenceUpper, geofenceOhio};

// burn wire pin
const int burnWirePin = 9;

// input tactile button
const int inputButtonPin = 10;

// Both temperature sensor chips are plugged into same pin
#define TEMP_WIRE_BUS 8

// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(TEMP_WIRE_BUS);
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature onewireTempSensors(&oneWire);
/********************************************************************/

// analog temp sensor
const bool useAnalogTemp = false;
const int analogTempPin = 8;
int analogTempInput = 0;

// defines pin used to write to SD card
#define WRITE_PIN 53

// set to true if you want output to Serial monitor
#define DEBUG false

// don't include file type for now, that is
// determined at initialization, as well as
// appending count to file name if needed
String logFileName = "LOG";
String dataFileName = "DATA";

// Keep track of last logged valid gps & barometric pressure data
String lastValidDateTime;
// how many readings to keep track of
const int readCount = 10;
// points to compare to max altitude to determine if falling
const int compareCount = 4;
float barAltitudeReadings[readCount] = {0};
float gpsAltitudeReadings[readCount] = {0};
float gpsHighestAltitude = 0.0;
float barHighestAltitude = 0.0;
int gpsHighestAltitudeIndex = -1;
int barHighestAltitudeIndex = -1;
const int minAltitude = 750; // Must be at least this far before dropping (so that the parachute is deployed)
const int fallingAltitude = 60; // must drop this far to be considered falling (in meters)
bool isFalling = false;

uint32_t timer = millis();
bool sd_init_successful = false;

void setup() {
  if (DEBUG) {
    // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
    // also spit it out
    Serial.begin(115200);
    delay(5000);
    debugPrint("Serial begin");
  }
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  // initialize burn wire as off
  digitalWrite(burnWirePin, LOW);

  // see if the card is present and can be initialized
  // keep running either way, because data is being recorded
  // on GPS chip
  if (!SD.begin(WRITE_PIN)) {
    debugPrint("Card failed, or not present");
    // keep running because data is stored on GPS chip as well
    sd_init_successful = false;
  } else {
    debugPrint("Card initialized.");
    sd_init_successful = true;
    // Write first line to SD card
    initDataFile();
    initLogFile();
  }
  // initialize burn wire as output
  pinMode(burnWirePin, OUTPUT);
  // initialize pushbutton pin as input
  pinMode(inputButtonPin, INPUT);
  setupSensors();
}

void initDataFile()
{
  int count = 0;
  String testFileName = dataFileName + "_00" + String(count) + ".csv";
  while (SD.exists(testFileName)) {
    count++;
    if (count < 10) {
      testFileName = dataFileName + "_00" + String(count);
    } else if (count < 100) {
      testFileName = dataFileName + "_0" + String(count);
    } else {
      testFileName = dataFileName + "_" + String(count);
    }
    testFileName = testFileName + ".csv";
  }
  dataFileName = testFileName;
  // dataFileName = "data.csv";
  debugPrint("file name found: " + dataFileName);
  // TODO: Make this dependent on what sensors exist 
  // (or on flags in beginning setup)
  // String titleStr = "Time,Date,Outside Temp,Inside Temp,";
  String titleStr = "Time,Date,Inside Temp (C),";
  titleStr += "Barometric Pressure,Barometric Temperature (C),";
  titleStr += "Barometric Humidity,Barometric Altitude (m),";
  titleStr += "Current (mA), BusVoltage (mV), Power (mW),";
  titleStr += "Latitude,Longitude,Speed (kph),Altitude (m),Satellites";
  writeToDataFile(titleStr);
}

void initLogFile()
{
  int count = 0;
  String testFileName = logFileName + "_00" + String(count) + ".txt";
  while (SD.exists(testFileName)) {
    count++;
    if (count < 10) {
      testFileName = logFileName + "_00" + String(count);
    } else if (count < 100) {
      testFileName = logFileName + "_0" + String(count);
    } else {
      testFileName = logFileName + "_" + String(count);
    }
    testFileName = testFileName + ".txt";
    // fileExists = SD.exists(testFileName);
  }
  logFileName = testFileName;
  debugPrint("file name found: " + logFileName);
  writeToLogFile("Log file initiated");
  writeToLogFile("file name found: " + logFileName);
}

String formatTime(int timeVal) {
  if (timeVal < 10) {
    return "0" + String(timeVal);
  }
  return String(timeVal);
}

void debugPrint(String output) {
  if (DEBUG) {
  // if (true) {
    Serial.println(output);
  }
}

void setupSensors()
{
  setupBarometricSensor();

  // Start temperature sensor data
  if (!useAnalogTemp) {
    onewireTempSensors.begin();
  }
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  gpsPort.begin(9600);
  if (!powerSensor.begin()) {
    powerSensorFound = false;
    writeToLogFile("Unable to find INA260 power sensor chip");
  } else {
    powerSensorFound = true;
  }
}

void setupBarometricSensor()
{
    // Uno uses address 0x77, mega uses 0x76
    bme280.parameter.I2CAddress = 0x76;

  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  
  //Now choose on which mode your device will run
  //On doubt, just leave on normal mode, that's the default value
  
  //0b00:     In sleep mode no measurements are performed, but power consumption is at a minimum
  //0b01:     In forced mode a single measured is performed and the device returns automatically to sleep mode
  //0b11:     In normal mode the sensor measures continually (default value)
  
  bme280.parameter.sensorMode = 0b11;                   //Choose sensor mode

  // Start barometric sensor
  Wire.begin();

  //*********************** TSL2591 *************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  
  //Great! Now set up the internal IIR Filter
  //The IIR (Infinite Impulse Response) filter suppresses high frequency fluctuations
  //In short, a high factor value means less noise, but measurements are also less responsive
  //You can play with these values and check the results!
  //In doubt just leave on default

  //0b000:      factor 0 (filter off)
  //0b001:      factor 2
  //0b010:      factor 4
  //0b011:      factor 8
  //0b100:      factor 16 (default value)

  bme280.parameter.IIRfilter = 0b100; //Setup for IIR Filter

  //************************** BME280 ***********************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  
  //Next you'll define the oversampling factor for the humidity measurements
  //Again, higher values mean less noise, but slower responses
  //If you don't want to measure humidity, set the oversampling to zero

  //0b000:      factor 0 (Disable humidity measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)
  
  bme280.parameter.humidOversampling = 0b101; //Setup Humidity Oversampling

  //************************** BME280 ***********************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  
  //Now define the oversampling factor for the temperature measurements
  //You know now, higher values lead to less noise but slower measurements
  
  //0b000:      factor 0 (Disable temperature measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)

  bme280.parameter.tempOversampling = 0b101; //Setup Temperature Ovesampling

  //************************** BME280 ***********************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  
  //Finally, define the oversampling factor for the pressure measurements
  //For altitude measurements a higher factor provides more stable values
  //On doubt, just leave it on default
  
  //0b000:      factor 0 (Disable pressure measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)
  
  bme280.parameter.pressOversampling = 0b101; //Setup Pressure Oversampling 

  
  //************************** BME280 ***********************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  
  //For precise altitude measurements please put in the current pressure corrected for the sea level
  //On doubt, just leave the standard pressure as default (1013.25 hPa)
  
    bme280.parameter.pressureSeaLevel = 1013.25;           //default value of 1013.25 hPa

  //Now write here the current average temperature outside (yes, the outside temperature!)
  //You can either use the value in Celsius or in Fahrenheit, but only one of them (comment out the other value)
  //In order to calculate the altitude, this temperature is converted by the library into Kelvin
  //For slightly less precise altitude measurements, just leave the standard temperature as default (15°C)
  //Remember, leave one of the values here commented, and change the other one!
  //If both values are left commented, the default temperature of 15°C will be used
  //But if both values are left uncommented, then the value in Celsius will be used    
  
    bme280.parameter.tempOutsideCelsius = 15;              //default value of 15°C
  //bme280.parameter.tempOutsideFahrenheit = 59;           //default value of 59°F
  
  //*********************************************************************
  //*************ADVANCED SETUP IS OVER - LET'S CHECK THE CHIP ID!*******

  if (bme280.init_BME280() != 0x60)  
  {
    writeToLogFile("BME280 could not be found.");
    bmeFound = false;
  }
  else
  {
    // writeToLogFile("BME280 detected");
    bmeFound = true;
  }
}

void loop()
{
  bool buttonPressActions = handleInputButtonPress();
  // if (buttonPressActions)
  //   debugPrint("button is being pressed");
  // after at least read time amount has passed, print out current stats
  // and save to SD card
  // includes both GPS stats (date/time/location/etc) and temperature
  bool timeToWrite = (millis() - timer > readTime);
  // if (timeToWrite && gps.available(gpsPort)) {
  if (gps.available(gpsPort)) {
    fix = gps.read(); // NeoGPS
    if (fix.valid.altitude) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    
    // Sends command to retrieve temperatures before querying
    onewireTempSensors.requestTemperatures();

    String powerSensorData;
    if (powerSensorFound) {
      // TODO: Unused, remove if not needed
      float current = powerSensor.readCurrent();
      float busVoltage = powerSensor.readBusVoltage();
      float power = powerSensor.readPower();
      powerSensorData = String(current, 4) + "," + String(busVoltage, 4) + 
                        "," + String(power, 4) + ",";
    } else {
      powerSensorData = "NAN,NAN,NAN";
    }

    // Read barometric sensor data and store in string
    String barSensorData = String(bme280.readPressure()) + "," +
                           String(bme280.readTempC()) + "," + 
                           String(bme280.readHumidity()) + ",";
    float barAltitude = bme280.readAltitudeMeter();
    // String barAltitude = String(bme280.readAltitudeMeter());
    
    timer = millis(); // reset the timer

    barSensorData += String(barAltitude) + ",";
    if (bmeFound) {
      addAltitudeData(barAltitude, "barometric");
    }

    // Reset lastValidDateTime and only set if fix is valid
    lastValidDateTime = "";
    String dateStr;
    String dataStr;
    adjustTime(fix.dateTime);

    dateStr += formatTime(fix.dateTime.hours) + ":" +
               formatTime(fix.dateTime.minutes) + ":" +
               formatTime(fix.dateTime.seconds) + ",";
    dateStr += String(fix.dateTime.month) + "/" +
               String(fix.dateTime.date) + "/" + 
               String(fix.dateTime.year) + ",";
    dataStr += dateStr;

    dataStr += String(onewireTempSensors.getTempCByIndex(0)) + ",";
    // dataStr += String(onewireTempSensors.getTempCByIndex(1)) + ",";

    dataStr += barSensorData;
    dataStr += powerSensorData;

    if (fix.valid.location && fix.valid.altitude) {
      String strLat = String(fix.latitude(), 6);
      String strLon = String(fix.longitude(), 6);
      String strSpeed = String(fix.speed_kph());
      float alt = fix.altitude();
      String strAlt = String(alt);
      String strSats = String((int)fix.satellites);
      // TODO: get working without cast to String
      // converting to string and back to float because
      // we were seeing issues when just using raw GPS data
      if (!outsideGeofence) {
        // Test 
        // outside fence:
        // outsideGeofence = !isInsideGeofence(43.15061, -82.67281);
        // outsideGeofence = !isInsideGeofence(43.92313, -82.89254);
        // inside fence:
        // outsideGeofence = !isInsideGeofence(43.08570, -83.29740);
        // outsideGeofence = !isInsideGeofence(43.09083, -83.29861);

        outsideGeofence = !isInsideGeofence(strLat.toFloat(), strLon.toFloat());
      }

      // Only save time and dates if we have a valid location
      // otherwise, it is using default settings
      lastValidDateTime = dateStr;
      addAltitudeData(alt, "gps");

      dataStr += strLat + 
                 "," + strLon + ",";
      dataStr += strSpeed + "," + strAlt + "," + strSats;
    } else {
      // print raw data for debugging
      dataStr += "unable to retrieve data";
    }

    // // add test data to barometric pressure
    // uint32_t testTimer = millis();
    // debugPrint("test looking for button");
    // while (millis() - testTimer < 60000) {
    //   handleInputButtonPress();
    // }
    // debugPrint("done looking for button");
    // int pointSize = 45;
    // float points[] = {204.43, 204.6, 204.52, 204.18, 204.6,
    //                   204.35, 204.52, 204.6, 204.18, 204.43,
    //                   204.35, 204.43, 205.79, 240.33, 310.96,
    //                   386.23, 460.4, 540.71, 616.18, 684.9,
    //                   755.61, 828.08, 897.07, 968.08, 1038.76,
    //                   1124.88, 1202.56, 23001.38, 23037.09, 23059.78,
    //                   22855.55, 22654.02, 22368.4, 22088.34, 21797.27,
    //                   21522.79, 21216.25, 20939.99, 20652.88, 20363.88,
    //                   20087.91, 19807.08, 19534.37, 19257.83, 18974.81};
    // for (int i = 0; i < pointSize; i++) {
    //   debugPrint("adding point " + String(points[i]));
    //   addAltitudeData(points[i], "barometric");
    //   if (checkAltitudeForFalling()) {
    //     handleFenceBreakOrFalling();
    //   }
    //   delay(1000);
    // }
    // debugPrint("done with testing");
    // delay(10000);

    // check to see if falling
    // don't need to check if we are already falling
    if (!isFalling) {
      isFalling = checkAltitudeForFalling();
    }
    if (isFalling) {
      handleFenceBreakOrFalling();
    }

    // if outside geofence, disconnect payload from balloon
    // only use if useGeofence is set to true
    if (useGeofence && outsideGeofence) {
      handleFenceBreakOrFalling();
    }

    if (sd_init_successful) {
       writeToDataFile(dataStr);
    } else {
      debugPrint("sd not available");
    }
    debugPrint(dataStr);
    readOnce = true;
  } else if (timeToWrite) {
    digitalWrite(LED_BUILTIN, HIGH);
    // Sends command to retrieve temperatures before querying
    onewireTempSensors.requestTemperatures();

    String powerSensorData;
    if (powerSensorFound) {
      // TODO: Unused, remove if not needed
      float current = powerSensor.readCurrent();
      float busVoltage = powerSensor.readBusVoltage();
      float power = powerSensor.readPower();
      powerSensorData = String(current, 4) + "," + String(busVoltage, 4) + 
                        "," + String(power, 4) + ",";
    } else {
      powerSensorData = "NAN,NAN,NAN";
    }

    // Read barometric sensor data and store in string
    String barSensorData = String(bme280.readPressure()) + "," +
                           String(bme280.readTempC()) + "," + 
                           String(bme280.readHumidity()) + ",";
    float barAltitude = bme280.readAltitudeMeter();
    
    timer = millis(); // reset the timer

    barSensorData += String(barAltitude) + ",";
    if (bmeFound && readOnce) {
      addAltitudeData(barAltitude, "barometric");
    }

    // Reset lastValidDateTime and only set if fix is valid
    lastValidDateTime = "";
    String dateStr;
    String dataStr;
    
    dateStr += " , ,";
    dataStr += dateStr;

    dataStr += String(onewireTempSensors.getTempCByIndex(0)) + ",";

    dataStr += barSensorData;
    dataStr += powerSensorData;

    dataStr += "unable to retrieve data";

    // check to see if falling
    // don't need to check if we are already falling
    if (!isFalling) {
      isFalling = checkAltitudeForFalling();
    }
    if (isFalling) {
      handleFenceBreakOrFalling();
    }

    // if outside geofence, disconnect payload from balloon
    // only use if useGeofence is set to true
    if (useGeofence && outsideGeofence) {
      handleFenceBreakOrFalling();
    }

    if (sd_init_successful) {
      if (readOnce) {
        writeToDataFile(dataStr);
      }
    } else {
      debugPrint("sd not available");
    }
    debugPrint(dataStr);
    readOnce = true;
  }
}

bool isInsideGeofence(float lat, float lon) {
  // Compares to geofence locations to see if in or out
  // if out, will trigger balloon disconnect sequence
  // (currently is burn wire)
  bool inside = true;

  // Don't compare if under lower latitude. Outside of danger
  // of Great Lakes
  if (lat >= geofenceLower.p2Lat) {
    float relativeLocLower = (lat - geofenceLower.p1Lat)*
                        (geofenceLower.p2Lon - geofenceLower.p1Lon)
                        - (lon - geofenceLower.p1Lon)*
                        (geofenceLower.p2Lat - geofenceLower.p1Lat);
    float relativeLocUpper = (lat - geofenceUpper.p1Lat)*
                        (geofenceUpper.p2Lon - geofenceUpper.p1Lon)
                        - (lon - geofenceUpper.p1Lon)*
                        (geofenceUpper.p2Lat - geofenceUpper.p1Lat);
    if (relativeLocLower > 0 || relativeLocUpper > 0 || lat > geofenceUpper.p1Lat) {
      inside = false;
      writeToLogFile("OUTSIDE FENCE at point (" +
                       String(lat, 6) + ", " + String(lon, 6) + ")");
      // for debugging
      if (relativeLocLower > 0) {
        writeToLogFile("Outside lower fence.");
      } else if (relativeLocUpper > 0) {
        writeToLogFile("Outside upper fence");
      } else {
        writeToLogFile("Above upper fence");
      }
    }
  }

  return inside;
}

void writeToDataFile(String writeData)
{
  File dataFile = SD.open(dataFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println(writeData);
    dataFile.close();
    debugPrint("successfully wrote to SD card");
    delay(100);
  } else {
    writeToLogFile("error opening data file");
    debugPrint("file name is: " + dataFileName);
  }
}

void writeToLogFile(String writeData)
{
  debugPrint(writeData);
  File logFile = SD.open(logFileName, FILE_WRITE);
  if (logFile) {
    // Write date/time if not invalid. If invalid, 
    // empty string is written
    logFile.println(lastValidDateTime + " ");    
    logFile.println(writeData);
    logFile.close();
    delay(100);
  } else {
    debugPrint("error opening log file");
  }
}

// dataType options are barometric or gps
// also sets the highest recorded altitude when 
// new data is passed
void addAltitudeData(float newData, String dataType) 
{
  if (dataType == "barometric") {
    if (newData > barHighestAltitude) {
      barHighestAltitude = newData;
      barHighestAltitudeIndex = 0;
    } else {
      barHighestAltitudeIndex++;
    }
    // barHighestAltitude = max(newData, barHighestAltitude);
    // Move every reading by one, removing last point
    for (int i = readCount-1; i > 0; i--) {
      barAltitudeReadings[i] = barAltitudeReadings[i-1];
    }
    barAltitudeReadings[0] = newData;
    String barReadings;
    for (int i = 0; i < readCount; i++) {
      barReadings += String(barAltitudeReadings[i]) + " ";
    }
  } else if (dataType == "gps") {
    if (newData > gpsHighestAltitude) {
      gpsHighestAltitude = newData;
      gpsHighestAltitudeIndex = 0;
    } else {
      gpsHighestAltitudeIndex++;
    }
    // Move every reading by one, removing last point
    for (int i = readCount-1; i > 0; i--) {
      gpsAltitudeReadings[i] = gpsAltitudeReadings[i-1];
    }
    gpsAltitudeReadings[0] = newData;
    String gpsReadings;
    for (int i = 0; i < readCount; i++) {
      gpsReadings += String(gpsAltitudeReadings[i]) + " ";
    }
  }
}

// checks both gps and geometric readings
bool checkAltitudeForFalling()
{
  bool barIsFalling = false;
  bool gpsIsFalling = false;
  bool gpsValid = true;
  bool barValid = true;
  float maxVal = gpsAltitudeReadings[0];
  float minVal = gpsAltitudeReadings[0];
  for (int i = 0; i < compareCount; i++) {
    int index = i;
    float reading = gpsAltitudeReadings[index];
    if (reading < 1.0) {
      gpsValid = false;
      break;
    }
    // We don't have enough readings to determine if we are below the highest
    // altitude or not if the gpsHighestAltitudeIndex
    if ((gpsHighestAltitude - gpsAltitudeReadings[index]) 
          < fallingAltitude || index > gpsHighestAltitude) {
      // If any of the compared points aren't below the highest altitude,
      // don't mark as falling and break
      gpsIsFalling = false;
      break;
    }
    if (gpsHighestAltitude > minAltitude) {
      gpsIsFalling = true;
    }
  }
  if (gpsValid && gpsIsFalling) {
    writeToLogFile("GPS recorded falling at altitude " + 
                    String(gpsAltitudeReadings[0]) + 
                    " with highest recorded altitude being " + 
                    String(gpsHighestAltitude));
  }

  maxVal = barAltitudeReadings[0];
  minVal = barAltitudeReadings[0];
  for (int i = 0; i < compareCount; i++) {
    int index = i;
    float reading = barAltitudeReadings[index];
    if (reading < 1.0) {
      barValid = false;
      break;
    }
    if ((barHighestAltitude - barAltitudeReadings[index]) 
          < fallingAltitude || index > barHighestAltitude) {
      // If any of the compared points aren't below the highest altitude,
      // don't mark as falling and break
      barIsFalling = false;
      break;
    }
    if (barHighestAltitude > minAltitude) {
      barIsFalling = true;
    }
  }
  if (barValid && barIsFalling) {
    writeToLogFile("Barometric pressure sensor recorded falling at altitude " + 
                    String(barAltitudeReadings[0]) + 
                    " with highest recorded altitude being " + 
                    String(barHighestAltitude));
  }

  // bool retIsFalling = barIsFalling || ();
  // Look for barometric pressure falling. If the barometric pressure
  // sensor was not found, look for gps falling

  return (barIsFalling || (gpsIsFalling && !bmeFound));
}

static const int32_t          zone_hours   = -5L; // EST
static const int32_t          zone_minutes =  0L; // usually zero
static const NeoGPS::clock_t  zone_offset  =
                                zone_hours   * NeoGPS::SECONDS_PER_HOUR +
                                zone_minutes * NeoGPS::SECONDS_PER_MINUTE;

// Uncomment one DST changeover rule, or define your own:
#define USA_DST
//#define EU_DST

#if defined(USA_DST)
  static const uint8_t springMonth =  3;
  static const uint8_t springDate  = 14; // latest 2nd Sunday
  static const uint8_t springHour  =  2;
  static const uint8_t fallMonth   = 11;
  static const uint8_t fallDate    =  7; // latest 1st Sunday
  static const uint8_t fallHour    =  2;
  #define CALCULATE_DST

#elif defined(EU_DST)
  static const uint8_t springMonth =  3;
  static const uint8_t springDate  = 31; // latest last Sunday
  static const uint8_t springHour  =  1;
  static const uint8_t fallMonth   = 10;
  static const uint8_t fallDate    = 31; // latest last Sunday
  static const uint8_t fallHour    =  1;
  #define CALCULATE_DST
#endif

void adjustTime( NeoGPS::time_t & dt )
{
  NeoGPS::clock_t seconds = dt; // convert date/time structure to seconds

  #ifdef CALCULATE_DST
    //  Calculate DST changeover times once per reset and year!
    static NeoGPS::time_t  changeover;
    static NeoGPS::clock_t springForward, fallBack;

    if ((springForward == 0) || (changeover.year != dt.year)) {

      //  Calculate the spring changeover time (seconds)
      changeover.year    = dt.year;
      changeover.month   = springMonth;
      changeover.date    = springDate;
      changeover.hours   = springHour;
      changeover.minutes = 0;
      changeover.seconds = 0;
      changeover.set_day();
      // Step back to a Sunday, if day != SUNDAY
      changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
      springForward = (NeoGPS::clock_t) changeover;

      //  Calculate the fall changeover time (seconds)
      changeover.month   = fallMonth;
      changeover.date    = fallDate;
      changeover.hours   = fallHour - 1; // to account for the "apparent" DST +1
      changeover.set_day();
      // Step back to a Sunday, if day != SUNDAY
      changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
      fallBack = (NeoGPS::clock_t) changeover;
    }
  #endif

  //  First, offset from UTC to the local timezone
  seconds += zone_offset;

  #ifdef CALCULATE_DST
    //  Then add an hour if DST is in effect
    if ((springForward <= seconds) && (seconds < fallBack))
      seconds += NeoGPS::SECONDS_PER_HOUR;
  #endif

  dt = seconds; // convert seconds back to a date/time structure

} // adjustTime

void burnWireCycle()
{
  for (int i = 0; i < burnCycles; i++) {
    digitalWrite(burnWirePin, HIGH);
    delay(burnTime);
    digitalWrite(burnWirePin, LOW);
    delay(burnDelayTime);
  }
}

bool handleInputButtonPress()
{
  // return bool to determine if we need to put rest
  // of functions on hold or not
  bool handlingButton = false;
  int buttonState = digitalRead(inputButtonPin);
  if (buttonState == HIGH) {
    writeToLogFile("Button pressed. Initializing burn cycle");
    delay(4000);
    burnWireCycle();
    handlingButton = false;    
  }

  return handlingButton;
}

void handleFenceBreakOrFalling() {
  if (((bmeFound && barAltitudeReadings[0] > burnAltitudeLimit) ||
      (!bmeFound && gpsAltitudeReadings[0] > burnAltitudeLimit)) &&
      !burnReleaseComplete) {
    writeToLogFile("Burn cycle started automatically. Disconnecting from balloon");
    burnWireCycle();
    burnReleaseComplete = true;
  } else {
    digitalWrite(burnWirePin, LOW);
  }
}
