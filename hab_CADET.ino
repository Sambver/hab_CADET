// Uses GPS and SD breakout board
// GPS generates data at a specified rate
// and data is written to SD card

// GPSort library gives more accurate readings than Adafruit GPS
#include <NMEAGPS.h>
#include <GPSport.h>
// Libaries to write to SD card
#include <SPI.h>
#include <SD.h>
// Read temperature sensors
#include <OneWire.h>
#include <DallasTemperature.h>
// servo library, controls connection between balloon and payload
#include <Servo.h>
// power sensor library
#include <Adafruit_INA260.h>
// barometric pressure sensor
#include <Wire.h>
#include "BlueDot_BME280_TSL2591.h"
BlueDot_BME280_TSL2591 bme280;

NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values

// detects voltage, current, and power
Adafruit_INA260 powerSensor = Adafruit_INA260();
bool powerSensorFound = false;

// servo related variables
Servo balloonAttachServo;
const int servoOpenPos = 115;
const int servoHalfOpenPos = 100;
const int servoClosePos = 70;
int servoCurrentPos = 0;
const int servoPin = 9;
bool closedFromButton = true;

// set sample time in milliseconds
const int readTime = 5000;

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
Geofence geofenceLower = {42.98269, -82.82611, 41.70606, -83.82611};
Geofence geofenceUpper = {43.84309, -83.06529, 42.98269, -82.82611};
Geofence geofenceOhio = {43.84309, -83.06529, 42.98269, -82.82611};

// Geofence geofences[] = {geofenceLower, geofenceUpper, geofenceOhio};

// input tactile button
const int servoButtonPin = 10;

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
const int fallingAltitude = 100; // must drop this far to be considered falling
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
  // TODO: remove
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

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
}

String formatTime(int timeVal) {
  if (timeVal < 10) {
    return "0" + String(timeVal);
  }
  return String(timeVal);
}

void debugPrint(String output) {
  if (DEBUG) {
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
  // setup servo that connects balloon to payload
  // as well as tactile button that controls servo
  setupBalloonAttachServo();
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
  }
  // else
  // {
  //   writeToLogFile("BME280 detected");
  // }
}

void setupBalloonAttachServo()
{
  balloonAttachServo.attach(servoPin);
  // start by closing the servo
  // we do this so that we know the starting location
  // of the servo, since it cannot tell us
  servoCurrentPos = servoClosePos;
  balloonAttachServo.write(servoCurrentPos);
  // initialize pushbutton pin as input
  pinMode(servoButtonPin, INPUT);
}

void loop()
{
  handleServoButtonPress();
  // approximately every 10 seconds or so, print out the current stats
  // includes both GPS stats (date/time/location/etc) and temperature
  if (millis() - timer > readTime && gps.available(gpsPort)) {
    fix = gps.read();
    
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
    addAltitudeData(barAltitude, "barometric");

    // Reset lastValidDateTime and only set if fix is valid
    lastValidDateTime = "";
    String dateStr;
    String dataStr;
    dateStr += formatTime(fix.dateTime.hours) + ":" +
               formatTime(fix.dateTime.minutes) + ":" +
               formatTime(fix.dateTime.seconds) + ",";
    dateStr += String(fix.dateTime.day) + "/" + 
               String(fix.dateTime.month) + "/" +
               String(fix.dateTime.year) + ",";
    dataStr += dateStr;

    dataStr += String(onewireTempSensors.getTempCByIndex(0)) + ",";
    // dataStr += String(onewireTempSensors.getTempCByIndex(1)) + ",";

    dataStr += barSensorData;
    dataStr += powerSensorData;

    // Test 
    // outside fence:
    // outsideGeofence = !isInsideGeofence(43.15061, -82.67281);
    // outsideGeofence = !isInsideGeofence(43.92313, -82.89254);
    // inside fence:
    // outsideGeofence = !isInsideGeofence(43.08570, -83.29740);
    // outsideGeofence = !isInsideGeofence(43.09083, -83.29861);

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
      outsideGeofence = !isInsideGeofence(strLat.toFloat(), strLon.toFloat());

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
    }
    debugPrint(dataStr);
  }
}

bool isInsideGeofence(float lat, float lon) {
  // Compares to geofence locations to see if in or out
  // if out, will trigger the servo opening and disconnecting
  // from balloon
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
      writeToLogFile("OUTSIDE FENCE");
      // for debugging
      if (relativeLocLower > 0) {
        writeToLogFile("Outside lower fence");
      } else if (relativeLocUpper > 0) {
        writeToLogFile("Outside upper fence");
      } else {
        writeToLogFile("Above upper fence");
      }
    } else {
      debugPrint("inside fence");
    }
  } else {
    debugPrint("inside fence");
  }

  return inside;
}

void writeToDataFile(String writeData)
{
  File dataFile = SD.open(dataFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println(writeData);
    dataFile.close();
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
    // Write date/time if not invalid. If the last reading was
    // not valid, it will write an empty line instead
    logFile.println(lastValidDateTime + " ");    
    logFile.println(writeData);
    logFile.close();
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
    gpsHighestAltitude = max(newData, gpsHighestAltitude);
    barHighestAltitude = max(newData, barHighestAltitude);
    // Move every reading by one, removing last point
    for (int i = readCount-1; i > 0; i--) {
      barAltitudeReadings[i] = barAltitudeReadings[i-1];
    }
    barAltitudeReadings[0] = newData;
    String barReadings;
    for (int i = 0; i < readCount; i++) {
      barReadings += String(barAltitudeReadings[i]) + " ";
    }
    debugPrint("new barometric readings: " + barReadings);
  } else if (dataType == "gps") {
    debugPrint("adding reading to gps readings " + String(newData));
    // Move every reading by one, removing last point
    for (int i = readCount-1; i > 0; i--) {
      gpsAltitudeReadings[i] = gpsAltitudeReadings[i-1];
    }
    gpsAltitudeReadings[0] = newData;
    String gpsReadings;
    for (int i = 0; i < readCount; i++) {
      gpsReadings += String(gpsAltitudeReadings[i]) + " ";
    }

    debugPrint("new gps readings: " + gpsReadings);
  }
}

// checks both gps and geometric readings
bool checkAltitudeForFalling()
{
  bool barIsFalling = false;
  bool gpsIsFalling = false;
  bool gpsValid = true;
  bool barValid = true;
  // set test readings
  // gpsHighestAltitude = 27448.2;
  // gpsAltitudeReadings[0] = 27132.8;
  // gpsAltitudeReadings[1] = 27268.6;
  // gpsAltitudeReadings[2] = 27339.2;
  // gpsAltitudeReadings[3] = 27406.5;
  // gpsAltitudeReadings[4] = 27448.2;
  // gpsAltitudeReadings[5] = 26582.7;
  // gpsAltitudeReadings[6] = 26105.1;
  // gpsAltitudeReadings[7] = 25620.7;
  // gpsAltitudeReadings[8] = 25131.3;
  // gpsAltitudeReadings[9]= 24164.5;
  float maxVal = gpsAltitudeReadings[0];
  float minVal = gpsAltitudeReadings[0];
  // First check gps. Do validity test at same time, fall test
  // is only valid if we have valid inputs
  // for (int i = 0; i < readCount; i++) {
  //   if (gpsAltitudeReadings[i] < 1.0) {
  //     gpsValid = false;
  //     break;
  //   }
  //   maxVal = max(gpsAltitudeReadings[i], maxVal);
  //   minVal = min(gpsAltitudeReadings[i], minVal);
  // }
  for (int i = 0; i < compareCount; i++) {
    int index = readCount-i-1;
    float reading = gpsAltitudeReadings[index];
    // debugPrint("gps index is: " + String(index));
    // debugPrint("gps reading is " + String(reading));
    if (reading < 1.0) {
      gpsValid = false;
      break;
    }
    if ((gpsHighestAltitude - gpsAltitudeReadings[index]) 
          < fallingAltitude) {
      // If any of the compared points aren't below the highest altitude,
      // don't mark as falling and break
      gpsIsFalling = false;
      break;
    }
    gpsIsFalling = true;
  }
  if (gpsValid && gpsIsFalling) {
    // gpsIsFalling = true;
    writeToLogFile("GPS recorded falling.");
  } else if (gpsValid) {
    debugPrint("not falling according to gps");
  }
  if (!gpsValid) {
    debugPrint("not enough data points to test for gps falling");
  }

  // test barometric data
  // barHighestAltitude = 23059.78;
  // // barAltitudeReadings[0] = 22758.25;
  // barAltitudeReadings[0] = 22810.27;
  // barAltitudeReadings[1] = 22853.73;
  // barAltitudeReadings[2] = 22888.41;
  // barAltitudeReadings[3] = 22919.64;
  // barAltitudeReadings[4] = 22954.77;
  // barAltitudeReadings[5] = 23037.09;
  // barAltitudeReadings[6] = 23059.78;
  // barAltitudeReadings[7] = 22855.55;
  // barAltitudeReadings[8] = 22654.02;
  // barAltitudeReadings[9] = 22368.4;
  // barAltitudeReadings[9] = 22088.34;

  maxVal = barAltitudeReadings[0];
  minVal = barAltitudeReadings[0];
  // Now check barometric readings
  // for (int i = 0; i < readCount; i ++) {
  //   if (barAltitudeReadings[i] < 1.0) {
  //     barValid = false;
  //     break;
  //   }
  //   maxVal = max(barAltitudeReadings[i], maxVal);
  //   minVal = min(barAltitudeReadings[i], minVal);
  // }
  for (int i = 0; i < compareCount; i++) {
    // debugPrint("reading bar index " + String(readCount-i));
    int index = readCount-i-1;
    float reading = barAltitudeReadings[index];
    if (reading < 1.0) {
      barValid = false;
      break;
    }
    if ((barHighestAltitude - barAltitudeReadings[index]) 
          < fallingAltitude) {
      // If any of the compared points aren't below the highest altitude,
      // don't mark as falling and break
      barIsFalling = false;
      break;
    }
    barIsFalling = true;
  }
  if (barValid && barIsFalling) {
    writeToLogFile("Barometric pressure recorded falling.");
  } else if (barValid) {
    debugPrint("not falling according to bar");
  }
  if (!barValid) {
    debugPrint("not enough data points to test for bar falling");
  }

  // only look for barometric pressure for now
  return barIsFalling;
  // return (barIsFalling || gpsIsFalling);
}

bool handleServoButtonPress()
{
  // if the servo is open, or we are currently closing the servo,
  // we pass true so that the gps read knows to pause
  // might not be needed, will test
  bool handlingServo = false;
  // Looks at button state. Opens quickly when pressed, and closes
  // slowly when release. Close is slow so to not cause pinching
  int buttonState = digitalRead(servoButtonPin);
  if (buttonState == HIGH && servoCurrentPos < servoHalfOpenPos) {
    handlingServo = true;
    servoCurrentPos = servoHalfOpenPos;
    writeToLogFile("Button pressed. Opening servo to half-open position.");
    balloonAttachServo.write(servoCurrentPos);
    delay(50);
    // toggle here to ensure that we don't close the servo if it
    // is opened from the geofence, only from button
    closedFromButton = false;
  } else if (buttonState == HIGH) {
    handlingServo = true;
  } else if (buttonState == LOW && servoCurrentPos > servoClosePos &&
             !closedFromButton) {
    handlingServo = true;
    servoCurrentPos--;
    // writeToLogFile("Button released. Servo being closed");
    balloonAttachServo.write(servoCurrentPos);
    delay(50);
    if (servoCurrentPos == servoClosePos) {
      writeToLogFile("Servo closed after button release");
      closedFromButton = true;
    }
  }

  return handlingServo;
}

void handleFenceBreakOrFalling() {
  if (servoCurrentPos < servoOpenPos) {
    writeToLogFile("Servo opening automatically. Disconnecting from balloon");
    servoCurrentPos = servoOpenPos;
    balloonAttachServo.write(servoCurrentPos);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}
