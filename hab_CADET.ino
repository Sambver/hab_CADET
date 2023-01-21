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

const float fenceLat1 = 43.08696;
const float fenceLon1 = -83.29721;
const float fenceLat2 = 43.08184;
const float fenceLon2 = -83.30335;

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
#define DEBUG true

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
    debugPrint("card initialized.");
    sd_init_successful = true;
    // Write first line to SD card
    File dataFile = SD.open("gpslog.csv", FILE_WRITE);
    if (dataFile) {
      String titleStr = "Time,Date,Outside Temp,Inside Temp,";
      titleStr += "Barometric Pressure,Barometric Temperature,";
      titleStr += "Barometric Humidity,Barometric Altitude,";
      titleStr += "Current (mA), BusVoltage (mV), Power (mW),";
      titleStr += "Latitude,Longitude,Speed,Altitude,Satellites";
      dataFile.println(titleStr);
      dataFile.close();
    }
    
  }
  setupSensors();
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
    debugPrint("Unable to find INA260 power sensor chip!");
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
    debugPrint("Ops! BME280 could not be found!");
  }
  else
  {
    debugPrint("BME280 detected!");
  }
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
  bool pauseRead = handleServoButtonPress();
  // while(pauseRead) {
  //   pauseRead = handleServoButtonPress();
  //   debugPrint("reading is paused for servo handling");
  // }
  // approximately every 10 seconds or so, print out the current stats
  // includes both GPS stats (date/time/location/etc) and temperature
  if (millis() - timer > 10000 && gps.available(gpsPort)) {
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
                           String(bme280.readHumidity()) + "," +
                           String(bme280.readAltitudeMeter()) + ",";
    
    timer = millis(); // reset the timer

    String dataStr;
    dataStr += formatTime(fix.dateTime.hours) + ":" +
               formatTime(fix.dateTime.minutes) + ":" +
               formatTime(fix.dateTime.seconds) + ",";
    dataStr += String(fix.dateTime.day) + "/" + 
               String(fix.dateTime.month) + "/" +
               String(fix.dateTime.year) + ",";

    dataStr += String(onewireTempSensors.getTempCByIndex(0)) + ",";
    dataStr += String(onewireTempSensors.getTempCByIndex(1)) + ",";

    dataStr += barSensorData;
    dataStr += powerSensorData;

    // Test 
    // outside fence:
    outsideGeofence = !isInsideGeofence(43.08570, -83.29740);
    // inside fence:
    // outsideGeofence = !isInsideGeofence(43.09083, -83.29861);

    if (fix.valid.location && fix.valid.altitude) {
      // TODO: get working without cast to String
      // converting to string and back to float because
      // we were seeing issues when just using raw GPS data
      String stLat = String(fix.latitude(),6);
      String stLon = String(fix.longitude(), 6);
      // outsideGeofence = !isInsideGeofence(stLat.toFloat(), stLon.toFloat());


      dataStr += String(fix.latitude(), 6) + 
                 "," + String(fix.longitude(), 6) + ",";
      dataStr += String(fix.speed_kph()) + "," + 
                 String(fix.altitude()) + "," + String((int)fix.satellites);
    } else {
      // print raw data for debugging
      dataStr += "unable to retrieve data";
    }

    // if outside geofence, disconnect payload from balloon
    // only use if useGeofence is set to true
    if (useGeofence && outsideGeofence) {
      handleFenceBreak();
    }

    if (sd_init_successful) {
       File dataFile = SD.open("gpslog.csv", FILE_WRITE);
       if (dataFile) {
         // if the file is available, write to it
         dataFile.println(dataStr);
         dataFile.close();
       } else if (DEBUG) {
          debugPrint("error opening file");
       }
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
    String output = "location in relation to lower fence: " + String(relativeLocLower, 6);
    String output = "location in relation to upper fence: " + String(relativeLocLower, 6);
    debugPrint(output);
    if (relativeLocLower > 0 || relativeLocUpper > 0 || lat > geofenceUpper.p1Lat) {
      inside = false;
      debugPrint("OUTSIDE FENCE");
      // for debugging
      if (relativeLocLower > 0) {
        debugPrint("Outside lower fence");
      } else if (relativeLocUpper > 0) {
        debugPrint("Outside upper fence");
      } else {
        debugPrint("Above upper fence");
      }
    } else {
      debugPrint("inside fence");
    }
  } else {
    debugPrint("inside fence");
  }

  return inside;
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
    String outputStr = "Opening servo. Servo pos will be set to: " 
                        + String(servoCurrentPos);
    debugPrint(outputStr);
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
    String outputStr = "Closing servo. Servo pos will be set to: " 
                        + String(servoCurrentPos);
    debugPrint(outputStr);
    balloonAttachServo.write(servoCurrentPos);
    delay(50);
    if (servoCurrentPos == servoClosePos) {
      closedFromButton = true;
    }
  }

  return handlingServo;
}

void handleFenceBreak() {
  if (servoCurrentPos < servoOpenPos) {
    debugPrint("opening servo to disconnect from balloon");
    servoCurrentPos = servoOpenPos;
    balloonAttachServo.write(servoCurrentPos);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}
