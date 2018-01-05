/*
 * Notre Dame Rocket Team Air-Breaking Payload Flight Code Version 0.8
 * Currently untested, relying on data from the control code simulation
 * 
 * Author: Aidan McDonald
 * Last Update: 01/05/2018
 * 
 * To-Dos:
 * TEST LITERALLY EVERYTHING
 * Potentiometer-based servo jam monitoring
 * Tune thresholds, especially servo limits!
 * Figure out which pin is the chip select pin
 * 
 * To-Dones:
 * Core flight-tracking switch case structure
 * PID control loop- load comparison data, calculate error, output angle
 * Accelerometer and barometer sensor data incorporation
 * Data saving
 * Servo startup routine for visual confirmation of payload functionality
 * Backup "emergency rescue" mode for if the rocket enters freefall again
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_BMP280 bmp; //I2C barometer initialization; for other versions check sample code
Servo tabServos;

//Flight-staging constants, for code readability
#define WAITING 0
#define LAUNCHED 1
#define BURNOUT 2
#define APOGEE 3
#define LANDED 4
//Other useful constants that may need to be tweaked over time
const int chipSelect = 4; //FIGURE OUT IF THIS IS RIGHT OR NOT!!!!!!!!!!!!!
const String dataFileName = "datalog.txt";
const int dataSize = 8; //Number of data points saved to SD card each cycle
const String inFileName = "bestflight.txt";
const int preCalcSize = 2350; //Number of data points in the pre-calculated ideal flight layout
const int potPin = A1;
const int servoPin = 9;
const int baroRegSize = 10; //Accuracy of regression varies wildly with number of points used
const float seaPressure = 1013.25; //Update @ launch site
const int potNoiseThreshold = 5; //Degrees
const int maxPropDelay = 250; //Milliseconds
const float accelLiftoffThreshold = 50; //m/s^2
const float baroLiftoffThreshold = 10; //m
const float accelBurnoutThreshold = -10; //m/s^2
const float baroApogeeThreshold = 5; //m
const float baroLandedThreshold = 5; //m
const float accelFreefallThreshold = 30; //m/s^2
const float thetaMin = 0; //Degrees
const float thetaMax = 90;

//Flags
bool saveData = false;
bool runPIDControl = false;
bool emergencyRescue = false;

//Global variables (i.e. things that I'd like to create in void setup(), but then wouldn't apply to void loop())
int flightState = WAITING; //Modify this to start the code at a different point
float accelX, accelY, accelZ;
float temperature, pressure, altitude;
float potValue;
float lastA, lastCalcT, lastPIDT, launchT, launchA;
float bestAlt[preCalcSize], bestVel[preCalcSize];
float theta, velocity = 0, maxA = 0;
float integralTerm = 0, lastError = 0;
float baroRegArr[baroRegSize], timeRegArr[baroRegSize];

void setup() {
  while (!Serial); //FOR TESTING PURPOSES ONLY!!!
  Serial.begin(9600);
  
  if(!SD.begin(chipSelect)) //Merge with the other loop in final code, separate now for debugging purposes
  {
    Serial.println("Error: SD Card initialization failure");
    while(1);
  }
  if(!accel.begin() || !bmp.begin())
  {
    Serial.println("Error: Sensor initialization failure");
    while(1);
  }

  tabServos.attach(servoPin);
  
  accel.setRange(ADXL345_RANGE_16_G);
  altitude = bmp.readAltitude(seaPressure); //Set a baseline starting altitude
  launchA = altitude;

  ReadBestFlight();

  tabServos.write(thetaMax); //Run servo startup routine
  delay(maxPropDelay*2);
  potValue = map(analogRead(potPin),0,1023,0,269);
  if(fabs(potValue-thetaMax) > potNoiseThreshold){
    Serial.println("Error: Jammed Mechanism");
    while(1);
  }
  delay(1000);
  tabServos.write(thetaMin);
  delay(maxPropDelay*2);
  if(fabs(potValue-thetaMin) > potNoiseThreshold){
    Serial.println("Error: Jammed Mechanism");
    while(1);
  }
}

void loop() {
  GetSensorData();
  switch(flightState){
    case WAITING:
      UpdateBaroBuffers();
      if(accelZ > accelLiftoffThreshold ||  (altitude-launchA) > baroLiftoffThreshold){
        flightState = LAUNCHED;
        launchT = millis();
        saveData = true;
      }
    break;
    case LAUNCHED:
      if(fabs(altitude-lastA) > 0.0001){
        UpdateBaroBuffers();
        velocity = CalcBaroVel();
      }
      if(accelZ < accelBurnoutThreshold){
        flightState = BURNOUT;
        runPIDControl = true;
      }
    break;
    case BURNOUT:
      velocity = CalcAccelVel(velocity);
      if(maxA > altitude + baroApogeeThreshold){
        flightState = APOGEE;
        runPIDControl = false;
      }
    break;
    case APOGEE:
      velocity = CalcAccelVel(velocity);
      if(velocity > accelFreefalThreshold)
        emergencyRescue = true;
      if(fabs(altitude - launchA) <= baroLandedThreshold){
        flightState = LANDED;
        saveData = false;
        emergencyRescue = false;
      }
    break;
    case LANDED:

    break;
  }
  if(runPIDControl){
    float error = CalcError(altitude, velocity, (int)(millis()-launchT)/10);
    theta = PID(error, lastError, integralTerm, millis()-lastPIDT);
    lastError = error;
    lastPIDT = millis();
    tabServos.write(theta);
  }
  if(saveData)
    SaveSensorData();
  if(emergencyRescue)
    tabServos.write(thetaMax);
}

void ReadBestFlight(){
  File inFile = SD.open(inFileName);
  int c = 0; //Counter variable
  if(inFile){
    while(inFile.available()){
      bestAlt[c] = inFile.parseFloat();
      bestVel[c] = inFile.parseFloat();
      c++;
    }
    inFile.close();
  }
  else
    while(1); //Freeze code if comparison dataset cannot be read
}

void UpdateBaroBuffers(){
  for(int c = 0; c < baroRegSize-1; c++){
    baroRegArr[c] = baroRegArr[c+1];
    timeRegArr[c] = timeRegArr[c+1];
  }
  baroRegArr[baroRegSize-1] = altitude;
  timeRegArr[baroRegSize-1] = millis();
}

float CalcBaroVel(){
  float sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
  for(int c = 0; c < baroRegSize; c++){ //Calculate variance and covariance components; fill time buffer
    sumX += timeRegArr[c];
    sumY += baroRegArr[c];
    sumXX += pow(timeRegArr[c],2);
    sumXY += timeRegArr[c]*baroRegArr[c];
  }
  double slope = (baroRegSize*sumXY - sumX*sumY) / (baroRegSize*sumXX - sumX*sumX); //Slope = covariance/variance
  return slope;
}

float CalcAccelVel(float lastVel){
  float newVel = lastVel + accelZ*(millis()-lastCalcT)/1000;
  lastCalcT = millis();
  return newVel;
}

float CalcError(float realAlt, float realVel, int startT){
  bool match = false;
  int c = startT; //To optimize search time, start at the current time point

  while(!match){ //Find closest altitude in buffer to current altitude
    float test = fabs(realAlt - bestAlt[c]); //Calculate delta-Y at, above, and below current test value
    float above = fabs(realAlt - bestAlt[c+1]);
    float below = fabs(realAlt - bestAlt[c-1]);
    if(test <= above && test <= below) //If current point has least error, we have a match
      match = true;
    else if (above < test) //Otherwise, go up or down accordingly
      c++;
    else if (below < test)
      c -= 1;
  }
  return (realVel - bestVel[c]); //Return difference in velocities at the given altitude point
}

float PID(float error, float lastE, float &iTerm, int deltaT){
  static float cP = 80; //P constant
  static float cI = 0; //I constant
  static float cD = 4; //D constant
  float dT = (float)deltaT/1000; //Variable delta-T term

  float pTerm = error*cP; //Calculate each term
  iTerm = iTerm + error*cI*dT; //The i-term is passed by reference and updated throughout flight
  float dTerm = (error - lastE)*cD/dT;

  float thetaOut = pTerm + iTerm + dTerm; //calculate intended output angle
  if(thetaOut < thetaMin)
    thetaOut = thetaMin;
  else if(thetaOut > thetaMax)
    thetaOut = thetaMax;
  return thetaOut;
}

void GetSensorData(){
  potValue = map(analogRead(potPin),0,1023,0,269); //Read potentiometer data and map to a displacement angle
  sensors_event_t event; //Read accelerometer data
  accel.getEvent(&event);
  accelX = event.acceleration.x;
  accelY = event.acceleration.y;
  accelZ = event.acceleration.z;
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  lastA = altitude; //Variable to track if new altitude data has come in
  altitude = bmp.readAltitude(seaPressure)-launchA; //Shift all altitude data relative to the starting point
  if(altitude > maxA)
    maxA = altitude; //Also track maximum altitude
}

void SaveSensorData(){
  File dataLog = SD.open(dataFileName, FILE_WRITE);
  if(dataLog){
    float dataBuff[dataSize];
    String dataString;
    dataBuff[0] = millis();
    dataBuff[1] = accelX;
    dataBuff[2] = accelY;
    dataBuff[3] = accelZ;
    dataBuff[4] = temperature;
    dataBuff[5] = pressure;
    dataBuff[6] = altitude;
    dataBuff[7] = potValue;
    for(int c = 0; c<dataSize; c++){
      dataString += String(dataBuff[c]);
      dataString += ",";
    }
    dataLog.println(dataString);
    dataLog.close();
  }
  else
    Serial.print("Error: Unable to open "); Serial.println(dataFileName);
}

