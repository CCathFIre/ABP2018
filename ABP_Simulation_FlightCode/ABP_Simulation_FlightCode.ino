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
#include <SdFat.h>
SdFat SD;
File dataLog;
int count = 0;
#include <Servo.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_BMP280 bmp; //I2C barometer initialization; for other versions check sample code
Servo tabServos;

//Flight-staging constants, for code readability
#define WAITING -1
#define ARMED 0
#define LAUNCHED 1
#define BURNOUT 2
#define APOGEE 3
#define LANDED 4
//Other useful constants that may need to be tweaked over time
const int chipSelect = 28;
const String dataFileName = "datalog_00.txt";
const String inFileName = "BESTFL~1.TXT";
const int preCalcSize = 2350; //Number of data points in the pre-calculated ideal flight layout
const int potPin = A1;
const int servoPin = 6;
const int armPin = 7; //CHANGE THIS TO REAL PIN
const int baroRegSize = 10; //Accuracy of regression varies wildly with number of points used
const float seaPressure = 1013.25; //Update @ launch site
const int potNoiseThreshold = 5; //Degrees
const int maxPropDelay = 250; //Milliseconds
const int sdWaitTime = 50;
const float accelLiftoffThreshold = 50; //m/s^2
const float baroLiftoffThreshold = 10; //m
const float accelBurnoutThreshold = -10; //m/s^2
const float baroApogeeThreshold = 5; //m
const float baroLandedThreshold = 5; //m
const float accelFreefallThreshold = 30; //m/s^2
const float thetaMin = 0; //Degrees
const float thetaFlush = 75;
const float thetaMax = 80;
const float maxStep = 15; //Degrees

//Flags
bool armed = false;
bool saveData = true;
bool runPIDControl = true;
bool emergencyRescue = false;

//Global variables (i.e. things that I'd like to create in void setup(), but then wouldn't apply to void loop())
int flightState = BURNOUT; //Modify this to start the code at a different point
float accelX, accelY, accelZ;
float temperature, pressure, altitude;
float potValue;
float lastA, lastCalcT, lastPIDT, launchT=0, launchA=0;
int lastSDT=0;
float bestAlt[preCalcSize], bestVel[preCalcSize];
float theta=thetaMax, velocity = 0, maxA = 0, lastTheta=theta;
float integralTerm = 0, lastError = 0;
float baroRegArr[baroRegSize], timeRegArr[baroRegSize];

//Simulation variables
float realA=-32, realV=600, realY=1350;
int t_init = 4000, lastT; //milliseconds

//Data File

void setup() {
  Serial.begin(9600);
  while (!Serial); //FOR TESTING PURPOSES ONLY!!!
  
  if(!SD.begin(chipSelect)) //Merge with the other loop in final code, separate now for debugging purposes
  {
    Serial.println("Error: SD Card initialization failure");
    while(1);
  }

  /*if(!accel.begin() || !bmp.begin())
  {
    Serial.println("Error: Sensor initialization failure");
    while(1);
  }*/

  pinMode(armPin, INPUT);

  tabServos.attach(servoPin);
  tabServos.write(thetaMax);

  Serial.println("SD and servos initialized");
  
  /*accel.setRange(ADXL345_RANGE_16_G);
  altitude = bmp.readAltitude(seaPressure); //Set a baseline starting altitude */

  ReadBestFlight();
  Serial.println("Flight Data Loaded");

  //Create the dataLog

  /*tabServos.write(thetaMax); //Run servo startup routine
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
  }*/
  lastT=millis();
  lastPIDT=millis();
  launchT = lastT-t_init;

  dataLog = SD.open(dataFileName, FILE_WRITE);
  if(!dataLog) {
    Serial.println("Error: SD Card could not open file, ensure connection and not faulty");
    while(1);
  }
  
}

void loop() {
  delay(10);
  GetSensorData();
  switch(flightState){
    case WAITING:
      if(digitalRead(armPin)){
        flightState = ARMED;
        tabServos.write(thetaMin);
        delay(1000);
        tabServos.write(thetaMax);
      }
    break;
    case ARMED:
      UpdateBaroBuffers();
      if(accelZ > accelLiftoffThreshold ||  (altitude-launchA) > baroLiftoffThreshold){
        flightState = LAUNCHED;
        launchT = millis();
        saveData = true;
      }
      if(digitalRead(armPin)){
        flightState = WAITING;
        tabServos.write(thetaFlush);
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
      velocity = realV; //CalcAccelVel(velocity);
      if(maxA > altitude + baroApogeeThreshold){
        flightState = APOGEE;
        runPIDControl = false;
        tabServos.write(thetaMax);
        dataLog.flush();
        dataLog.close();
        Serial.println("Ending control phase.");
        while(1); //ONLY FOR TESTING PURPOSES
      }
    break;
    case APOGEE:
      velocity = CalcAccelVel(velocity);
      if(velocity > accelFreefallThreshold)
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
    Serial.print("Set servo angle: "); Serial.println(theta);
  }
  if(saveData){
    if(millis()-lastSDT > sdWaitTime){
      Serial.println("Writing saveData");
      SaveSensorData(dataLog);
      lastSDT = millis();
    }
  }
  if(emergencyRescue)
    tabServos.write(thetaMin);
}

void ReadBestFlight(){
  File inFile = SD.open(inFileName);
  int c = 0; //Counter variable
  if(inFile){
    while(inFile.available()){
      bestVel[c] = inFile.parseFloat();
      bestAlt[c] = inFile.parseFloat();
      c++;
    }
    inFile.close();
  }
  else{
    Serial.println("Error: Unable to open comparison datafile.");
    while(1); //Freeze code if comparison dataset cannot be read
  }
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
  thetaOut = thetaMax-thetaOut; //0 is full extension, MAX is full retraction
  if(thetaOut < thetaMin)
    thetaOut = thetaMin;
  else if(thetaOut > thetaMax)
    thetaOut = thetaMax;
  if(thetaOut > lastTheta + maxStep)
    thetaOut = lastTheta + maxStep;
  if(thetaOut < lastTheta - maxStep)
    thetaOut = lastTheta - maxStep;
  lastTheta=thetaOut;
  return thetaOut; //Since 75 degrees is actually full retraction, not full extension, the output had to be slightly modified
}

void GetSensorData(){
  float dT = 0.01;
  //lastT=millis();
  float W_rocket=50; //pounds
  float C_rocket=0.3, C_tab=1.3, rho_i = 0.0765, A_rocket=0.3068;
  float A_tab=0.0556*cos(theta*PI/(2*thetaMax)); //theta*90/75*PI/180
  accelX=0;accelY=0;
  float rho_real = rho_i*(1-0.1715*realY/5280);

  realA = -32.2 - 0.5*rho_real*(C_rocket*A_rocket+C_tab*A_tab)*pow(realV, 3)/(fabs(realV)*W_rocket);
  realV += realA*dT;
  realY += realV*dT;
  Serial.print("Time: "); Serial.print(millis()); Serial.print(" Altitude: "); Serial.print(realY);
  Serial.print(" Velocity: "); Serial.print(realV); Serial.print(" Acceleration: "); Serial.println(realA);


  altitude=realY;
  accelZ=realA;
  
  /*potValue = map(analogRead(potPin),0,1023,0,269); //Read potentiometer data and map to a displacement angle
  sensors_event_t event; //Read accelerometer data
  accel.getEvent(&event);
  accelX = event.acceleration.x;
  accelY = event.acceleration.y;
  accelZ = event.acceleration.z;
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  lastA = altitude; //Variable to track if new altitude data has come in
  altitude = bmp.readAltitude(seaPressure)-launchA; //Shift all altitude data relative to the starting point*/
  if(altitude > maxA)
    maxA = altitude; //Also track maximum altitude
}

void SaveSensorData(File dataLog){
  
  if(dataLog.isOpen()){
    dataLog.println(millis());
//    dataLog.println(accelX); dataLog.print(",");
//    dataLog.println(accelY); dataLog.print(",");
//    dataLog.println(accelZ); dataLog.print(",");
//    dataLog.println(temperature); dataLog.print(",");
//    dataLog.println(pressure); dataLog.print(",");
//    dataLog.println(altitude); dataLog.print(",");
//    dataLog.println(potValue);

    if(count % 10 == 0){
      Serial.println("Attempting flush");
      dataLog.flush();
      Serial.println("Completed Flush");
    }
    count++;
  }
  else {
    Serial.print("Error: Unable to open "); Serial.println(dataFileName);
    while(1);
  }
}


