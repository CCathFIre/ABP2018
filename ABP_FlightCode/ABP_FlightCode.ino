/*
 * Notre Dame Rocket Team Air-Breaking Payload Flight Code Version 1.0
 * 
 * Authors: Aidan McDonald, Tommy Flanagan
 * Last Update: 3/5/2018
 * 
 * Update description: Post-test flight update
 * 
 * Completed Items:
 * Core flight-tracking switch case structure
 * PID control loop- load comparison data, calculate error, output angle
 * Accelerometer and barometer sensor data incorporation
 * SD saving/loading data running consistently
 * Servo startup routine for visual confirmation of payload functionality
 * Backup "emergency rescue" mode for if the rocket enters freefall
 * Encoder jam thresholds tuned
 * Code ground-tested in a simulated environment
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
#define WAITING -1
#define ARMED 0
#define LAUNCHED 1
#define BURNOUT 2
#define APOGEE 3
#define LANDED 4

//Other useful constants
const int chipSelect = 28;
const String dataFileName = "datalog.txt";
const String inFileName = "BESTFL~1.TXT";
const int preCalcSize = 1887; //Number of data points in the pre-calculated ideal flight layout
const int potPin = A1;
const int servoPin = 6;
const int armPin = 7;
const int baroRegSize = 10; //Number of data points to use in linear regression
const float seaPressure = 1013.25;
const int sdWaitTime = 70; //millis
const float accelLiftoffThreshold = 50; //m/s^2
const float baroLiftoffThreshold = 25; //m
const float accelBurnoutThreshold = -5; //m/s^2
const float baroApogeeThreshold = 5; //m
const float baroLandedThreshold = 40; //m
const float accelFreefallThreshold = 40; //m/s
const float gravOffset = -10; //m/s^2, compensate for measured acceleration due to gravity
const float thetaMin = 0; //Degrees. Note that the mechanism is such that thetaMin causes full extension and thetaMax causes full retraction.
const float thetaFlush = 60;
const float thetaMax = 70;
const float thetaOff = 10;
const float maxStep = 10; //Degrees
const float servoJamThreshold = 12; //Degrees, approx. two standard deviations

//Flags
bool pushed = false;
bool armed = false;
bool saveData = false;
bool runPIDControl = false;
bool emergencyRescue = false;

//Control variables
int flightState = WAITING; 
float accelX, accelY, accelZ;
float temperature, pressure, altitude;
float potValue;
float lastA, lastCalcT, lastPIDT, launchT, launchA;
int lastSDT=0;
float bestAlt[preCalcSize], bestVel[preCalcSize];
float theta, velocity = 0, maxA = 0, lastTheta=theta;
float integralTerm = 0, lastError = 0;
float baroRegArr[baroRegSize], timeRegArr[baroRegSize];
int buttonArray[10] = {0,0,0,0,0,0,0,0,0,0};

void setup() {
  //Freeze the code if a sensor initialization failure occurs
  if(!SD.begin(chipSelect) || !accel.begin() || !bmp.begin())
  {
    while(1);
  }
  Serial.println("Starting setup");
  pinMode(armPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  tabServos.attach(servoPin);
  tabServos.write(thetaMax+thetaOff); //Fully retract drag tabs initially
  
  accel.setRange(ADXL345_RANGE_16_G);
  altitude = bmp.readAltitude(seaPressure); //Set a baseline starting altitude
  launchA = altitude;

  ReadBestFlight();
  PrintHeader();
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  GetSensorData();
  
  switch(flightState){ //Main control section- runs commands and sets flags based on the current state, updates state where necessary
     case WAITING:
      RunButtonControl();
      if(armed){
        flightState = ARMED;
        altitude = bmp.readAltitude(seaPressure); //Set a new baseline starting altitude
        launchA = altitude;
        File dataLog = SD.open(dataFileName, FILE_WRITE);
        if(dataLog){
          dataLog.print("Launch altitude: "); dataLog.flush();
          dataLog.println(altitude); dataLog.flush();
          dataLog.close();
        }
        tabServos.write(thetaMin+thetaOff);
        delay(1000);
        tabServos.write(thetaFlush+thetaOff);
        delay(500);
        tabServos.write(thetaMin+thetaOff);
        delay(1000);
        tabServos.write(thetaFlush+thetaOff);
      }
    break;
    case ARMED:
      UpdateBaroBuffers();
      RunButtonControl();
      if(accelZ > accelLiftoffThreshold ||  (altitude-launchA) > baroLiftoffThreshold){
        flightState = LAUNCHED;
        launchT = millis();
        saveData = true;
      }
      if(!armed){
        flightState = WAITING;
        tabServos.write(thetaMin+thetaOff);
        delay(1000);
        tabServos.write(thetaMax+thetaOff);
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
        tabServos.write(thetaFlush+thetaOff);
        File dataLog = SD.open(dataFileName, FILE_WRITE);
        if(dataLog){
          dataLog.print("Apogee: "); dataLog.flush();
          dataLog.println(maxA); dataLog.flush();
          dataLog.close();
        }
      }
    break;
    case APOGEE:
      velocity = CalcAccelVel(velocity);
      if(fabs(velocity) > accelFreefallThreshold)
        emergencyRescue = true;
      if(fabs(altitude - launchA) <= baroLandedThreshold){
        flightState = LANDED;
        tabServos.write(thetaMax+thetaOff);
        saveData = false;
        emergencyRescue = false;
      }
    break;
    case LANDED:
    break;
  }
  
  //Run other subroutines based on the states of flags
  if(runPIDControl){ 
    float error = CalcError(altitude-launchA, velocity, (int)(millis()-launchT)/10);
    theta = PID(error, lastError, integralTerm, millis()-lastPIDT);
    lastError = error;
    lastPIDT = millis();
    tabServos.write(theta+thetaOff); //Account for occasional servo slippage in testing by adding a constant offset
  }
  if(saveData){
    if(millis()-lastSDT > sdWaitTime){ //Only save data once every few cycles for the sake of processing speed
      SaveSensorData();
      lastSDT = millis();
    }
  }
  if(emergencyRescue)
    tabServos.write(thetaMin+thetaOff); //Fully deploy tabs if free fall is occurring

}


//Functions:
void ReadBestFlight(){ //Read in ideal velocity and altitude data
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
    while(1); //Freeze code if comparison dataset cannot be read
  }
}

void UpdateBaroBuffers(){ //Cycle barometer buffer data for linear regression  purposes
  for(int c = 0; c < baroRegSize-1; c++){
    baroRegArr[c] = baroRegArr[c+1];
    timeRegArr[c] = timeRegArr[c+1];
  }
  baroRegArr[baroRegSize-1] = altitude;
  timeRegArr[baroRegSize-1] = millis();
}

float CalcBaroVel(){ //Calculate velocity based on stored barometric data
  float sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
  for(int c = 0; c < baroRegSize; c++){ //Calculate variance and covariance components
    sumX += timeRegArr[c];
    sumY += baroRegArr[c];
    sumXX += pow(timeRegArr[c],2);
    sumXY += timeRegArr[c]*baroRegArr[c];
  }
  double slope = (baroRegSize*sumXY - sumX*sumY) / (baroRegSize*sumXX - sumX*sumX); //Slope = covariance/variance
  return slope;
}

float CalcAccelVel(float lastVel){ //Perform numeric integration to calculate velocity based on acceleration data
  float newVel = lastVel + accelZ*(millis()-lastCalcT)/1000;
  lastCalcT = millis();
  return newVel;
}

float CalcError(float realAlt, float realVel, int startT){ //First PID control function
  //Looks up closest matching altitude point in the model data, and calculates error based on the corresponding model velocity
  bool match = false;
  
  if(startT > preCalcSize-1) //Limit starting point to the size of the array
    startT = preCalcSize-1;
  
  int c = startT; //To optimize search time, start at the current time point in the model dataset
  int last_c = c;

  while(!match){ //Find closest altitude in buffer to current altitude
    float test = fabs(realAlt - bestAlt[c]); //Calculate delta-Y at, above, and below current test value
    float above = fabs(realAlt - bestAlt[c+1]);
    float below = fabs(realAlt - bestAlt[c-1]);
    last_c = c;
    if(test <= above && test <= below) //If current point has least error, we have a match
      match = true;
    else if (above < test) //Otherwise, go up or down accordingly
      c++;
    else if (below < test)
      c -= 1;
    if(last_c == c) //Backup code in case an infinite search loop is entered
      match = true;
  }
  return (realVel - bestVel[c]); //Return difference in velocities at the given altitude point
}

float PID(float error, float lastE, float &iTerm, int deltaT){ //Second PID control function; returns an output angle based on the error
  static float cP = 80; //P constant
  static float cI = 0; //I constant
  static float cD = 4; //D constant
  float dT = (float)deltaT/1000; //Variable delta-T term

  float pTerm = error*cP; //Calculate each term
  iTerm = iTerm + error*cI*dT; //The i-term is passed by reference and updated throughout flight
  float dTerm = (error - lastE)*cD/dT;

  float thetaOut = pTerm + iTerm + dTerm; //calculate intended output angle
  thetaOut = thetaFlush-thetaOut; //Flip because min is full extension while flush is full retraction
  if(thetaOut < thetaMin) //Limit the output to the range of possible values
    thetaOut = thetaMin;
  else if(thetaOut > thetaFlush)
    thetaOut = thetaFlush;

  //Subroutine to limit servo jitteriness by restricting how far one output can deviate from the previous output
  if(thetaOut > lastTheta + maxStep)
    thetaOut = lastTheta + maxStep;
  if(thetaOut < lastTheta - maxStep)
    thetaOut = lastTheta - maxStep;
  lastTheta=thetaOut;
  
  return thetaOut; //With all transformations complete, return the output angle
}

void GetSensorData(){ //Read in data from all sensors
  potValue = analogRead(potPin); //Read potentiometer data
  sensors_event_t event; //Read accelerometer data
  accel.getEvent(&event);
  accelX = event.acceleration.x;
  accelY = event.acceleration.y;
  accelZ = event.acceleration.z + gravOffset;
  temperature = bmp.readTemperature(); //Read barometer data
  pressure = bmp.readPressure();
  lastA = altitude; //Variable to track if new altitude data has come in
  altitude = bmp.readAltitude(seaPressure);
  if(altitude > maxA)
    maxA = altitude; //Also track maximum altitude
}

void SaveSensorData(){ //Save data from all sensors
  File dataLog = SD.open(dataFileName, FILE_WRITE);
  if(dataLog){
    dataLog.print(millis()); dataLog.print(","); dataLog.flush();
    dataLog.print(accelX); dataLog.print(","); dataLog.flush();
    dataLog.print(accelY); dataLog.print(","); dataLog.flush();
    dataLog.print(accelZ); dataLog.print(","); dataLog.flush();
    dataLog.print(velocity); dataLog.print(","); dataLog.flush();
    dataLog.print(temperature); dataLog.print(","); dataLog.flush();
    dataLog.print(pressure); dataLog.print(","); dataLog.flush();
    dataLog.print(altitude); dataLog.print(","); dataLog.flush();
    dataLog.print(lastTheta); dataLog.print(","); dataLog.flush();
    dataLog.print(potValue);  dataLog.print(","); dataLog.flush();
    dataLog.print(Check_Jam()); dataLog.print(","); dataLog.flush();
    dataLog.println(flightState); dataLog.flush();
    dataLog.close();
  }
}

void PrintHeader(){ //Print a descriptive header to the SD datalog
  File dataLog = SD.open(dataFileName, FILE_WRITE);
  if(dataLog){
    dataLog.print("Time,"); dataLog.flush();
    dataLog.print("X Accel,"); dataLog.flush();
    dataLog.print("Y Accel,"); dataLog.flush();
    dataLog.print("Z Accel,"); dataLog.flush();
    dataLog.print("Vertical Velocity,"); dataLog.flush();
    dataLog.print("Temperature,"); dataLog.flush();
    dataLog.print("Pressure,"); dataLog.flush();
    dataLog.print("Altitude,"); dataLog.flush();
    dataLog.print("Intended Position,"); dataLog.flush();
    dataLog.print("Encoder Value,"); dataLog.flush();
    dataLog.print("Jammed?,"); dataLog.flush();
    dataLog.println("Flight State"); dataLog.flush();
    dataLog.close();
  }
}

bool Check_Jam(){ //Check if the tabs are jammed
  float realTheta = (potValue)/12; //Always calibrate these constants before flight!!
  if(fabs(realTheta-lastTheta) > servoJamThreshold)
    return true;
  else
    return false;
}

void RunButtonControl(){ //Button debouncing subroutine for arming / disarming the rocket
  float threshold = 0.8;
  for(int c=9; c>0; c--)
    buttonArray[c] = buttonArray[c-1];
  buttonArray[0] = digitalRead(armPin);
  float avg = 0;
  for(int c=0; c<10; c++)
    avg += buttonArray[c];
   avg /= 10;
  if(!pushed && avg > threshold){
    armed = !armed;
    pushed = true;
  }
  else if(avg <= threshold)
    pushed = false;
}

