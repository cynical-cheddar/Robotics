#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "magnetometer.h"
#include "pid.h"
#include "results.h"
#define YELLOW_LED_PIN 13


Motors_c motors;
Kinematics_c kinematics;
Magnetometer_c magnetometer;

// magnetometer calibrator 
int currentStep = 0;
float currentStepTargetRotation = 0;
//-----------
int darknessOffset = 0;
bool foundLine = false;
int defaultMotorPower = 20;
int fastMotorPower = 20;
bool calibrating = false;

const int calibrationArraySize = 100;
int calibrationIndex = 0;

float estimatedWhiteValue = 0;
float estimatedBlackValue = 0;
float estimatedWhiteThreshold = 0;
float estimatedBlackThreshold = 0;
float blackThresholdErrorPercent = 40;

double timeLeftPath = 0;
double timeSinceLeftPath = 0;
double countdown = 0;
double countdownMax = 1000;
// go home if you reach 1 seconds with no path
double leftPathTimeout = 4000;
unsigned long deltaTime = 0;
// odometry settings
bool testingOdometry = false;
float testingDistance = 2.5;

float lastForwardAngle = 0;

float northRelativeRotation = 0;



// defined when calibration is exited and we are seeking line / starting test
long movement_time_start = 0;

float bestMagneticRotation = 180;
float bestMetalAngle = 180;
#define STATE_SETUP  0
#define STATE_CALIBRATING  1
#define STATE_CALIBRATING_FINISH  2
#define STATE_SEEKING_START  3
#define STATE_FOLLOWING  4
#define STATE_SEEKING  5
#define STATE_TEST_ODOMETRY_FORWARD  6
#define STATE_TEST_ODOMETRY_HOME  7

#define STATE_MAGNETOMETER_GOVERNOR  8
#define STATE_MAGNETOMETER_CALIBRATE  9
#define STATE_MAGNETOMETER_SEEK_NORTH 10
#define STATE_MAGNETOMETER_METAL_DETECT 11
#define STATE_MAGNETOMETER_METAL_SEEK 12
#define STATE_MAGNETOMETER_ROTATE_TO_BEST_MAGNETIC_ANGLE 13
#define STATE_FINISH 14
int currentState = 0;
int calibrationRotationCount = 1;
int calibrateWheel = 0;

float targetRotation = 0;
unsigned long currentDetectionTime = 0;
unsigned long detectionTimePerOrientation = 350000;

float divisionSteps = 20;
float divisionTotal = 360;
int cyclesCompleted = 0;
int currentDivisionStep = 0;
float stepSize = 18;
int fullRotationsIntoCurrentRotation = 0;



// ===========================================CALIBRATION========================================


// WOT WE SHOULD GO AND DO FOR EXPERIMENT LIKE IT'S 2016 AND WE'RE DOING FOOKIN GCSE PHYSICS

//  HYPOTHESIS - WE SHOULD EB ABLE TO DETECT FERROUS MATERIAL FROM THE ERROR SIGNAL BETWEEN MAGNETOMETER READINGS AND THE EARTH'S MAGNETIC FIELD
// WE THINK THERE WILL BE A MINIMUM SIZE OF FERROUS OBJECT WE WILL BE ABLE TO DETECT. THIS IS BECAUSE OF THE GENERAL NOISE IN MAGNETIC FIELD. READINGS +- GAUSS. THIS WILL BE AT A LIMITED RESOLUTION.


// ======================================================================================================================

// Runs once.
void setup() {

  delay(1000);
  
  currentState = STATE_CALIBRATING;

  
  // Start serial, send debug text.
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  motors.initialise();
  pinMode(YELLOW_LED_PIN, OUTPUT);
  digitalWrite(YELLOW_LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.println("Setup complete.");


  magnetometer.setupMagnetometer();
  delay(1000);
  //currentState = STATE_MAGNETOMETER_LOOP;
  currentState = STATE_MAGNETOMETER_SEEK_NORTH;
  //calibrate();

  
  
}


bool isBestMagneticRotation(float newAngle){

  // reduce rotation to deviation from north
  float newDifference = newAngle;
  if(newAngle > 180) newDifference = 360 - newAngle;

  float oldDifference = bestMetalAngle;
  if(bestMetalAngle > 180) oldDifference = 360 - bestMetalAngle;
  
  Serial.println((String) "newAngle " + newAngle + " bestMetalAngle: " + bestMetalAngle);
  Serial.println((String) "new difference " + newDifference + " oldDifference: " + oldDifference);
  
  

  if(newDifference < oldDifference) return true;
  else return false;

  
}


// Repeats.
void loop() {
  
  unsigned long exec_time_start;
  exec_time_start = micros();

  //lineSensors.readLineSensors();
  kinematics.updateLoop();
  
  /*if(lineSensors.isOnLine(estimatedBlackThreshold)){
    Serial.println( "On Line" );
    foundLine = true;
  }
  else{
    Serial.println( "Off Line" );
  }*/



    // state is used to dictate the searching for metal cycle
    if (currentState == STATE_MAGNETOMETER_GOVERNOR){
      motors.turnRightStationary(0);
      cyclesCompleted += 1;
      divisionTotal = divisionTotal / 2;
      delay(100);
      
      tone(6, 2000);
      delay(100);
      noTone(6);
      float currentAngle= kinematics.currentRotation * (180/PI);
      fullRotationsIntoCurrentRotation = (currentAngle / 360) -1;

      if(cyclesCompleted > 15){
        currentState = STATE_FINISH;
      }
      else{
        currentState = STATE_MAGNETOMETER_ROTATE_TO_BEST_MAGNETIC_ANGLE;
      }
      
    }
    if(currentState == STATE_FINISH){
      float currentAngle = kinematics.currentRotation * (180/PI);
      motors.turnLeftStationary(20);
      float resetAngle = (bestMagneticRotation + fullRotationsIntoCurrentRotation*360);
      if(currentAngle < resetAngle){
        motors.turnRightStationary(0);
        tone(6, 1000);
        delay(200);
        tone(6, 1500);
        delay(200);
        noTone(6);
        delay(300);
      }
    }
    
    if(currentState == STATE_MAGNETOMETER_ROTATE_TO_BEST_MAGNETIC_ANGLE){
      motors.turnLeftStationary(40);

      
      float currentAngle = kinematics.currentRotation * (180/PI);

      

      float resetAngle = (bestMagneticRotation + fullRotationsIntoCurrentRotation*360) - (divisionTotal/2);
      //targetAngle = (bestMagneticRotation + fullRotationsIntoCurrentRotation*360) + (divisionTotal/2);

      

      
      if(currentAngle > resetAngle -5  && currentAngle < resetAngle +5){
        motors.turnRightStationary(0);
        delay(100);
      
        tone(6, 250);
        delay(100);
        noTone(6);
        targetRotation = currentAngle + divisionTotal;
        currentState = STATE_MAGNETOMETER_METAL_DETECT;
      }
    }




    
    else if (currentState == STATE_MAGNETOMETER_CALIBRATE){

      int steps = 90;
      int interval = 360 / steps;
    //  Serial.println(kinematics.currentRotation);
      if(currentStep < steps){
        // motors.turnRightStationary(20);
        
         
        if(kinematics.currentRotation*(180 / PI) >= currentStepTargetRotation){
          motors.turnRightStationary(0);
          delay(50);
          currentStep += 1;
          currentStepTargetRotation += interval;
          Serial.println((String) " Magnetometer heading in calibration:  " + magnetometer.averageHeading());
          Serial.println((String) " Kinematics heading in calibration:  " + kinematics.currentRotation*(180/PI));
          magnetometer.calibrationStep(currentStep);
          //tone(6,10 * currentStep);
          delay(50);
          noTone(6);
          if(calibrateWheel == 0)calibrateWheel = 1;
          else calibrateWheel = 0;
          
        }
        else{
          if(calibrateWheel == 0){
            motors.turnRightOneWheel(20);
          }
          if(calibrateWheel == 1){
            motors.turnLeftOneWheel(-20);
          }
          
        }
      }
      else{
        Serial.println((String) " Done magnetometer calibration");

        motors.turnRightStationary(0);
        magnetometer.finishedCalibration();
        
        currentState = STATE_MAGNETOMETER_METAL_DETECT;
        targetRotation = kinematics.currentRotation*(180 / PI) + divisionTotal;
      }
    }

    
    else if (currentState == STATE_MAGNETOMETER_METAL_DETECT){
      // if we have moved enough, halve our search pattern
      //if(currentDivisionStep >= divisionSteps) currentState = STATE_MAGNETOMETER_GOVERNOR;
      
      motors.turnRightStationary(20);
      
      currentDetectionTime += deltaTime;
      // wait n seconds until changing state
      
      if(kinematics.currentRotation*(180 / PI) > targetRotation){
        currentDetectionTime = 0;
        currentState = STATE_MAGNETOMETER_GOVERNOR;
       // targetRotation = kinematics.currentRotation*(180/PI) + stepSize;
       
      }
      float currentAngle = kinematics.currentRotation * (180/PI);

      // convert current rotation to simply be in 360 degree bounds
      while(currentAngle > 360){
        currentAngle -= 360;
      }
      while(currentAngle < 0){
        currentAngle += 360;
      }
      
      float metalAngle = magnetometer.calculateFilteredHeading(currentAngle/4);
      float teslas = magnetometer.calculateTeslaSumFiltered(currentAngle/4);
    //  float tmag = magnetometer.calculateTeslaSumMagnitudeFiltered(currentAngle/4);

      float compassAngle = magnetometer.averageHeading();
      float teslasUnfiltered = magnetometer.calculateTeslaSum();
     // float tmagUnfiltered = magnetometer.calculateTeslaSumMagnitudeFiltered(currentAngle/4);
      //tone(6, metalAngle*10);
      analogWrite( YELLOW_LED_PIN, metalAngle*2 );

     // Serial.println((String)"teslas: " + teslas);
     // Serial.println((String)"tmag: " + tmag);

      
     // Serial.println((String)"teslas: " + teslas);
     // Serial.println((String)"teslasUnfiltered: " + teslasUnfiltered);
     // delay(500);

      // if the teslas are greater than a threshold, detect magnets and show existence. Do this while turning slowly
      if(abs(teslas) > magnetometer.largestCalibrationMagnitude*1.75){
       // tone(6, abs(teslas));

        // there is some sort of field here, calculate the metal angle
        float metalAngle = magnetometer.averageHeadingFiltered(currentAngle/4);
        // if this is angle with the least deviation from odometric north, then this is our new best angle
        if(isBestMagneticRotation(metalAngle)){
           motors.turnRightStationary(0);
          Serial.println("NEW BEST!!!!!!!!!!!!!!!!!!!!");
          bestMagneticRotation = currentAngle;
          bestMetalAngle = metalAngle;
          tone(6, 100);
          delay(400);
          noTone(6);
          Serial.println((String) "Best angle: " + bestMagneticRotation);
          Serial.println((String) "Best bestMetalAngle: " + bestMetalAngle);
        }
      }
      else{
        noTone(6);
      }
    //  Serial.println((String)"tmag: " + tmag);
     // Serial.println((String)"metalAngle: " + metalAngle);
      
    }

    else if (currentState == STATE_MAGNETOMETER_METAL_SEEK){
      /*float currentAngle = kinematics.currentRotation * (180/PI);

      
      // convert current rotation to simply be in 360 degree bounds
      while(currentAngle > 360){
        currentAngle -= 360;
      }
      while(currentAngle < 0){
        currentAngle += 360;
      }*/


      // rotate 45 degrees until having reached strongest field + lowest magnetic deviation
      // we will know the best orientation. after a complete revolution, go to this 
      motors.turnRightStationary(20);
      if(kinematics.currentRotation*(180/PI) >= targetRotation){
        tone(6, 200);
        delay(100);
        noTone(6);
        currentDivisionStep += 1;
        currentState = STATE_MAGNETOMETER_METAL_DETECT;
      }
      

      
    }
    else if (currentState == STATE_MAGNETOMETER_SEEK_NORTH){
      // 180 is magic number used for size of backgroundFieldMap_xs
      
      motors.turnRightStationary(0);
      delay(1000);
      // firstly, rotate to north
      while(abs(magnetometer.averageHeading()) > 10){
        Serial.println((String) " Magnetometer heading:  " + magnetometer.averageHeading());
        Serial.println((String) " Kinematics heading:  " + kinematics.currentRotationCutoff);
        motors.turnRightStationary(20);
        kinematics.updateLoop();
       // tone(6,100);
       // delay(10);
      }
      motors.turnRightStationary(0);
      delay(100);
      tone(6,100);
      delay(100);
      noTone(6);
      delay(100);
      tone(6,200);
      delay(100);
      noTone(6);
      delay(100);

      kinematics.updateLoop();
      northRelativeRotation = kinematics.currentRotationCutoff;

      // set up the encoders, stay at 0 degrees bearing before this
      setupEncoder0();
      setupEncoder1();

      
      currentState = STATE_MAGNETOMETER_CALIBRATE;
      currentStep=0;
    }
    
    delay(30);
    // Store the time afterwards
    unsigned long exec_time_end;
    exec_time_end = micros();
    deltaTime = exec_time_end - exec_time_start;
    // Report total execution time
    //Serial.println( (exec_time_end - exec_time_start) );

  
}
