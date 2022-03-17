// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#define LS_LEFT_IN_PIN 18
#define LS_CENTRE_IN_PIN 20
#define LS_RIGHT_IN_PIN 21
#define EMIT_PIN 11
#define LS_PIN_COUNT 3

// Class to operate the linesensor(s).
class LineSensor_c {
  public:

    int lsPins[LS_PIN_COUNT] = {LS_LEFT_IN_PIN, LS_CENTRE_IN_PIN, LS_RIGHT_IN_PIN};
    unsigned long sensorTimeout = 2500;
    float latestSensorTimes[LS_PIN_COUNT] = {0, 0 ,0};
    
    // Constructor, must exist.
    LineSensor_c() {
  
    } 

    
  
    void initialise() {
      enableIRLEDs(true);
      pinMode(LS_LEFT_IN_PIN, INPUT);
      pinMode(LS_CENTRE_IN_PIN, INPUT);
      pinMode(LS_RIGHT_IN_PIN, INPUT);
    }

    void enableIRLEDs(bool lineSensors) {
      pinMode(EMIT_PIN, OUTPUT);
      if (lineSensors) {
        digitalWrite(EMIT_PIN, HIGH);
      } else {
        digitalWrite(EMIT_PIN, LOW);
      }
    }
    
    void disableIRLEDs() {
      pinMode(EMIT_PIN, INPUT);
      digitalWrite(EMIT_PIN, LOW);
    }

    void chargeCapacitors() {
      for (int i = 0; i < LS_PIN_COUNT; i++) {
        pinMode(lsPins[i], OUTPUT);
        digitalWrite(lsPins[i], HIGH);
      }
      delayMicroseconds(10);
      for (int i = 0; i < LS_PIN_COUNT; i++) {
        pinMode(lsPins[i], INPUT);
      }
    }

    float calculateErrorLine(){
      // get motor data
      float eLine = 0.0;
      float wLeft = 0.0;
      float wRight = 0.0;
      // normalise sensors
      float sum = latestSensorTimes[0] + latestSensorTimes[1]+ latestSensorTimes[2];
      //Serial.println("sum:");
     // Serial.println(sum);
      latestSensorTimes[0] = latestSensorTimes[0] / sum;
      latestSensorTimes[1] = latestSensorTimes[1] / sum;
      latestSensorTimes[2] = latestSensorTimes[2] / sum;
      float normSum = latestSensorTimes[0] + latestSensorTimes[1]+ latestSensorTimes[2];
   //   Serial.println("normSum:");
   //   Serial.println(normSum);
      wLeft = latestSensorTimes[2] + latestSensorTimes[1] * 0.50;
      wRight = latestSensorTimes[0] + latestSensorTimes[1] * 0.50;
      eLine = wLeft - wRight;
      
    //  Serial.println("eline:");
    //  Serial.println(eLine);
      return eLine;
    }


    bool isOnLine(float estimatedBlackThreshold){
      for (int i = 0; i < LS_PIN_COUNT; i++) {
        if(latestSensorTimes[i] > estimatedBlackThreshold){
          //Serial.println((String) "estimatedBlackThreshold" + estimatedBlackThreshold + " , " + latestSensorTimes);
          Serial.println((String) "estimatedBlackThreshold:" + estimatedBlackThreshold + " , " + latestSensorTimes[i]);
          return true;
        }
      }
      return false;
    }

    bool isForwardOnLine(float estimatedBlackThreshold){
      float forwardScore = latestSensorTimes[1];
      /*if(!isOnLine(estimatedBlackThreshold)){
        return false;
      }*/
      for (int i = 0; i < LS_PIN_COUNT; i++) {
        if(latestSensorTimes[i] > forwardScore && i != 1){
          return false;
        }
      }
      return true;
    }
    void readLineSensors() {
      chargeCapacitors();
    
      unsigned long sensorTimes[LS_PIN_COUNT];
      for (int i = 0; i < LS_PIN_COUNT; i++) {
        sensorTimes[i] = sensorTimeout;
      }
      int pinsRemaining = LS_PIN_COUNT;  
      unsigned long startTime = micros();
      bool done = false;
    
      while (pinsRemaining > 0) {
        unsigned long currentTime = micros();
        unsigned long elapsedTime = currentTime - startTime;
    
        if (elapsedTime >= sensorTimeout) {
          pinsRemaining = 0;
          break;
        }
        
        for (int i = 0; i < LS_PIN_COUNT; i++) {
          if (sensorTimes[i] == sensorTimeout) { // Only updates the first time the pin is read as LOW.
            if (digitalRead(lsPins[i]) == LOW) {
              sensorTimes[i] = elapsedTime;
              pinsRemaining--;
            }
          }
        }
        
      }
      
     // Serial.println((String)"Left: " + sensorTimes[0] + ", Middle: " + sensorTimes[1] + ", Right:" + sensorTimes[2]);
     
    // Serial.println((String) "Middle: " + sensorTimes[1]);
      for (int i = 0; i < LS_PIN_COUNT; i++) {
        latestSensorTimes[i] = sensorTimes[i];
      }
      
    }
    
};



#endif
