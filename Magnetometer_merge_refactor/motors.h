// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _MOTORS_H
#define _MOTORS_H
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15
#define FWD LOW
#define REV HIGH


// Class to operate the motor(s).
class Motors_c {
  public:

    int maxPWM = 50;
    int minPWM = 20;
    
    // Constructor, must exist.
    Motors_c() {

    } 

    void initialise() {
      // Set all the motor pins as outputs.
      pinMode(L_PWM_PIN, OUTPUT);
      pinMode(L_DIR_PIN, OUTPUT);
      pinMode(R_PWM_PIN, OUTPUT);
      pinMode(R_DIR_PIN, OUTPUT);
    
      // Set initial direction (HIGH/LOW) for the direction pins.
      digitalWrite(L_DIR_PIN, FWD);
      digitalWrite(R_DIR_PIN, FWD);
    
      // Set initial values for the PWM Pins.
      analogWrite(L_PWM_PIN, 0);
      analogWrite(R_PWM_PIN, 0);
    }

    /*
     * Sets the power of a motor using analogWrite().
     * This function sets direction and PWM (power).
     * This function catches all errors of input PWM.
     *  inputs: 
     *     pwm   accepts negative, 0 and positve values.
     *           Sign of value used to set the direction of the motor.
     *           Values are limited in range [ 0 : 50 ].
     *           Magnitude used to set analogWrite().
     */
    void setMotorPower(String side, float pwm) {
      // Cap values.
      if (pwm > maxPWM) {
        //Serial.println((String)"setMotorPower WARNING: Speed capped " + pwm + " -> " + maxPWM);
        pwm = maxPWM;
      } else if (pwm < -maxPWM) {
        pwm = -maxPWM;
        //Serial.println((String)"setMotorPower WARNING: -ve Speed capped. " + pwm + " -> -" + maxPWM);
      } else if (pwm < 0 && pwm > -minPWM) {
        pwm = 0;
       // Serial.println((String)"setMotorPower WARNING: -ve Speed capped. " + pwm + " -> 0.");
      } else if (pwm > 0 && pwm < minPWM) {
        pwm = 0;
       // Serial.println((String)"setMotorPower WARNING: +ve Speed capped. " + pwm + " -> 0.");
      }
    
      if (side == "left") {
        if (pwm >= 0) digitalWrite(L_DIR_PIN, FWD);
        else digitalWrite(L_DIR_PIN, REV);
        analogWrite(L_PWM_PIN, abs(pwm));
      } else if (side == "right") {
        if (pwm >= 0) digitalWrite(R_DIR_PIN, FWD);
        else digitalWrite(R_DIR_PIN, REV);
        analogWrite(R_PWM_PIN, abs(pwm));
      } else {
        Serial.println((String)"setMotorPower ERROR: Undefined motor name '" + side + "'.");
      }
    }


    void driveStraight(float pwm){
      setMotorPower("right", pwm);
      setMotorPower("left", pwm);
      return;
    }

    void turnRightOneWheel(float pwm){
      if(pwm < 20 && pwm > -20){
        pwm = 20;
      }
      setMotorPower("right", 0);
      setMotorPower("left", pwm);
    }
    void turnLeftOneWheel(float pwm){
      if(pwm < 20 && pwm > -20){
        pwm = 20;
      }
      setMotorPower("right", pwm);
      setMotorPower("left", 0);
    }

    void turnRightStationary(float pwm){
      setMotorPower("right", -pwm);
      setMotorPower("left", pwm);
    }
    void turnLeftStationary(float pwm){
      setMotorPower("right", pwm);
      setMotorPower("left", -pwm);
    }
};



#endif
