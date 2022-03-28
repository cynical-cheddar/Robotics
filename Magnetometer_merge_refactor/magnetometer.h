#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H

# include <Wire.h>
# include <LIS3MDL.h>



class Magnetometer_c {
  public:

  #define CALIBRATION_SAMPLES 70  // Number of compass readings to take when calibrating
  
  // Allowed deviation (in degrees) relative to target angle that must be achieved before driving straight
  #define DEVIATION_THRESHOLD 5
  
  
  float largestCalibrationMagnitude = 0;


  int16_t backgroundFieldMap_xs[90];
  int16_t backgroundFieldMap_ys[90];
  int16_t backgroundFieldMap_zs[90];

  // Change next line to this if you are using the older 3pi+
  // with a black and green LCD display:

  //OLED display;
  //Pololu3piPlus32U4::LCD display;

  LIS3MDL mag;
  
  Motors_c motors;



  
  //Pololu3piPlus32U4::IMU::vector<int16_t> m_max; // maximum magnetometer values, used for calibration

  int16_t m_max_x;
  int16_t m_max_y;
  int16_t m_min_x;
  int16_t m_min_y;

  float bearing = 180;
  
  //Pololu3piPlus32U4::IMU::vector<int16_t> m_min; // minimum magnetometer values, used for calibration

  // create a vector of length 360
  
  /* Configuration for specific 3pi+ editions: the Standard, Turtle, and
  Hyper versions of 3pi+ have different motor configurations, requiring
  the demo to be configured with different parameters for proper
  operation.  The following functions set up these parameters using a
  menu that runs at the beginning of the program.  To bypass the menu,
  you can replace the call to selectEdition() in setup() with one of the
  specific functions.
  */
  
  uint16_t speedStraightLeft; // Maximum motor speed when going straight; variable speed when turning
  uint16_t speedStraightRight;
  uint16_t turnBaseSpeed; // Base speed when turning (added to variable speed)
  uint16_t driveTime; // Time to drive straight, in milliseconds


  void finishedCalibration(){
    tone(6, 1500);
    delay(200);
    tone(6, 1000);
    delay(200);
    noTone(6);
    delay(300);
    tone(6, 1500);
    delay(200);
    tone(6, 1000);
    delay(200);
    noTone(6);
    delay(300);
    tone(6, 1500);
    delay(200);
    tone(6, 1000);
    delay(200);
    noTone(6);
    delay(300);
    tone(6, 1500);
    delay(200);
    tone(6, 1000);
    delay(200);
    noTone(6);
    delay(300);
    tone(6, 1500);
    delay(200);
    tone(6, 1000);
    delay(200);
    noTone(6);
    delay(300);
    tone(6, 1500);
    delay(200);
    tone(6, 1000);
    delay(200);
    noTone(6);
    delay(300);
    tone(6, 1500);
    delay(200);
    tone(6, 1000);
    delay(200);
    noTone(6);
    delay(300);
  }

  
  void selectHyper()
  {
    speedStraightLeft = 70;
    speedStraightRight = 80;
    turnBaseSpeed = 20;
    driveTime = 500;
  }
  
  void selectStandard()
  {
    speedStraightLeft = 30;
    speedStraightRight = speedStraightLeft;
    turnBaseSpeed = 20;
    driveTime = 1000;
  }
  
  void selectTurtle()
  {
    speedStraightLeft = 20;
    speedStraightRight = speedStraightLeft;
    turnBaseSpeed = 40;
    driveTime = 2000;
  }


    // Converts x and y components of a vector to a heading in degrees.
  // This calculation assumes that the 3pi+ is always level.
  float heading(float x, float y)
  {
    float x_scaled =  2.0*(float)(x - m_min_x) / (m_max_x - m_min_x) - 1.0;
    float y_scaled =  2.0*(float)(y - m_min_y) / (m_max_y - m_min_y) - 1.0;
  
    float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
    if (angle < 0)
      angle += 360;
    return angle;
  }

  
  float averageHeading()
    {

      float avg_x = 0;
      float avg_y = 0;
      for(int i = 0; i < 10; i ++)
      {
        mag.read();
        avg_x += mag.m.x;
        avg_y += mag.m.y;
      }
      avg_x /= 10.0;
      avg_y /= 10.0;
     // Serial.println((String) "avg x " + avg_x + " avg y " + avg_y); 
    
      // avg is the average measure of the magnetic vector.
      return heading(avg_x, avg_y);
    }

  float averageHeadingFiltered(int filterIndex){
      float avg_x = 0;
      float avg_y = 0;
      float background_x = backgroundFieldMap_xs[filterIndex];
      float background_y = backgroundFieldMap_ys[filterIndex];
      for(int i = 0; i < 10; i ++)
      {
        mag.read();
        avg_x += (mag.m.x - background_x);
        avg_y += (mag.m.y - background_y);
      }
      avg_x /= 10.0;
      avg_y /= 10.0;
     // Serial.println((String) "avg x " + avg_x + " avg y " + avg_y); 
    
      // avg is the average measure of the magnetic vector.
      return heading(avg_x, avg_y);
  }

  float calculateTeslaSumFiltered(int filterIndex){
      float avg_x = 0;
      float avg_y = 0;
      float avg_z = 0;
      float background_x = backgroundFieldMap_xs[filterIndex];
      float background_y = backgroundFieldMap_ys[filterIndex];
      float background_z = backgroundFieldMap_zs[filterIndex];
      for(int i = 0; i < 10; i ++)
      {
        mag.read();
        avg_x += (mag.m.x - background_x);
        avg_y += (mag.m.y - background_y);
        avg_z += (mag.m.z - background_z);
      }
      avg_x /= 10.0;
      avg_y /= 10.0;
      avg_z /= 10.0;

      // TODO: Find vector direction to further filter

      
     // Serial.println((String) "avg x " + avg_x + " avg y " + avg_y); 


     // Serial.println((String) "Corrected teslas: " + avg_x + " " +avg_y + " " + avg_z);
      // avg is the average measure of the magnetic vector.
      return sqrt(avg_x * avg_x + avg_y*avg_y);
  }

    float calculateTeslaSum(){
      float avg_x = 0;
      float avg_y = 0;
      float avg_z = 0;
      for(int i = 0; i < 10; i ++)
      {
        mag.read();
        avg_x += (mag.m.x);
        avg_y += (mag.m.y);
        avg_z += (mag.m.z);
      }
      avg_x /= 10.0;
      avg_y /= 10.0;
      avg_z /= 10.0;
     // Serial.println((String) "avg x " + avg_x + " avg y " + avg_y); 


     // Serial.println((String) "raw teslas: " + avg_x + " " +avg_y + " " + avg_z);
      // avg is the average measure of the magnetic vector.
      return avg_x + avg_y;
  }

  float calculateTeslaSumMagnitudeFiltered(int filterIndex){
    float avg_x = 0;
      float avg_y = 0;
      float avg_z = 0;
      float background_x = backgroundFieldMap_xs[filterIndex];
      float background_y = backgroundFieldMap_ys[filterIndex];
      float background_z = backgroundFieldMap_zs[filterIndex];
      for(int i = 0; i < 10; i ++)
      {
        mag.read();
        avg_x += (mag.m.x - background_x);
        avg_y += (mag.m.y - background_y);
        avg_z += (mag.m.z - background_z);
      }
      avg_x /= 10.0;
      avg_y /= 10.0;
      avg_z /= 10.0;

      return abs(avg_x) + abs(avg_y) + abs(avg_z);
  }

  

  float calculateFilteredHeading(float currentRotation){
    float angle = 0;
    // get closest step to current rotation
    int index = (int)currentRotation;

    angle = averageHeadingFiltered(index);
    
    return angle;
  }

     // Yields the angle difference in degrees between two headings
  float relativeHeading(float heading_from, float heading_to)
  {
    float relative_heading = heading_to - heading_from;
  
    // constrain to -180 to 180 degree range
    if (relative_heading > 180)
      relative_heading -= 360;
    if (relative_heading < -180)
      relative_heading += 360;
  
    return relative_heading;
  }
  
  // Average 100 vectors to get a better measurement and help smooth out
  // the motors' magnetic interference.
  

  
  // Setup will calibrate our compass by finding maximum/minimum magnetic readings
  void setupMagnetometer()
  {
    // The highest possible magnetic value to read in any direction is 32767
    // The lowest possible magnetic value to read in any direction is -32767

    int16_t running_min_x = 32767;
    int16_t running_min_y = 32767;
    int16_t running_max_x = -32767;
    int16_t running_max_y = -32767;
    unsigned char index;
  
    Serial.begin(9600);
  
    // Initialize the Wire library and join the I2C bus as a master
    Wire.begin();
  
    selectStandard();
    // Check we have intialised commmuncation
      if (!mag.init() ) {  // no..? :(
    
        // Since we failed to communicate with the
        // magnetometer, we put the robot into an infinite
        // while loop and report the error.
        while(1) {
          Serial.println("Failed to detect and initialize magnetometer!");
          delay(1000);
        }
      }
    mag.enableDefault();
    delay(1000);
  
    // To calibrate the magnetometer, the 3pi+ spins to find the max/min
    // magnetic vectors. This information is used to correct for offsets
    // in the magnetometer data.
    motors.turnRightStationary(speedStraightLeft);
  
    for(index = 0; index < CALIBRATION_SAMPLES; index ++)
    {
      Serial.println(index);
      // Take a reading of the magnetic vector and store it in compass.m
      mag.read();
  
      running_min_x = min(running_min_x, mag.m.x);
      running_min_y = min(running_min_y, mag.m.y);
  
      running_max_x = max(running_max_x, mag.m.x);
      running_max_y = max(running_max_y, mag.m.y);
  
      Serial.println(index);
  
      delay(50);
    }
  
    motors.turnRightStationary(0);
  
    /*Serial.print("max.x   ");
    Serial.print(running_max_x);
    Serial.println();
    Serial.print("max.y   ");
    Serial.print(running_max_y);
    Serial.println();
    Serial.print("min.x   ");
    Serial.print(running_min_x);
    Serial.println();
    Serial.print("min.y   ");
    Serial.print(running_min_y);
    Serial.println();*/
  
    // Store calibrated values in m_max and m_min
    m_max_x = running_max_x;
    m_max_y = running_max_y;
    m_min_x = running_min_x;
    m_min_y = running_min_y;


  }

  

  // called every n degrees from north to add background values to arrays
  void calibrationStep(int index){
    float avg_x = 0;
      float avg_y = 0;
      float avg_z = 0;
      for(int i = 0; i < 10; i ++)
      {
        mag.read();
        avg_x += mag.m.x;
        avg_y += mag.m.y;
        avg_z += mag.m.z;
      }
      avg_x /= 10.0;
      avg_y /= 10.0;
      avg_z /= 10.0;
      backgroundFieldMap_xs[index] = avg_x;
      backgroundFieldMap_ys[index] = avg_y;
      backgroundFieldMap_zs[index] = avg_z;


      float stepMagnitude = sqrt(avg_x * avg_x + avg_y*avg_y);
      if(stepMagnitude >largestCalibrationMagnitude ) largestCalibrationMagnitude = stepMagnitude;
  }
  
  void updateMagnetometer()
  {
    float heading, relative_heading;
    int speed;
    static float target_heading = 0;
  
    // Heading is given in degrees away from the magnetic vector, increasing clockwise
    heading = averageHeading();
  
    // This gives us the relative heading with respect to the target angle
    relative_heading = relativeHeading(heading, target_heading);
  
    Serial.print("Target heading: ");
    Serial.print(target_heading);
    Serial.print("    Actual heading: ");
    Serial.print(heading);
    Serial.print("    Difference: ");
    Serial.print(relative_heading);

   // tone(6, heading * 5);
    int headingI = (int)heading;
 //   display.clear();
    
 //   display.println(headingI);
    delay (100);
  }
  };



#endif

  

  
