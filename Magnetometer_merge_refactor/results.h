// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _RESULTS_H
#define _RESULTS_H

# include <EEPROM.h>

/*
 * A known problem is that removing the USB cable during
 * the robot operation can cause very irratic behaviour.
 * Plugging in a USB cable can also reset your robot.
 * The root cause is not known (memory error?), but the 
 * following example provides a solution.
 */
# include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
# define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

# define RESULTS_DIM 100
# define EEPROM_MAX_COUNT floor(1000 / RESULTS_DIM)

class Results_c {
  public:

  int volatileCount = 0;
  int EEPROMCount = 0;
  float resultsBuffer[RESULTS_DIM];
  float outputBuffer[RESULTS_DIM];

  bool writeBufferToEEPROM() {
    for (int i = 0; i < RESULTS_DIM; i++) {
      EEPROM.update(EEPROMCount * RESULTS_DIM + i, resultsBuffer[i]);
    }
    EEPROMCount++;
    volatileCount = 0;
    if (EEPROMCount == EEPROM_MAX_COUNT) {
      return false;
    } else {
      return true;
    }
  }
  
  bool addResult(float result) {
    resultsBuffer[volatileCount] = result;
    volatileCount++;
    if (volatileCount == RESULTS_DIM) {
      return writeBufferToEEPROM();
    } else {
      return true;
    }
  }

  /*
   * Checks if the Serial connection is alive
   * before it attempts to transmit.
   * This seems to be more stable when:
   * - short amounts of data is transmitted
   * - there is small delay between transmission
   * You'll have to check that your data is 
   * reported without any missing elements.
   */
  void reportResultsOverSerial() {
  
    // Print millis for debug so we can 
    // validate this is working in real
    // time, and not glitched somehow
    //if( SERIAL_ACTIVE ) Serial.print( "Time(ms): " );
    //if( SERIAL_ACTIVE ) Serial.println( millis() );
    //delay(1);
  
    // Loop through array to print all 
    // results collected
    for (int i = 0; i < EEPROMCount; i++) {
      for (int j = 0; j < RESULTS_DIM; j++) {
        outputBuffer[j] = EEPROM.read(i * RESULTS_DIM + j);
        delay(1);
      }
      for (int k = 0; k < RESULTS_DIM; k++) {
        if( SERIAL_ACTIVE ) Serial.println(outputBuffer[k]);
        delay(1);
      }
    }

    for (int i = 0; i < volatileCount; i++) {
        if( SERIAL_ACTIVE ) Serial.println(resultsBuffer[i]);
        delay(1);
      }
    
  
    if( SERIAL_ACTIVE ) Serial.println( "---End of Results ---\n\n" ); 
    //if( SERIAL_ACTIVE ) Serial.println( "\n\n" ); 
  }

};



#endif
