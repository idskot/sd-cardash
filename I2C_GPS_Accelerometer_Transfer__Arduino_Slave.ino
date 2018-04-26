/*
 *  Name: GPS And Accelerometer combinational program -- transfer via I2C w/ Arduino as slave (I2C_GPS_Accelerometer_Transfer__Arduino_Slave.ino)
 * 
 *  Description:  Takes input from sensor (accelerometer, gyrometer - LSM6DS3), computes rolling average & triggers an output pin 
 *                that causes Raspberry Pi to request and read data over I2C via sister program (i2c__master_floating_interrupt.py)
 *                Sends GPS coordinates and time, as well as accelerometer data to Pi.
 *  
 *  Pins: D3 - Interrupt Output -- Connects to BCM.12 (Physical pin 32, 5 pins up on the bottom right)
 *        D18 - GPS TX1
 *        D19 - GPS TX2
 *        D20 - SDA (Raspberry Pi && LSM6DS3)
 *        D21 - SCL (Raspberry Pi && LSM6DS3)
 *        
 *                
 *  Created by Scott Levine
 *        April 20th, 2018
 */


/*********************************************************************************************************
         PARAMETER SETS
*********************************************************************************************************/        

/*************************************** FOR ERROR FINDING **********************************************/



/*************************************** FOR ERROR FINDING **********************************************/

/********************************************************************************************************/  
    // For I2C communication
#include <Wire.h>  

#define SLAVE_ADDRESS 0x04
#define FLOATS_SENT 7       // Number of floating values sent

int number = 0;     // Number to send
int state = 0;      // Global state of communication

float floatData[FLOATS_SENT] = {0};



/*********************************************************************************************************/   
    // For GPS setup  
#include <Adafruit_GPS.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
     
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

/*********************************************************************************************************/
  // For Telemetry Setup (LSM6DS3)
#include "SparkFunLSM6DS3.h"

  // DEFINE ROLLING AVERAGE PARAMETERS
#define N               50             // Total number of entries for rolling average
#define THRESHOLD       1000           // Threshold to trigger, must be this amount or greater to trigger Pi interrupt

int16_t accelDataX[N];                 // Declare array to store sensor data for rolling average
int16_t accelDataY[N];                 // Declare array to store sensor data for rolling average
int16_t accelDataZ[N];                 // Declare array to store sensor data for rolling average
int count = 0;                        // Initialize count 

  // DEFINE INTERRUPT
#define INTERRUPTPIN    7             // Pin to use as interrupt to Raspberry Pi
#define INTERRUPTDELAY  250           // Time to hold interrupt pin high for (in ms)
#define INTERRUPTTIMELIMIT  10000     // Minimum time between interrupts (in ms)
long interruptTime = 0;

uint16_t errorsAndWarnings = 0;

//Create instance of LSM6DS3Core
LSM6DS3Core myIMU( I2C_MODE, 0x6A );   //I2C device address 0x6A




/*********************************************************************************************************
      I2C DATA COMMUNICATION FUNCTIONS
*********************************************************************************************************/      

// Function for callback for received data

void receiveData(int byteCount){
 Serial.println("Request to send data made:");
}


// callback for sending data
void sendData(){

      // Send current latitude and longitudinal coordinates (floating point - 4 bytes)
  floatData[0] = GPS.latitude;
  floatData[1] = GPS.longitude;
  floatData[2] = GPS.hour;
  floatData[3] = GPS.minute;
  floatData[4] = GPS.seconds;
  floatData[5] = GPS.lat;
  floatData[6] = GPS.lon;


  Wire.write((byte*) &floatData[0], (FLOATS_SENT*sizeof(float)));   // Writes from the data array, starting at Index 0 and going to Index 'FLOATS_SENT', thus variable via definition
}


/*********************************************************************************************************
        TESTING RANGE FUNCTION
*********************************************************************************************************/ 

 int testRange(int16_t accelData[], int16_t current){
  int16_t rollingAccelAverage = rollingAverage(accelData);      // Call rolling average function to find

  if(abs(current - rollingAccelAverage) >= THRESHOLD)          // IFF diff btwn current & rolling average > Thresh: return 1
  {
    return 1;
  }
  else
  {
    return 0;
  }
} 


/*********************************************************************************************************
        ROLLING AVERAGE FUNCTION
*********************************************************************************************************/        

  // Calculate average of previously stored sensor values held in global array
int16_t rollingAverage(int16_t data[]){
  long average = 0;
  int i;

    // Cycle through all currently stored sensor data
    for(i=0; i<N; i++){
      average+=data[i];
    }
    return average = average/N;
}


/*********************************************************************************************************
        MAIN SETUP
*********************************************************************************************************/        

void setup()
{
   // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);

  while(!Serial); // Wait for Serial
  
  Serial.println("Adafruit GPS library basic test!");
     
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  /*********************************************************************************************************/  
    // SENSOR SETUP && Initialization
  
  Serial.println("Setting up sensor"); 
  //Call .beginCore() to configure the IMU
  if( myIMU.beginCore() != 0 )
  {
    Serial.print("\nDevice Error.\n\n");
  }
  else
  {
    Serial.print("Device OK.\n");
  }
  
  uint8_t dataToWrite = 0;  //Temporary variable

  //Setup the accelerometer
  dataToWrite = 0; //Start Fresh!
  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;

  //Now, write the patched together data
  errorsAndWarnings += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  //Set the ODR bit
  errorsAndWarnings += myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
  dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

  int i = 0;
  int16_t accelAverageX, accelAverageY, accelAverageZ  = 0;
  int16_t temp;
  


  /*********************************************************************************************************/   

      // Initialize sensor data array -- fill array for comparison
  for(i=0; i<N; i++){
  int16_t accelAverage = 0;
     //Acelerometer axis X
  if( myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTX_L_XL) != 0 )
  {
    errorsAndWarnings++;
  }
  accelDataX[i]=temp;
  
     //Acelerometer axis Y 
  if( myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTY_L_XL) != 0 )
  {
    errorsAndWarnings++;
  }
  accelDataY[i]=temp;
  
    //Acelerometer axis Z   
  if( myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTZ_L_XL) != 0 )
  {
    errorsAndWarnings++;
  }
  accelDataZ[i]=temp;
  }    

  Serial.println("Sensor initialized!"); 

  
  /*********************************************************************************************************/     

    // I2C Setup
  Wire.begin(SLAVE_ADDRESS);    // Initialize I2C as slave

    // Define Callbacks for I2C comm
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.println("I2C Ready!");
   

  delay(2500);
}

void loop() // run over and over again
{
/*********************************************************************************************************/   
   // ROLLING AVERAGE -- TELEMETRY
  int16_t accelX, accelY, accelZ;
  int16_t temp;
  int interruptFlag = 0;
  
  //Get all parameters
 
  //Acelerometer axis X
  if( myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTX_L_XL) != 0 )
  {
    errorsAndWarnings++;
  }

    // Save accel data
  accelX = temp;

 // IFF out of range, add to interrupt flag
  interruptFlag += testRange(accelDataX, accelX);
   
  
  //Acelerometer axis Y 
  if( myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTY_L_XL) != 0 )
  {
    errorsAndWarnings++;
  }

  accelY = temp;
  
  interruptFlag += testRange(accelDataY, accelY);
  
  //Acelerometer axis Z  
  if( myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTZ_L_XL) != 0 )
  {
    errorsAndWarnings++;
  }

  accelZ = temp;
  
  interruptFlag += testRange(accelDataZ, accelZ);
  /*
  Serial.print("Current Accel info::   X = "); Serial.print(accelX); Serial.print("   Y = "); Serial.print(accelY); Serial.print("   Z = "); Serial.println(accelZ);
  Serial.print("Average Accel info::   X = "); Serial.print(rollingAverage(accelDataX)); Serial.print("   Y = "); Serial.print(rollingAverage(accelDataY)); Serial.print("   Z = "); Serial.println(rollingAverage(accelDataZ));
  Serial.print("Total Number of Interrupts = "); Serial.println(interruptFlag);
  */

  if(interruptFlag) // IFF there was at least 1 out of range, Interrupt required
    {
      if(millis() - interruptTime > INTERRUPTTIMELIMIT)
      {
        interruptTime = millis();
        Serial.println("Interrupt!");
        digitalWrite(INTERRUPTPIN, HIGH);
        delay(INTERRUPTDELAY);
        digitalWrite(INTERRUPTPIN, LOW);
      }
      else
      {
        Serial.println("Interrupt too quickly!");
      }
    }
  else  // Else save data to arrays
    {
      accelDataX[count] = accelX;
      accelDataY[count] = accelY;
      accelDataZ[count] = accelZ;
    }

    // If count is out of bounds, reset count
  if(count > N)
  {
    count = 0;
  }
  else
  {
    count++;
  }


  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
   // Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
     
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 5000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude/100, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude/100, 4); Serial.println(GPS.lon);
    }
  }

}// END LOOP
