/*
NAME OF THE PROJECT : SL Gavatar
MADE BY - H/W Team

THIS PROJECT READS SENSOR VALUE AND TRY TO UNDERSTAND SYMBOLS GENERATED FROM EGYPTIAN SIGN LANGUAGE CHART
AND DISPLAY ON A SMARTPHONE.
-------PIN CONFIGURATION----------------
A0-A3 & A6 : FLEX SENSOR
D2&D3 : FOR BLUETOOTH RX AND TX
A4&A5 : XPIN AND YPIN FOR ACCELROMETER
*/

#include <SoftwareSerial.h>
#include <Wire.h>
#include "Kalman.h"

#define RESTRICT_PITCH

SoftwareSerial bluetoothSerial(2,3);

String c;

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
//int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine


//variable initializtion


int FLEX_PIN1 = A0; 
int flexADC1 = 0; 
int sensorMin1 = 1023; 
int sensorMax1 = 0;

int FLEX_PIN2 = A1; 
int flexADC2 = 0; 
int sensorMin2 = 1023; 
int sensorMax2 = 0;

int FLEX_PIN3 = A2; 
int flexADC3 = 0; 
int sensorMin3 = 1023; 
int sensorMax3 = 0;

int FLEX_PIN4 = A3; 
int flexADC4 = 0; 
int sensorMin4 = 1023; 
int sensorMax4 = 0;

int FLEX_PIN5 = 0; 
int flexADC5 = 0; 
int sensorMin5 = 1023; 
int sensorMax5 = 0;


void setup() 
{
bluetoothSerial.begin(9600);
Serial.begin(9600);
 Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];
  
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();



while (!Serial) 
{
; // wait for serial port to connect. Needed for native USB port only
}
// callibrating the sensors for adaptivity with different bends
while(millis()<1500)
{


Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();


float flexADC1 = analogRead(FLEX_PIN1); //Reads sensor values
float flexADC2 = analogRead(FLEX_PIN2);
float flexADC3 = analogRead(FLEX_PIN3);
float flexADC4 = analogRead(FLEX_PIN4);
float flexADC5 = analogRead(FLEX_PIN5);

if(flexADC1<sensorMin1)
{
sensorMin1=flexADC1;
}
if(flexADC1>sensorMax1)
{
sensorMax1=flexADC1;
}

if(flexADC2<sensorMin2)
{
sensorMin2=flexADC2;
}
if(flexADC2>sensorMax2)
{
sensorMax2=flexADC2;
}

if(flexADC3<sensorMin3)
{
sensorMin3=flexADC3;
}
if(flexADC3>sensorMax3)
{
sensorMax3=flexADC3;
}
if(flexADC4<sensorMin4)
{
sensorMin4=flexADC4;
}
if(flexADC4>sensorMax4)
{
sensorMax4=flexADC4;
}

if(flexADC5<sensorMin5)
{
sensorMin5=flexADC5;
}
if(flexADC5>sensorMax5)
{
sensorMax5=flexADC5;
}


//}
}
}



void printfun(String cp) //to avoid printing repeating symbols
{
if(cp!=c)
{
bluetoothSerial.println(cp);
Serial.println(cp) ;
c=cp;
}
}






void loop() 
{
  
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

 
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;


 if(bluetoothSerial.available()){ 
// reading sensor value
float flexADC1 = analogRead(FLEX_PIN1);
float flexADC2 = analogRead(FLEX_PIN2);
float flexADC3 = analogRead(FLEX_PIN3);
float flexADC4 = analogRead(FLEX_PIN4);
float flexADC5 = analogRead(FLEX_PIN5);

Serial.println(flexADC1);
Serial.println("1 ^^^^");
Serial.println(flexADC2);
Serial.println("2 ^^^^");
Serial.println(flexADC3);
Serial.println("3 ^^^^");
Serial.println(flexADC4);
Serial.println("4 ^^^^");
Serial.println("782.00");
Serial.println("5 ^^^^");

Serial.println("and the mpu is");

Serial.print(kalAngleX); Serial.print("\t");

Serial.println(kalAngleY); 


 

delay(865);


flexADC1 = constrain(flexADC1,sensorMin1, sensorMax1);
flexADC2 = constrain(flexADC2,sensorMin2, sensorMax2);
flexADC3 = constrain(flexADC3,sensorMin3, sensorMax3);
flexADC4 = constrain(flexADC4,sensorMin4, sensorMax4);
flexADC5 = constrain(flexADC5,sensorMin5, sensorMax5);





if(((flexADC1>=790)&&(flexADC1<=980))&&((flexADC2>=750)&&(flexADC2<=850))&&((flexADC3>=720)&&(flexADC3<=800))&&((flexADC4>=600)&&(flexADC4<=700))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-1)&&(kalAngleY>=-25))){
printfun("\u0623");}    /*أ*/

if(((flexADC1>=790)&&(flexADC1<=980))&&((flexADC2>=750)&&(flexADC2<=850))&&((flexADC3>=500)&&(flexADC3<=680))&&((flexADC4>=700)&&(flexADC4<=820))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=0)&&(kalAngleY>=-10))){
printfun("\u0628");}    /*ب*/

if(((flexADC1>=790)&&(flexADC1<=980))&&((flexADC2>=600)&&(flexADC2<=700))&&((flexADC3>=500)&&(flexADC3<=680))&&((flexADC4>=700)&&(flexADC4<=820))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-20)&&(kalAngleY>=-55))){
printfun("\u062A");}   /*ت*/

if(((flexADC1>=790)&&(flexADC1<=980))&&((flexADC2>=600)&&(flexADC2<=700))&&((flexADC3>=500)&&(flexADC3<=680))&&((flexADC4>=700)&&(flexADC4<=820))&&((flexADC5>=600)&&(flexADC5<=700))&&((kalAngleY<=-20)&&(kalAngleY>=-55))){
printfun("\u062B");}   /*ث*/

if(((flexADC1>=760)&&(flexADC1<=870))&&((flexADC2>=730)&&(flexADC2<=799))&&((flexADC3>=670)&&(flexADC3<=750))&&((flexADC4>=650)&&(flexADC4<=740))&&((flexADC5>=670)&&(flexADC5<=750))&&((kalAngleY<=-25)&&(kalAngleY>=-90))){
printfun("\u062C");}    /*ج*/

if(((flexADC1>=760)&&(flexADC1<=830))&&((flexADC2>=680)&&(flexADC2<=725))&&((flexADC3>=600)&&(flexADC3<=660))&&((flexADC4>=640)&&(flexADC4<=830))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-60)&&(kalAngleY>=-90))){
printfun("\u062D");}    /*ح*/

if(((flexADC1>=760)&&(flexADC1<=830))&&((flexADC2>=680)&&(flexADC2<=725))&&((flexADC3>=600)&&(flexADC3<=660))&&((flexADC4>=540)&&(flexADC4<=630))&&((flexADC5>=710)&&(flexADC5<=800))&&((kalAngleY<=-60)&&(kalAngleY>=-90))){
printfun("\u062E");}    /*خ*/

if(((flexADC1>=820)&&(flexADC1<=980))&&((flexADC2>=750)&&(flexADC2<=850))&&((flexADC3>=660)&&(flexADC3<=710))&&((flexADC4>=620)&&(flexADC4<=730))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-25)&&(kalAngleY>=-90))){
printfun("\u062F");}    /*د*/

if(((flexADC1>=820)&&(flexADC1<=980))&&((flexADC2>=750)&&(flexADC2<=850))&&((flexADC3>=660)&&(flexADC3<=710))&&((flexADC4>=620)&&(flexADC4<=730))&&((flexADC5>=680)&&(flexADC5<=750))&&((kalAngleY<=-25)&&(kalAngleY>=-90))){
printfun("\u0630");}    /*ذ */

if(((flexADC1>=820)&&(flexADC1<=980))&&((flexADC2>=750)&&(flexADC2<=850))&&((flexADC3>=660)&&(flexADC3<=710))&&((flexADC4>=620)&&(flexADC4<=730))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-25)&&(kalAngleY>=-90))){
printfun("\u0631");}    /*ر*/

if(((flexADC1>=820)&&(flexADC1<=980))&&((flexADC2>=660)&&(flexADC2<=750))&&((flexADC3>=680)&&(flexADC3<=750))&&((flexADC4>=780)&&(flexADC4<=850))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-25)&&(kalAngleY>=-90))){
printfun("\u0632");}    /*ز*/

if(((flexADC1>=720)&&(flexADC1<=790))&&((flexADC2>=640)&&(flexADC2<=720))&&((flexADC3>=580)&&(flexADC3<=630))&&((flexADC4>=620)&&(flexADC4<=740))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-20)&&(kalAngleY>=-70))){
printfun("\u0633");}    /*س*/

/*if(((flexADC1>=720)&&(flexADC1<=790))&&((flexADC2>=640)&&(flexADC2<=720))&&((flexADC3>=580)&&(flexADC3<=630))&&((flexADC4>=620)&&(flexADC4<=740))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-20)&&(kalAngleY>=-70))){
printfun("\u0634");}    /*ش*/                /* needs FSR To detect*/ 

if(((flexADC1>=820)&&(flexADC1<=860))&&((flexADC2>=750)&&(flexADC2<=850))&&((flexADC3>=690)&&(flexADC3<=810))&&((flexADC4>=750)&&(flexADC4<=850))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-60)&&(kalAngleY>=-110))){
printfun("\u0635");}   /*ص*/

if(((flexADC1>=790)&&(flexADC1<=980))&&((flexADC2>=750)&&(flexADC2<=850))&&((flexADC3>=720)&&(flexADC3<=800))&&((flexADC4>=600)&&(flexADC4<=700))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-50)&&(kalAngleY>=-90))){
printfun("\u0636");}    /*ض*/

if(((flexADC1>=800)&&(flexADC1<=920))&&((flexADC2>=650)&&(flexADC2<=780))&&((flexADC3>=580)&&(flexADC3<=660))&&((flexADC4>=580)&&(flexADC4<=700))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-60)&&(kalAngleY>=-90))){
printfun("\u0637");}    /*ط*/

if(((flexADC1>=800)&&(flexADC1<=920))&&((flexADC2>=650)&&(flexADC2<=780))&&((flexADC3>=620)&&(flexADC3<=700))&&((flexADC4>=580)&&(flexADC4<=700))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-60)&&(kalAngleY>=-90))){
printfun("\u0638");}    /*ظ*/

if(((flexADC1>=790)&&(flexADC1<=980))&&((flexADC2>=600)&&(flexADC2<=700))&&((flexADC3>=500)&&(flexADC3<=680))&&((flexADC4>=700)&&(flexADC4<=820))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-1)&&(kalAngleY>=-25))){
printfun("\u0639");}   /*ع*/

if(((flexADC1>=800)&&(flexADC1<=950))&&((flexADC2>=650)&&(flexADC2<=700))&&((flexADC3>=580)&&(flexADC3<=660))&&((flexADC4>=590)&&(flexADC4<=680))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=0)&&(kalAngleY>=-30))){
printfun("\u063A");}    /*غ*/

if(((flexADC1>=800)&&(flexADC1<=950))&&((flexADC2>=750)&&(flexADC2<=850))&&((flexADC3>=620)&&(flexADC3<=680))&&((flexADC4>=590)&&(flexADC4<=650))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-50)&&(kalAngleY>=-90))){
printfun("\u0641");}    /*ف*/

if(((flexADC1>=800)&&(flexADC1<=950))&&((flexADC2>=700)&&(flexADC2<=750))&&((flexADC3>=680)&&(flexADC3<=720))&&((flexADC4>=590)&&(flexADC4<=650))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-50)&&(kalAngleY>=-90))){
printfun("\u0642");}    /*ق*/

if(((flexADC1>=720)&&(flexADC1<=790))&&((flexADC2>=640)&&(flexADC2<=720))&&((flexADC3>=580)&&(flexADC3<=630))&&((flexADC4>=740)&&(flexADC4<=850))&&((flexADC5>=500)&&(flexADC5<=700))&&((kalAngleY<=-20)&&(kalAngleY>=-70))){
printfun("\u0643");}    /*ك*/

if(((flexADC1>=790)&&(flexADC1<=880))&&((flexADC2>=730)&&(flexADC2<=820))&&((flexADC3>=560)&&(flexADC3<=640))&&((flexADC4>=560)&&(flexADC4<=680))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-40)&&(kalAngleY>=-90))){
printfun("\u0644");}    /*ل*/

if(((flexADC1>=740)&&(flexADC1<=820))&&((flexADC2>=750)&&(flexADC2<=840))&&((flexADC3>=700)&&(flexADC3<=840))&&((flexADC4>=690)&&(flexADC4<=780))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-35)&&(kalAngleY>=-80))){
printfun("\u0645");}    /*م*/

if(((flexADC1>=820)&&(flexADC1<=980))&&((flexADC2>=750)&&(flexADC2<=850))&&((flexADC3>=660)&&(flexADC3<=710))&&((flexADC4>=620)&&(flexADC4<=730))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-1)&&(kalAngleY>=-25))){
printfun("\u0646");}    /*ن*/

if(((flexADC1>=870)&&(flexADC1<=920))&&((flexADC2>=780)&&(flexADC2<=820))&&((flexADC3>=720)&&(flexADC3<=800))&&((flexADC4>=780)&&(flexADC4<=830))&&((flexADC5>=790)&&(flexADC5<=860))&&((kalAngleY<=-60)&&(kalAngleY>=-110))){
printfun("\u0647");}   /*هـ*/

if(((flexADC1>=790)&&(flexADC1<=980))&&((flexADC2>=750)&&(flexADC2<=850))&&((flexADC3>=720)&&(flexADC3<=800))&&((flexADC4>=600)&&(flexADC4<=700))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=-70)&&(kalAngleY>=-90))){
printfun("\u0648");}    /*و*/

if(((flexADC1>=790)&&(flexADC1<=880))&&((flexADC2>=730)&&(flexADC2<=820))&&((flexADC3>=560)&&(flexADC3<=640))&&((flexADC4>=560)&&(flexADC4<=680))&&((flexADC5>=760)&&(flexADC5<=850))&&((kalAngleY<=0)&&(kalAngleY>=-30))){
printfun("\u0649");}    /*ي*/




delay(120);
  }
}

//----------------------END-----------------------------
