/* *******************************************************************************
* David Wernli & Hollis Harrel
* ECE 450 - LAB #6, sketch a
* ***************************************************************************** */

#include <Robot.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#define LOG 1      //debug variable, turns all serial print statements on for data logging.
#define DB 0       //debug variable, turns all serial print statements on for debugging (not data logging).

  /*--- function declarations ---*/
void serialEvent();
void serialCommand(char ch);
void wallfollower();
void estimateDistance(int front, int side, float loc[2]);
float atan2(int y,int x);
long readVcc();
//void avoidance(){}  //not utilized
//void turn(){}       //not utilized

  /*--- sensors ---*/
const int irPinR = 15;    //pin reserved for Sharp IR input (analog)
const int pingPinR = 52;  //pin reserved for ping sensor input (digital)
const int pingPinF = 53;
//const int cdsPin = 0;     //pin reserved for photoresistor input (analog)

  /*--- servos ---*/
//const int servPin = 8   //pin reserved for servo output (PWM)

  /*--- motor connections ---*/
const int l_EnPin = 7;  //left motor enable pin
const int l_hPin1 = 5;  //left motor hbridge pins
const int l_hPin2 = 4;
const int r_EnPin = 6;  //right motor enable pin (for PWM speed ctrl)
const int r_hPin1 = 3;  //right motor hbridge pins
const int r_hPin2 = 2;

  /*--- globals ---*/
int mode = 0;
int speed = 50;        //speed in PWM format [0 - 255]
float Kp = 7;         //temporary variable to allow tuning of gains without uploading everytime
float Ki = 0;
float Kd = 0;
const String headers = "FR\tRT\tDIST\tANG\tERR\tP term \tI term\tD term\tOutput\tT(ms) \tDIFF";

  /*--- intitialize ---*/
Robot robo(1,1);    //start the Robot, with Serial debugging ON
                  //refer to Robot class definition for capabilities and code
/* --------------------------------------------------------------------
 * -----------                   SETUP                     ------------
 * -------------------------------------------------------------------- */
void setup() {
  //open serial connection
  //Serial.begin(9600);  
  robo.start();
  //set the Robot class up with the pin info for each motor
  robo.setLeft(l_EnPin, l_hPin1, l_hPin2);
  robo.setRight(r_EnPin, r_hPin1, r_hPin2);  
  //print instructions to user - this program requires Serial ON
  Serial.println("- - - Wall Following Robot - - -");
  Serial.println("   Enter 0 for Wallfollower");
  Serial.println("   Enter s,S to stop all movement");  
  //make sure the robot is stopped
  robo.stop();
  robo.setSpeed(speed); //set speed for turning/driving
  Serial.println(headers);
  delay(2000);  //give me a chance to put it down before driving!
 }

/* --------------------------------------------------------------------
 * -----------                 MAIN LOOP                   ------------
 * -------------------------------------------------------------------- */
void loop(){
  switch (mode) {
    case -1:
      //do nothing, this is the 'stop' mode
      break;
    case 0:
      wallfollower();
      break;
    case 2:
      robo.drive();
      break;
    default:
      Serial.println("And you may ask yourself");
      Serial.println("What is this beautiful House?");
      Serial.println("(invalid mode selected, stopping)");
      mode = -1;
  }
 }

/* --------------- SerialEvent ---------------
 occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    serialCommand(inChar);
    } 
 }

/* --------------- serialCommand ---------------
 each character coming in the serial line is evaluated
 against known commands - commands listed in the setup()
 */
void serialCommand(char ch){
  if (ch == 's' || ch == 'S') {
    robo.stop();
    mode = -1;
    Serial.println("STOP COMMAND RECIEVED");
  } else if (ch == '0')  { //user selected wallfollower mode
    mode = 0;
    Serial.println("Go Forward! Move Ahead! It's not too late!");
    Serial.println(headers);
  } else if (ch == 'p')  { //user selected wallfollower mode
    Kp += Serial.parseFloat();
    Serial.print("Kp updated to : ");  Serial.println(Kp);
  } else if (ch == 'i')  { //user selected wallfollower mode
    Ki += Serial.parseFloat();
    Serial.print("Ki updated to : ");  Serial.println(Ki);
  } else if (ch == 'd')  { //user selected wallfollower mode
    Kd += Serial.parseFloat();
    Serial.print("Kd updated to : ");  Serial.println(Kd);
  } else if (ch == '+')  { //user selected wallfollower mode
    speed = Serial.parseInt();
    robo.setSpeed(speed);
    Serial.print("SPEED updated to : ");  Serial.println(speed);
  } else if (ch == '2')  { //user selected mode
    mode = 2;  //just drive - test function
  } else {
    Serial.print(ch);
    Serial.println(" : unrecognized command");
  }
 }
 

/* --------------- wallfollower ---------------
 * main routine for following a wall, to the right of the robot at a set distance
 * robot should start at a point between 5cm and 1.5m from the wall to work well.
 */
void wallfollower(){  
  const int ComputeCeil  =  300;   //the maximum allowed/expected error correction value: +-
  const int ComputeFloor = -300;   //lowering this value makes the robot more responsive to error output
  const int DiffMax = 60;        //the maximum wheel differential for turning, see robo.drive_dif()
  const float maxAngle = 30;     //maximum approach angle permitted
  static int frontDistance = 0;  //input variable for front sensor distance (cm)
  static int sideDistance = 0;   //input variable for side sensor distance (cm)
  static int IR = 0;             //input variable for side mounted IR distance sensor
  static float location[2];      //{distance, approach_angle} - calculated location based on sensor readings
  int differential;              //wheel differential ratio, straight = 0 = 1:1, leftturn = -1 = 100:DiffMax
  //get the readings from the ping sensors
  
  frontDistance = 3000;                 //option 1, set the distance to a fixed value
  //frontDistance = robo.ping(pingPinF);     //option 2, ping sensor
  //frontDistance = robo.IRdistance(irPinF); //option 3, IR sensor
  
  //sideDistance = 3000;                //option 1, set the distance to a fixed value
  //sideDistance = robo.ping(pingPinR);        //option 2, ping sensor
  sideDistance = robo.IRdistance(irPinR);  //option 3, IR sensor
  
  //figure the actual distance and approach angle based on the returned sensor values
  estimateDistance(sideDistance, frontDistance, location);
  //logging statements, formatted as tab-separated-data for import to excel
  if(LOG){ 
    Serial.print(""); Serial.print(frontDistance); 
    Serial.print(" \t"); Serial.print(sideDistance); 
    Serial.print(" \t"); Serial.print(location[0]); 
    Serial.print(" \t"); Serial.print(location[1]); }
  //
  if (location[1] > maxAngle)  {
    //if we're approaching too fast, or there's an upcoming inside corner
    //turn back or go straight - we won't even do the error calculations
    //as we don't want to turn more to the right (or spend time on all the float math).
    differential = map( location[1], 30, 90, -10, -DiffMax);
  } else { 
    //if the angle is less than the max approach
    //calculate the PID error correction
    differential = compute(location[0]);
    differential = min(ComputeCeil, differential); //limit the differntial used to a certain point
    differential = max(ComputeFloor, differential); 
    //map the differential recieved to the differential required for the function, 
    //drive_dif() is limited to -100 to 100 for wheel speeds,
    // we are limiting this further to prevent the robot from pivoting on a single wheel.
    differential = map(differential, ComputeFloor, ComputeCeil, DiffMax, -DiffMax); 
  }
  //DO EET!
  robo.drive_dif(differential);
  //data logging...
  if(LOG){ 
    Serial.print("\t");
    Serial.print(differential);
    //Serial.print("\t");
    //Serial.println(readVcc());
    Serial.println();
  }
}
  
int compute(float distance){
  const int setPoint = 40;      //goal distance from the wall in cm
  //const float Kp = 10;          //P coefficient
  //const float Ki = 0;      //I coefficient
  //const float Kd = 0;          //D coefficient
  static float lastError = 0;  //the value of the last error - stored between function calls
  static float errorSum = 0;   //sum of all errors - stored between calls
  static float dError = 0;         //change in error between this and last function call
  static unsigned long lastTime = 0;  //last time the function was called
  unsigned long now = millis(); //current time in milliseconds 
  float timeChange = (now - lastTime);  //self explanitory?
  float error;                 //current error
  float Pterm;                 //proportional term
  float Iterm;                 //integral term
  float Dterm;                 //derivative term
  int output;
  //
  error = (float)(setPoint - distance);
  errorSum += (error*timeChange);       
  dError = (error - lastError)/timeChange;
  Pterm = (Kp*error);
  Iterm = (Ki*errorSum);
  Dterm = (Kd*dError);
  output = (Pterm + Iterm + Dterm);
  //
  lastError = error;
  lastTime = now;
  //
  if(LOG){ 
    Serial.print(" \t"); Serial.print(error);
    Serial.print(" \t"); Serial.print(Pterm);
    Serial.print(" \t"); Serial.print(Iterm);
    Serial.print(" \t"); Serial.print(Dterm);
    Serial.print(" \t"); Serial.print(output);
    Serial.print(" \t"); Serial.print(now); 
  }
  //
  return (output);
 } 

void estimateDistance(int front, int side, float loc[2]){
  //
  if (front > 250) { 
    loc[0] = side;
    loc[1] = 0;
    return;
  }
  //
  loc[1] = (atan2(side, front));  //approach angle, 0 is perpendicular to wall
  loc[0] = (front * cos(loc[1])); //triangulated distance, assumes flat wall 
  loc[1] *=  57.295; //180/pi, radian to degrees
 }

float atan2(int y,int x){
  //atan2(y,x) = (y/x) - (1/3)(y/x)^3 + (1/5)(y/x)^5...
  float factor;
  float dbx = (float)x;
  float dby = (float)y;
  float y_over_x = 0;
  float result = 0;
  const int n = 3; //terms to iterate over
  int evenOdd;
  //Serial.print("atan2: ");
  if (y>x){
    y_over_x = (dbx/dby);
  } else {
    y_over_x = (dby/dbx);
  }
  //
  for (int i = 0; i < n; ++i)
  {
    factor = (2*i) + 1;
    evenOdd = pow(-1,i%2);
    result += evenOdd * pow(y_over_x,factor) * (1/factor); 
    //Serial.print("\t ");Serial.print(factor);
    //Serial.print("\t ");Serial.print(evenOdd);
    //Serial.print("\t ");Serial.println(result);
  }
  //Serial.print("\t ");Serial.print(dbx);
  //Serial.print("\t ");Serial.println(dby);
  if (y>x){
    return result;
  } else {
    return 1.571-result;
  }
 }
 
 long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

