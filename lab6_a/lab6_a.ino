/* *******************************************************************************
* David Wernli & Hollis Harrel
* ECE 450 - LAB #6, sketch a
* ***************************************************************************** */

#include <Robot.h>
#define LOG 1      //debug variable, turns all serial print statements on for data logging.
#define DB 0       //debug variable, turns all serial print statements on for debugging (not data logging).

  /*--- function declarations ---*/
void serialEvent();
void serialCommand(char ch);
void wallfollower();
void estimateDistance(int front, int side, float loc[2]);
float atan2(int y,int x);
void avoidance(){}
void turn(){}

  /*--- sensors ---*/
const int irPin = 15;    //pin reserved for Sharp IR input (analog)
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
int speed = 80;        //speed in PWM format [0 - 255]
const String headers = "FR\tRT\tDIST\tANG\tERR\tP term \tI term\tD term\tOutput\tTime(ms)";

  /*--- intitialize ---*/
Robot robo(1);    //start the Robot, with Serial debugging ON
                  //refer to Robot class definition for capabilities and code
/* --------------------------------------------------------------------
 * -----------                   SETUP                     ------------
 * -------------------------------------------------------------------- */
void setup() {
  //open serial connection
  Serial.begin(9600);  
  //set the Robot class up with the pin info for each motor
  robo.setLeft(l_EnPin, l_hPin1, l_hPin2);
  robo.setRight(r_EnPin, r_hPin1, r_hPin2);  
  //pinMode(pingPinR)  
  //print instructions to user - this program requires Serial ON
  Serial.println("- - - Wall Following Robot - - -");
  Serial.println("   Enter 0 for Wallfollower");
  Serial.println("   Enter s,S to stop all movement");  
  //make sure the robot is stopped
  robo.stop();
  robo.setSpeed(speed); //set speed for turning/driving
  Serial.println(headers);
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
    case 1:
      avoidance(); //not implemented
      break;
    case 2:
      turn(); //not implemented
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
  } else if (ch == '1')  { //unused at this time
    mode = 1;}
     else if (ch == '2')  { //unused at this time, for testing only.
    robo.drive();           //tells the robot to just drive straight
  } else {
    Serial.print(ch);
    Serial.println(" : unrecognized command");
  }
 }
 
/* --------------- ping ---------------
 * gets the value from the Ping sensor and returns a distance in cm
 */
int ping(int pingPin) {
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, cm;
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  // convert the time into a distance
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  cm = duration / 29 / 2;
  return cm ;
 }
 
/* --------------- IRdistance ---------------
 * gets the value from the Sharp IR sensor and returns a distance in cm
 * calculated by linear approximation.
 */
float IRdistance(int sensorPin)
{
  // R = 1/(0.00004*V -0.0037) - 0.5
  // discovered by experimentation
  int val = analogRead(sensorPin);
  int mV = (val * referenceMv) / 1023;
  float dist = -0.5;
  dist += 1/(0.04*val/1000 - 0.0037);
  return dist
}

/* --------------- wallfollower ---------------
 * main routine for following a wall, to the right of the robot at a set distance
 * robot should start at a point between 5cm and 1.5m from the wall to work well.
 */
void wallfollower(){  
  const int ComputeMax = 350;    //the maximum allowed/expected error correction value: +-
                                 //lowering this value makes the robot more responsive to error output
  const int DiffMax = 100;       //the maximum wheel differential for turning, see robo.drive_dif()
  const float maxAngle = 30;     //maximum approach angle permitted
  static int frontDistance = 0;  //input variable for front sensor distance (cm)
  static int sideDistance = 0;   //input variable for side sensor distance (cm)
  static int IR = 0;             //input variable for side mounted IR distance sensor
  static float location[2];      //{distance, approach_angle} - calculated location based on sensor readings
  int differential;              //wheel differential ratio, straight = 0 = 1:1, leftturn = -1 = 100:DiffMax
  //get the readings from the ping sensors
  
  frontDistance = 3000;                 //option 1, set the distance to a fixed value
  //frontDistance = ping(pingPinF);     //option 2, ping sensor
  //frontDistance = IRdistance(irPinF); //option 3, IR sensor
  
  //sideDistance = 3000;                //option 1, set the distance to a fixed value
  sideDistance = ping(pingPinR);        //option 2, ping sensor
  //sideDistance = IRdistance(irPinR);  //option 3, IR sensor
  
  //figure the actual distance and approach angle based on the returned sensor values
  estimateDistance(sideDistance, frontDistance, location);
  //logging statements, formatted as tab-separated-data for import to excel
  if(LOG){ Serial.print(""); Serial.print(frontDistance); }
  if(LOG){ Serial.print(" \t"); Serial.print(sideDistance); }
  if(LOG){ Serial.print(" \t"); Serial.print(location[0]); }
  if(LOG){ Serial.print(" \t"); Serial.print(location[1]); }
  //
  if (location[1] > maxAngle)  {
    //if we're approaching too fast, or there's an upcoming inside corner
    //turn back or go straight - we won't even do the error calculations
    //as we don't want to turn more to the right (or spend time on all the float math).
    differential = map( location[1], 30, 90, -10, -DiffMax);
  } else { //if the angle is less than the max approach
    //calculate the PID error correction
    differential = compute(location[0]);
    if (differential > 0){  //if PIR output is telling us to increase distance
      differential = min(ComputeMax, differential); //limit the differntial used to a certain point
    } else {                //PIR is telling us to reduce distance
      differential = max(-ComputeMax, differential); 
    }
    //map the differential recieved to the differential required for the function, 
    //drive_dif() is limited to -100 to 100 for wheel speeds,
    // we are limiting this further to prevent the robot from pivoting on a single wheel.
    differential = map(differential, -ComputeMax, ComputeMax, DiffMax, -DiffMax); 
  }
  //DO EET!
  robo.drive_dif(differential);
  //data logging...
  if(LOG){ Serial.print("\t");Serial.println(differential); }
}
  
int compute(float distance){
  const int setPoint = 40;      //goal distance from the wall in cm
  const float Kp = 5;          //P coefficient
  const float Ki = .0001;      //I coefficient
  const float Kd = 0.1;          //D coefficient
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
  if (front > 310) { 
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
  //
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

