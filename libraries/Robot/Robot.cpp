/*
  Robot.cpp - Library for GMU ECE450 Stingray Robot control
  Created by David R. Wernli, June 18,2014
  TinyGPS and Software Serial libraries are required as well.
*/

#include "Arduino.h"
#include "Robot.h"
#include <SoftwareSerial.h>
#include <TinyGPS.h>
  #define WHEELDIA 4.875    //in inches
  #define WIDTH 10.9      //in inches - width of robot
  #define _RPM (300*9.6/12) //max RPM with current voltage conditions
  #define _ipr (WHEELDIA*31.41592) //inches per 10 rotations of the wheel
  #define _robotCirc (2*WIDTH*3.14159) //circumf pivoting on a wheel
  #define DB 0 //debug variable to control serial print statements for all functions
  #define GPS 1 //set to 1 if GPS is to be enabled. Requires software 
  				//serial connection to GPS module on pins 3 & 4 of arduinoMega
TinyGPS gps;
SoftwareSerial ss(4, 3); 

/* ***************************  INITIALIZE  ******************************* */
/* Robot function must be called to start the robot up, called as
 * Robot yourLocalName(1);
 * It has tended to be more reliable to start serial in the calling sketch
 * and use 0 for serialStart parameter
 */
Robot::Robot(int serialStart, int gpsStart){
	_alive = 1;  				//global confirmation of initialization
	_Speed = 0;					//preset speed to stop any inadvertant movement
	if (gpsStart == 1) { 		 

	}
	Serial.println("Robot Initialized.");
 }
void Robot::start(){
	Serial.begin(9600);
	if (GPS) { ss.begin(4800); }
}
/* ************************  SETUP FUNCTIONS  ***************************** */
/* Fuctions setLeft, setRight used to configure the h-bridge. */
void Robot::setLeft(int enPin, int hPin1, int hPin2){	
 //initialize the pins for control of the left wheel H-bridge
	pinMode(hPin1, OUTPUT);
	pinMode(hPin2, OUTPUT);
	pinMode(enPin, OUTPUT);
	_mLpinEN = enPin;  // H-Bridge enable pin, Left motor
	_mLpin1 = hPin1;   // H-Bridge input pins
	_mLpin2 = hPin2;
	stop(); //prevent accidental movement
 }
void Robot::setRight(int enPin, int hPin1, int hPin2)
 {	//initialize the pins for control of the right wheel H-bridge
	pinMode(hPin1, OUTPUT);
	pinMode(hPin2, OUTPUT);
	pinMode(enPin, OUTPUT);
	_mRpinEN = enPin;  // H-Bridge enable pin, Right motor
	_mRpin1 = hPin1;   // H-Bridge input pins
	_mRpin2 = hPin2;
	stop(); //prevent accidental movement
 }
/* Software fixes to allow correction of hardware wiring issues on h-bridge */
//Swap left and right wheels
void Robot::swapWheels(){	
	//store left pins in temp variables
	int temp1 = _mLpinEN;  // H-Bridge enable pin, Left motor
	int temp2 = _mLpin1;   // H-Bridge input pins
	int temp3 = _mLpin2;
	//swap right pins to left pins
	_mLpinEN  = _mRpinEN;  // H-Bridge enable pin, Left motor
	_mLpin1   = _mRpin1;   // H-Bridge input pins
	_mLpin2   = _mRpin2;
	//store former left pins into right pin variables
	_mRpinEN  = _mLpinEN;  // H-Bridge enable pin, Right motor
	_mRpin1   = _mLpin1;   // H-Bridge input pins
	_mRpin2   = _mLpin2;
	//the pins are all initialized alredy - so no further work is req'd
 }
//swap direction pins on selected wheel; corrects for fwd/reverse confusion
void Robot::swapDirectionPins(char wheel[]){
	int temp;
	if (wheel == "left" || wheel == "l"){
		temp = _mLpin1;
		_mLpin1 = _mLpin2;
		_mLpin2 = temp;
	} else if (wheel == "right" || wheel == "r"){
		temp = _mRpin1;
		_mRpin1 = _mRpin2;
		_mRpin2 = temp;
	}
 }
//swap direction pins on both wheels
void Robot::swapDirectionPins(){
	swapDirectionPins("l");
	swapDirectionPins("r");
 }


/* ************************  SENSOR FUNCTIONS  ************************** */

/* --------------- ping ---------------
 * gets the value from the Ping sensor and returns a distance in cm
 */
int Robot::ping(int pingPin) {
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
float Robot::IRdistance(int sensorPin) {
  const long referenceMv = 5000; // long int to prevent overflow when multiplied
  // R = 1/(0.00004*V -0.0037) - 0.5
  // discovered by experimentation
  int val = analogRead(sensorPin);
  int mV = (val * referenceMv) / 1023;
  float dist = -0.5;
  dist += 1/(0.04*mV/1000 - 0.0037);
  return dist;
 }

/* ************************  MOVEMENT FUNCTIONS  ************************** */

/* ------------------------   PUBLIC FUNCTIONS   -------------------------- */ 

void Robot::setSpeed(int speed){
	if (speed != -1 && speed != _Speed) {
		if(DB){ Serial.print("Speed was ");	Serial.print(_Speed); Serial.print(", set to ");  Serial.println(speed);}
		_Speed = speed;
		
	}
 }
/* FUNCTIONS driveForward & driveReverse & drive
 * Tells wheels to operate the robot in specified direction
 * speed is given as int - valid range is 0-255
 * output as PWM on enable pin for drive motors
 */
void Robot::driveForward(int speed){
	setSpeed(speed);
	setDirection(1,1);
	_go(_Speed, _Speed);
 }
void Robot::driveReverse(int speed){
	setSpeed(speed);
	setDirection(-1,-1);
	_go(_Speed, _Speed);
 }
//Simplified function allowing variable to control direction, or coast to stop
void Robot::drive(int speed, int dir){
	if(DB) { 
		Serial.print(".drive at "); Serial.print(speed);
		Serial.print(", stored spd: "); Serial.println(_Speed); } 
	setSpeed(speed);
	if (dir == 1)	{
		driveForward(_Speed);
	} else if (dir == -1)	{
		driveReverse(_Speed);
	} else {
		coast(); //coast to a stop if direction is 0
	}
 }
/* FUNCTION - Differential Drive(diff, speed = -1)
 * allows the robot to turn in controllable radius turns in fwd direction
 * primarily for control via PID gain controls
 * -100 is max left turn, ccw rotation (100 used in place of '1'...)
 * +100 is max right turn, cw rotation (...to eliminate need for float math)
 * speed is typical 0-255 PWM speed control value
 */
void Robot::drive_dif(int diff, int speed, int MAX_DIFF){
	//the maximum differential we want to use MAX_DIFF
	//this is the most that will be added or subtracted to the 
	//base _Speed variable 
	setSpeed(speed);
	//set the differential values (speed multiplication factors)
	//when we go negative, the right wheel will be at full turn
	diff = map(diff, -100, 100, -MAX_DIFF, MAX_DIFF);
	int speed_l = (_Speed + diff);	//left turns are negative, slow leftwheel
	//limit the wheel speed to within the PWM values
	speed_l = max(speed_l, 0);
	speed_l = min(speed_l, 255);
	int speed_r = (_Speed - diff);	//left turns are negative, speed rightwheel
	//limit the wheel speed to within the PWM values
	speed_l = max(speed_l, 0);
	speed_l = min(speed_l, 255);
	//notify user 
	if(DB){ 
		Serial.print(" Differential Drive: ");
		Serial.print(speed_l);
		Serial.print(", ");
		Serial.println(speed_r); }
	//send the command
	setDirection(1);
	_go(speed_l, speed_r);
 }
/* FUNCTION - Robot will pivot as directed
 * speed is max wheel speed
 * dir is direction, 1=cw, -1=ccw
 * type is 0=center, 1=on inside wheel, 2=on outside wheel(reverse pivot)
 * offset adds startup time for the motor - to correct for error
 */
int Robot::pivot(int angle, int offset){
 if (angle == 0){
  		return 1; }
 	//calculate time to turn
	 //calculate current RPM
	int speed = _RPM * 0.6;	
	int rpm = speed;
		if(DB){ Serial.print(" rpms: "); Serial.print(rpm); 
				Serial.print(" : "); Serial.println(_RPM); }
    //calculate degrees per 100 inches
	int dpi = 7200/_robotCirc; //360 / [1/2 * (2pi * WIDTH)] - times 100
	Serial.print(" dpi = "); Serial.println(dpi);
	 //calculate milliseconds per degree
	int mspd = 600000/rpm; 	//60k ms per minute, ms per 10 revolutions
		if(DB){ Serial.print(" 600000*10/rpm (ms/inch by wheel): "); Serial.println(mspd); }
	mspd = mspd/_ipr*10* _abs(angle)/dpi; //ms per inch, each offset by 10
		if(DB){ Serial.print(" ms per inch: "); Serial.println(mspd); }
	mspd = mspd + offset;		//add motor start time
		if(DB){ Serial.print(" ^ /dpi: "); Serial.println(mspd); }
	//calculate turn time required 
		// angle * ms per 100/100 deg +1 for rounding error correction
	//figure direction to rotate
	int dir;
	if (angle>0){
		dir = 1; 
	} else {
		dir = -1;
	}
	//do the turn
	//stop();
	if(DB){ Serial.print("Pivoting for (ms): "); Serial.println(mspd); }
	_pivotCenter(dir, speed);
	delay(mspd); //adding 2 allows time for motor to start, 1ms pulse doesn't move at all
	_pivotCenter(-dir, speed);
	stop();
	return 1;
 }
/* pivot 
   dir, -1=ccw, 1=cw
   type, 0=center, 1=left wheel stop, 2 = right wheel stop */
void Robot::pivot(int dir, int speed, int type) {
	if(type = 0) {
		_pivotCenter(dir, speed);
	} else if (type = 1) {
		_pivotLeft(dir, speed);
	} else if (type = 2) {
		_pivotRight(dir, speed);
	}
 }
/* turn functions */
int Robot::turn_ang(int angle, int speed){
	setSpeed(speed);
	if (angle == 0){
		return 1; }
	int diff;
	//calculate time to turn
	//calculate current RPM
	int rpm = speed/255*_RPM;
	 //calculate degrees per 100 inches
	int dpi = 36000/_robotCirc; //360 / [1/2 * (2pi * WIDTH)] - times 100
	 //calculate milliseconds per degree
	int mspd = 600000/rpm; 	//60k ms per minute, ms per 10 revolutions
	mspd = 10000*mspd/_ipr; //ms per inch, each offset by 10
	mspd = mspd/dpi;		//ms per 100 degree
	//calculate turn time required 
		// degrees * ms per 100/100 deg +1 for rounding error correction
	int turnTime = angle*mspd/100 + 1;
	//figure direction to rotate
	if (angle>=0){
		diff = -1; 
	} else {
		diff = 1;
	}
	//do the turn
	if(DB){ Serial.print("Turning for (ms): "); Serial.println(mspd); }
	drive_dif(diff);
	delay(mspd); 
	coast();
 return 1;
 }
//turns on left or right wheel moving forward only
void Robot::turn(int dir){
	if (dir == 1) {
		_pivotLeft(1,_Speed);
	} else if (dir == -1)	{
		_pivotRight(1,_Speed);
	}
 }
/* FUNCTION - stops movement by halting PWM output on enable pins 
 * as well as shorting the motors to ground
 * this causes an abrupt stop
 */
void Robot::stop(){
	if(DB) Serial.println("Stopping");
	//setSpeed(0);
	setDirection(0);
 }
/* FUNCTION
 * Robot will roll to a stop, setting enable low, leaving the H-bridge connected
 * allows the botor to continue to turn without power
 * Stopping, as above - shorts the motor and the Robot stops immediately
 */
void Robot::coast(){
	if(DB) Serial.println("Coasting");
	analogWrite(_mLpinEN, 0);
 	analogWrite(_mRpinEN, 0);
 }

/* -----------------------   PRIVATE FUNCTIONS   -------------------------- */ 

/* PRIVATE function to interpret all drive commands
   defaults to current speed if none expressly specified */
void Robot::_go(int left=_Speed, int right=_Speed){
	analogWrite(_mLpinEN, left); //drive PWM pin to move wheel 
	analogWrite(_mRpinEN, right); //drive PWM pin to move wheel 
 }
/* PRIVATE FUNCTION - setDirection
 * Allows the commands to set the direction of each wheel
 * Reports status via Serial.print
 * dir is -1 for reverse, 0 for stop, 1 for forward
 */
void Robot::setDirection(int dir){
	setDirection(dir, dir);
 }
void Robot::setDirection(int right, int left){
	//Serial.print("Setting Direction:");Serial.print(left);Serial.print(", ");Serial.println(right);
	static int _dirLeft;
	static int _dirRight;
	if (left != _dirLeft){	//left wheel
		if (left == -1)	//if commanded to go in reverse
		{
			if(DB) Serial.println("left wheel reverse"); 
			digitalWrite(_mLpin1,LOW);	//set the H-bridge
 			digitalWrite(_mLpin2,HIGH);
		} else if (left == 1) {	//if commanded to go fwd
			if(DB) Serial.println("left wheel forward"); 
			digitalWrite(_mLpin1,HIGH);
 			digitalWrite(_mLpin2,LOW);
		} else {	//if command is not recognized or is '0'
			if(DB) Serial.println("left wheel hard stop");
			digitalWrite(_mLpin1,LOW);	//open the H-bridge
 			digitalWrite(_mLpin2,LOW);
		}
		_dirLeft = left;
	} else { 
		//Serial.print("  Left already set to ");
		//Serial.println(_dirLeft);
	}
	if (right != _dirRight){	//right wheel
		if (right == -1)	//if commanded to go in reverse
		{
			if(DB) Serial.println("right wheel reverse");
			digitalWrite(_mRpin1,LOW);	//set the H-bridge
 			digitalWrite(_mRpin2,HIGH);
		} else if (right == 1) {	//if commanded to go fwd
			if(DB) Serial.println("right wheel forward");
			digitalWrite(_mRpin1,HIGH);
 			digitalWrite(_mRpin2,LOW);
		} else {	//if command is not recognized or is '0'
			if(DB) Serial.println("right wheel hard stop");
			digitalWrite(_mRpin1,LOW);	//open the H-bridge
 			digitalWrite(_mRpin2,LOW);
		}
		_dirRight = right;
	} else {
		//Serial.print("  Right already set to ");
		//Serial.println(_dirRight);
	}
 }
/* PRIVATE FUNCTIONs - pivot variations 
   internal function for turning as described, 
   Left pivots about left wheel, Right the right, center about the central point */
void Robot::_pivotLeft(int dir, int speed){	
	//pivots around the left wheel
	// speed is int
	// dir is int = [1,-1]
	setSpeed(speed);
	setDirection(0,dir);
 	_go();
 }
void Robot::_pivotRight(int dir, int speed){	
	//pivots around the right wheel
	// speed is int
	// dir is int = [1,-1]
	setSpeed(speed);
	setDirection(dir, 0);
	_go();
 }
void Robot::_pivotCenter(int dir, int speed){	//center pivot only
	// dir is int = [1,-1], cw = 1, ccw=-1
	setSpeed(speed);
	setDirection(-dir, dir);
	_go();
 }

 /* *************************  EXTRA FUNCTIONS  **************************** */
int Robot::_abs(int value){
 	if (value < 0){
 		return -value;
 	} else {
 		return value;
 	}
 }