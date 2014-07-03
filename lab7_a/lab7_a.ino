/* *******************************************************************************
* David Wernli & Hollis Harrel
* ECE 450 - LAB #7, Subsumption - sketch a
* ***************************************************************************** */
#include <Robot.h>
#define LOG 0 
#define DB  1

/*--- function declarations ---*/
void cruise();
void lightSeeker(int reading);
void avoid();
void serialEvent();
void serialCommand(char ch);

/*--- sensor pins ---*/
//const int irPinF = 14;    //pin reserved for Sharp IR input (analog)
const int irPinR 	= 15;    //front IR sensor  
const int pingPinR  = 52;  //pin reserved for ping sensor input (digital)
const int pingPinF  = 53;
const int cdsPin 	= 0;     //pin reserved for photoresistor input (analog)

/*--- servo pins ---*/
//const int servPin = 8   //pin reserved for servo output (PWM)

/*--- motor pins ---*/
const int l_EnPin = 7;  //left motor enable pin
const int l_hPin1 = 5;  //left motor hbridge pins
const int l_hPin2 = 4;
const int r_EnPin = 6;  //right motor enable pin (for PWM speed ctrl)
const int r_hPin1 = 3;  //right motor hbridge pins
const int r_hPin2 = 2;

/*--- global variables ---*/
int arbiter = 0;		//holds the arbitration decision 
int speed = 80;       	//speed in PWM format [0 - 255]

/*--- intitialize ---*/
Robot robo;    //start the Robot, with Serial debugging ON
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
	Serial.println("- - - Subsimption Emergent Robot - - -");
  	Serial.println("   Enter 0 for Subsumption Behavior");
	Serial.println("   Enter s,S to stop all movement");  
	//make sure the robot is stopped
	robo.stop();
	robo.setSpeed(speed); //set speed for turning/driving
	if (DB) Serial.println(headers);
	delay(2000);	//give some time to put the robot down after reset
 }

void loop(){
	const int center = 512;		//CdS differntial center point (equal light input)
	const int buffer = 40;		//CdS differntial deadband
	//check for activation conditions for each state
	//checking in order from lowest importance to highest
	//higher import modes will overwrite the arbitration variable
	//and override the action of the robot. 
	//Robot executes one action per 'loop'
	arbiter = 0;	//0 case is the 'cruise function'
	//get Cds divider info, set flag if it's outside of deadband
	int lightdir = lightReading(cdsPin);
	if (lightdir > center+buffer || lightdir < center-buffer) {
		arbiter = 1;	//if we see a brighter spot, set conditional flag
	}
	//get front sensor distance & make sure we are not about to ram something
	//robo.IRdistance(irPinF) 
	if ( < 20) {
		arbiter = 2;	//if we're too close, set conditional flag
	}
	//arbitrate - arbitration is handled by the switch statement
	//the flags are set by the above if statements
	switch(arbitrate) {
		case -1:		//stop mode, for serial control only
			robo.stop();
			break;
		case 0:			//case 0 is the cruise flag
			cruise();
			break;
		case 1:			//case 1 is seeking the bright spot
			lightSeeker(lightdir);
			break;
		case 2:			//case 2 is avoiding a sensed object
			avoid();
			break;
		case default:
			Serial.println("You really shouldn't be here - invalid mode selected");
	}
}

void cruise(){
	//cruise around randomly, looking for light or objects
	robo.driveForward();
 }

void lightSeeker(int reading){
	const int MaxDelta = 400;	//anticipated maximum difference between for any light reading
	const int MaxTurn  = 80;	//maximum differential drive for turning in response to light
	//turn toward the light
	//map the reading value to numbers usable by differential drive function
	reading = map(reading, -MaxDelta, MaxDelta, -MaxTurn, MaxTurn); 
	//ensure the number mapped is within the appropriate range
	reading = min(MaxTurn, reading);
	reading = max(-MaxTurn reading);
	//execute the turn
	robo.drive_dif(reading, -1, 80);
 }

void avoid(){
	//back away from an object detected
	robo.stop()
	robo.driveReverse();
	//at speed = 80/255 * 9.6V/12V * 300RPM = 75 RPM :: 19 inch/second
	//back up for 6 inches, 375ms + startup time of 150ms
	delay(525); 
	//then pivot 45 degrees
	robo.pivot(45);
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
  } else if (ch == '+')  { //user selected wallfollower mode
    speed = Serial.parseInt();
    robo.setSpeed(speed);
    Serial.print("SPEED updated to : ");  Serial.println(speed);
  } else {
    Serial.print(ch);
    Serial.println(" : unrecognized command");
  }
 }
