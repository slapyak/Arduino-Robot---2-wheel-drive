/* *******************************************************************************
* David Wernli & Hollis Harrell
* ECE 450 - LAB #8, Subsumption - sketch a
* ***************************************************************************** */
#include <Robot.h>
#include <Waypoint.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#define LOG 0 
#define DB  0 

/*--- function declarations ---*/
void cruise();
void lightSeeker(int reading);
void avoid();
void serialEvent();
void serialCommand(char ch);

/*--- sensor pins ---*/
//const int irPinF = 14;    //pin reserved for Sharp IR input (analog)
//const int irPinR = 15;    //front IR sensor  
//const int pingPinR = 52;    //pin reserved for ping sensor input (digital)
//const int pingPinF = 53;
//const int cdsPin = 0;     //pin reserved for photoresistor input (analog)

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
int mode = 0;		//holds the arbitration decision 
int program = 0;	//preprogrammed navigation function
int speed = 75;       	//speed in PWM format [0 - 255]

const Waypoint WPT1(38.828528, -77.304768);	
//defined by lab manual - WPT1:  38° 49’ 42.62” N, 77° 18’ 17.34” W
const Waypoint WPT2(38.828768, -77.304736);	
//defined by lab manual - WPT2:  38° 49’ 42.41” N, 77° 18’ 15.77” W
const Waypoint WPT3(38.828732, -77.304472);	
//defined by lab manual - WPT3:  38° 49’ 43.56” N, 77° 18’ 17.12” W
const Waypoint WPT4(38.828467, -77.304611);	
//defined by lab manual - WPT4:  38° 49’ 42.48” N, 77° 18’ 16.60” W
const Waypoint WPT5(38.828728, -77.304539);	
//defined by lab manual - WPT5:  38° 49’ 43.42” N, 77° 18’ 16.34” W

const Waypoint WPT6(38.824616, -77.433262);	//testing waypoint Betsy/Red River
const Waypoint WPT7(38.824759, -77.433075);	//testing waypoint Red River 1
const Waypoint WPT8(38.825115, -77.433328);	//testing waypoint Red River/Betsy
const Waypoint WPT9(38.824936, -77.433387);	//testing waypoint Betsy Ross

/*--- intialize ---*/
Robot robo;    //start the Robot, with Serial debugging ON
               //refer to Robot class definition for capabilities and code

/* --------------------------------------------------------------------
 * -----------                   SETUP                     ------------
 * -------------------------------------------------------------------- */
void setup() {
    robo.start();
	//open serial connection
	//Serial.begin(9600);
	//set the Robot class up with the pin info for each motor
	robo.setLeft( l_EnPin, l_hPin1, l_hPin2);
	robo.setRight(r_EnPin, r_hPin1, r_hPin2);
	Serial.println("- - - GPS Navigating Robot - - -");
  	Serial.println("   Enter N to change Navigation Program 1");
	Serial.println("   Enter s,S to stop all movement");  
	//make sure the robot is stopped
	robo.stop();
	robo.setSpeed(speed); //set speed for turning/driving
	//if (DB) Serial.println(headers);
	delay(2000);	//give some time to put the robot down after reset
 }

/* --------------------------------------------------------------------
 * -----------                   LOOP                      ------------
 * -------------------------------------------------------------------- */
void loop(){
	//check for something in front of bot, collision avoidance
	//float distance = robo.IRdistance(irPinF);
	//get front sensor distance & make sure we are not about to ram something
	//robo.IRdistance(irPinF) 
	//if (distance < 30) {    //12 inches ~= 30cm
		//if(DB) { Serial.print("\tfront: \t"); Serial.println(distance); }
		//avoid();
	//}
	switch(mode) {
		case 0:			//stop mode, for serial control only
			//nothing to do...
			break;
		case -1:		//avoidance! mode - back off from detected object.
            //avoid();	//handled prior to switch statement to avoid changing modes
            //no break - we want to continue after execution
		case 1:			//case 1 - do the Navigation! 
			mode += executeNavigation();
			//executeNav will return a 1 once navigation program is completed.
			break;
		case 2:			//case 2 
			done();
			break;
		default:
			Serial.println("You really shouldn't be here - invalid mode selected");
	}
}

/*void avoid(){	//obstacle avoidance - back up and rotate away
	//back away from an object detected
	robo.stop();
        //int reading = robo.IRdistance(irPinF);
	while(reading < 45) {  //back up about 6"
          robo.driveReverse();
          reading = robo.IRdistance(irPinF);
        }
	//then pivot 45 degrees
	robo.pivot_ang(45);
    robo.setSpeed(speed);
 }*/

void done(){	//let the user know the program is completed.
	if(DB) {Serial.print("Program "); Serial.print(program); Serial.println(" completed."); }
	robo.stop();
	mode = 0;
 }

/* ---------------- Navigation ---------------
 * Function executeNavigation() will run the specified pre-programmed nav function
 * each nav# function is pre-programmed per the lab manual direction
 */
int executeNavigation(){
	int status=-1;	//holds the status of the selected nav function, 1 means completed.
	//if(DB){ Serial.print("Program "); Serial.print(program); Serial.println(" running."); }
		//initialized to -1 primarily for debugging purposes.
	switch (program){
		case 1:
			status = nav1();	//function returns 1 when completed
			return status;
			break;
		case 2:
			status = nav2();	//function returns 1 when completed
			return status;
			break;
		case 3:
			status = nav3();	//function returns 1 when completed
			return status;
			break;
		case 4:
			status = navTest();	//function returns 1 when completed
			return status;
			break;
                case 5:
			navDiagnostic();	//function returns 1 when completed
			return 0;
			break;
		default:
			Serial.print("Invalid navigation program selected : "); Serial.println(program);
	}
 }
int nav1(){
 /*First Run:
	i.	Starting at WPT1, move from WPT1 to WPT2.
	ii.	Once at WPT2, turn 180 degrees and return to WPT1.
	iii.	Do a 360 degree turn and stop.
 */
	//returns 1 when completed, 0 otherwise.
	static int step = 0;	//internal variable to keep track of where we are in the program
	if(DB){ Serial.print("Program 1, step "); Serial.print(step); Serial.println(" running."); }
	//execute the step we are on	
	switch (step){
		case 0:	//starting at waypoint 1 (manually placed)
			step = 1;
			return 0;	//not done yet
			break;
		case 1:	//go to waypoint #2
			step += robo.travel_to(WPT2); //returns 1 when completed
                        //robo.pivot_ang(180);
			return 0;	//not done yet
			break;
		case 2:	//spin in place at waypoint 2
                        //Serial.println();
			robo.pivot_ang(90);
                        robo.pivot_ang(90);
                        //robo.pivot_ang(90);
                        //robo.pivot_ang(90);
                        step += 1;
                        return 0;
                 case 3:
                     step += robo.travel_to(WPT1);
                     return 0;
                     break;
                 case 4:
                     robo.pivot_ang(360);
                     
                     
		default:
			step = 0;	//reset when done
			return 1;	//job's done
	}
 }
int nav2(){
 /*Second Run:
	i.	Starting at WPT3, move from WPT3 to WPT1.
	ii.	Once at WPT1, move to WPT4.
	iii.	Once at WPT4, move to WPT5.
	iv.	Once at WPT5, do two 360 degree turns to signal that you’ve completed the course.
 */
	//returns 1 when completed, 0 otherwise.
	static int step = 0;	//internal variable to keep track of where we are in the program
	if(DB){ Serial.print("Program 2, step "); Serial.print(step); Serial.println(" running."); }
	//execute the step we are on	
	switch (step){
		case 0:	//starting at waypoint 3 (manually placed)
			step = 1;
			return 0;	//not done yet
			break;
		case 1:	//go to waypoint #1
			step += robo.travel_to(WPT1); //returns 1 when completed
			return 0;	//not done yet
			break;
		case 2:	//go to waypoint #4
			step += robo.travel_to(WPT4); //returns 1 when completed
			return 0;	//not done yet
			break;
		case 3:	//go to waypoint #5
			step += robo.travel_to(WPT5); //returns 1 when completed
			return 0;	//not done yet
			break;
		case 4:	//spin in place at waypoint 5
			robo.pivot(360);
			robo.pivot(360);
		default:
			step = 0;	//reset when done
			return 1;	//job's done
	}
 }
int nav3(){
 /*Third Run:
	i.	Place an obstacle between WPT1 and WPT2.
	ii.	Starting at WPT1, move from WPT1 to WPT2.
	iii.    Detect and successfully avoid the obstacle between the two 
		waypoints and continue to navigate to WPT2 after passing the obstacle;
	iv.	Once at WPT2, do a 360 degree turn and stop
 */
	//returns 1 when completed, 0 otherwise.
	static int step = 0;	//internal variable to keep track of where we are in the program
	if(DB){ Serial.print("Program 3, step "); Serial.print(step); Serial.println(" running."); }
	//execute the step we are on	
	switch (step){
		case 0:	//starting at waypoint 1 (manually placed)
			step = 1;
			return 0;	//not done yet
			break;
		case 1:	//go to waypoint #2
			step += robo.travel_to(WPT2); //returns 1 when completed
			return 0;	//not done yet
			break;
		case 2:	//spin in place at waypoint 2
			robo.pivot(360);
		default:
			step = 0;	//reset when done
			return 1;	//job's done
	}
 }
int navDiagnostic(){
  robo.GPStest();
}
int navTest(){
 /*Test Run
	Drive around in the road...
 */
	//returns 1 when completed, 0 otherwise.
	static int step = 0;	//internal variable to keep track of where we are in the program
	if(DB){ Serial.print("Test program, step "); Serial.print(step); Serial.println(" running."); }
	//execute the step we are on	
	switch (step){
		case 0:	//starting at waypoint 1 (manually placed)
			step = 1;
			return 0;	//not done yet
			break;
		case 1:	//go to waypoint #2
			step += robo.travel_to(WPT6); //returns 1 when completed
			return 0;	//not done yet
			break;
		case 2:	//go to waypoint #2
			step += robo.travel_to(WPT9); //returns 1 when completed
			return 0;	//not done yet
			break;
		case 3:	//go to waypoint #2
			step += robo.travel_to(WPT6); //returns 1 when completed
			return 0;	//not done yet
			break;
		case 4:	//spin in place at waypoint 
			robo.pivot(360);
		default:
			step = 0;	//reset when done
			return 1;	//job's done
	}
 }

/* --------------------------------------------------------------------
 * -----------               INTERACTION                   ------------
 * ------------------------------------------------------------------- */

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
    mode = 0;
    Serial.println("STOP COMMAND RECIEVED");
  } else if (ch == 'n' || ch == 'N')  { //Change Navigation Mode
    Serial.println("Go Forward! Move Ahead! It's not too late!");
    program = Serial.parseInt();	//get the integer from serial
    mode = 1;						//set mode to operate nav program
    Serial.print("Navigation Program : ");  Serial.println(program);
  } else if (ch == '+')  { 
  	//Serial speed update interface
  	//enter as one command "+80"
    speed = Serial.parseInt();
    robo.setSpeed(speed);
    Serial.print("SPEED updated to : ");  Serial.println(speed);
  } else {
    Serial.print(ch);
    Serial.println(" : unrecognized command");
  }
 }
