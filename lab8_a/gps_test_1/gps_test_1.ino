#include "Robot.h"
#include "TinyGPS.h"
#include "SoftwareSerial.h"

Robot robo;  //intialize Robot object

void setup()
{
  robo.start();	
}

void loop()
{
	//Serial.println("starting loop");
	robo.GPStest();
}
