/*
  Robot.h - Library for GMU ECE450 Stingray Robot control
  Created by David R. Wernli, June 18,2014
*/
#ifndef Robot_h
#define Robot_h
#include "Arduino.h"


static int _Speed;

class Robot
{
  public:
    Robot(int serialStart);
    //setup functions
    void setLeft(int enPin, int hPin1, int hPin2);
    void setRight(int enPin, int hPin1, int hPin2);
    void swapWheels();
    void swapDirectionPins();
    void swapDirectionPins(char wheel[]);
    void setWheelRadius(int radius);
    //drive functions
    void setSpeed(int speed);
    void setDirection(int dir);
    void setDirection(int right, int left);
    void driveForward(int speed=-1); //speed is optional
    void driveReverse(int speed=-1); //speed is optional
    void drive(int speed=-1, int dir=1);
    void drive_dif(int diff, int speed=-1, int MAX_DIFF = 50); //speed is optional
    void stop();
    void coast();
    int  pivot(int angle, int offset = 100);
    void pivot(int dir, int speed, int type);
    int  turn_ang(int angle, int speed=-1);
    void turn(int dir);
  private:
    void _go(int left, int right); //both arguments are optional - default to prior speed
    void _pivotLeft(int dir=1, int speed=-1);
    void _pivotRight(int dir=1, int speed=-1);
    void _pivotCenter(int dir=1, int speed=-1);
    int  _abs(int value);
    //private variables
    int _mLpinEN;  // H-Bridge enable pin, Left motor
	int _mLpin1;   // H-Bridge input pins
	int _mLpin2;
	int _mRpinEN;  // H-Bridge enable pin, Right motor
	int _mRpin1;   // H-Bridge input pins
	int _mRpin2;
	int _alive;    
};

#endif
