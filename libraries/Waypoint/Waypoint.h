/*
  Waypoint.h - Library for GMU ECE450 Stingray Robot control
  Created by David R. Wernli, June 18,2014
*/
#ifndef Waypoint_h
#define Waypoint_h
#include "Arduino.h"

class Waypoint
{
  public:
    Waypoint(float inLat = 999999999, float inLon = 999999999);
    float lat;
    float lon;
  private:  
    //none  
};

#endif
