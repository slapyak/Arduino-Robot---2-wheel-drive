/*
  Waypoint.cpp - Library for GMU ECE450 Stingray Robot GPS control
  Created by David R. Wernli, July 9,2014
*/

#include "Arduino.h"
#include "Waypoint.h"

Waypoint::Waypoint(float inLat, float inLon){
	lat = inLat;
	lon = inLon;
}