#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>
#include "vehicle.h"

/**
 * Initializes Vehicle object
 */
Vehicle::Vehicle() {}

Vehicle::Vehicle(int id, double x, double y, double s, double d) {

    this->id = id;
    this->last_x = this->last_y = this->last_s = this->last_d = 0;
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
}

void Vehicle::UpdateVehicleStatus(double x, double y, double s, double d){
    this->last_x = this->x;
    this->last_y = this->y;
    this->last_s = this->s;
    this->last_d = this->d;
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
}

Vehicle::~Vehicle() {}

void Vehicle::ComputeCurrentLane(double laneWidth){
    this->current_lane = ceil(this->d/laneWidth);
}
