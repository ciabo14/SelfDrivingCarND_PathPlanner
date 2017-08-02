#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>
#include "vehicle.h"
#include "otherVehicle.h"

/**
 * Initializes Vehicle object
 */
OtherVehicle::OtherVehicle() {}

OtherVehicle::OtherVehicle(int id, double x, double y, double s, double d, double vx, double vy) 
    :Vehicle(id, x, y, s, d){

    this->last_vx = .0;
    this->last_vy = .0;
    this->vx = vx;
    this->vy = vy;
}

void OtherVehicle::UpdateOtherVehicleStatus(double x, double y, double s, double d, double vx, double vy){
    UpdateVehicleStatus(x, y, s, d);
    this->last_vx = this->vx;
    this->last_vy = this->vy;
    this->vx = vx;
    this->vy = vy;

}
/*
void OtherVehicle::UpdateVxAndVy(double vx, double vy){
    
    this->last_vx = this->vx;
    this->last_vy = this->vy;

    this->vx = vx;
    this->vy = vy;
}
*/
OtherVehicle::~OtherVehicle() {}