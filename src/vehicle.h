#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>

using namespace std;

class Vehicle{
    
    public:

    int current_lane;
    int id;

    double x, y, s, d;
    double last_x, last_y, last_s, last_d;

    Vehicle();
    Vehicle(int id, double x, double y, double s, double d);
    virtual ~Vehicle();

    void UpdateVehicleStatus(double x, double y, double s, double d);
    void ComputeCurrentLane(double laneWidth);

};
#endif /* VEHICLE_H_ */