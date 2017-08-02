#ifndef OTHER_VEHICLE_H
#define OTHER_VEHICLE_H

#include <vector>
#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include "vehicle.h"

using namespace std;

class OtherVehicle: public Vehicle{
    
    public:

    double vx, vy, last_vx, last_vy;

    OtherVehicle();
    OtherVehicle(int id, double x, double y, double s, double d, double vx, double vy);
    void UpdateOtherVehicleStatus(double x, double y, double s, double d, double vx, double vy);
    //void UpdateVxAndVy(double vx, double vy);
    virtual ~OtherVehicle();
};
#endif /* OTHER_VEHICLE_H_ */