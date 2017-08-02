#ifndef MAIN_VEHICLE_H
#define MAIN_VEHICLE_H

#include <vector>
#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include "vehicle.h"
#include "otherVehicle.h"
#include "path.h"
#include <map>


using namespace std;

/*********************************
* MainVehicle object is a specialization of the vehicle class which add to the common attributes also 
* the car yaw and speed, the vector with all the close cars and the trajectory 
*********************************/
class MainVehicle: public Vehicle{
    
    public:

    double car_yaw, car_speed;
    double last_car_yaw, last_car_speed;

    Path current_path, last_path;

    map<int,OtherVehicle> close_vehicles;
    
    MainVehicle();
    MainVehicle(int id, double x, double y, double s, double d, double car_yaw, double car_speed);
    void UpdateMainVehicleStatus(double x, double y, double s, double d, double car_yaw, double car_speed);
    //void UpdateYawAndSpeed(double car_yaw, double car_speed);
    void UpdateSensorFusion(vector<vector<double>> sensor_data);
    virtual ~MainVehicle();

};
#endif /* MAIN_VEHICLE_H_ */