#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>
#include "vehicle.h"
#include "mainVehicle.h"
#include "otherVehicle.h"

/*********************************
* Initializes Main Vehicle object
*********************************/
MainVehicle::MainVehicle() {}

MainVehicle::MainVehicle(int id, double x, double y, double s, double d, double car_yaw, double car_speed) 
    :Vehicle(id, x, y, s, d){

    this->last_car_yaw = this->last_car_speed = 0;
    this->car_yaw = car_yaw;
    this->car_speed = car_speed;
}
/*
void MainVehicle::UpdateYawAndSpeed(double car_yaw, double car_speed){
    this->last_car_yaw = this->car_yaw;
    this->last_car_speed = this->car_speed;
    this->car_yaw = car_yaw;
    this->car_speed = car_speed;
}
*/
/*********************************
* Update all the vehicle parameters. Respect to the base class, this function also update the yaw and speed of the vehicle
*********************************/
void MainVehicle::UpdateMainVehicleStatus(double x, double y, double s, double d, double car_yaw, double car_speed)
{
    UpdateVehicleStatus(x, y, s, d);
    this->last_car_yaw = this->car_yaw;
    this->last_car_speed = this->car_speed;
    this->car_yaw = car_yaw;
    this->car_speed = car_speed;
}

/*********************************
* Update the set of the close vehicle for the current main vehicle, starting from the sensor_fusion data.
*********************************/
void MainVehicle::UpdateSensorFusion(vector<vector<double>> sensor_data){

    vector<int> sensedId;
	//exectute the update only id the data from the sensor_fusion is not empty
    if(sensor_data.size()>0){
		
		// Update (or add the first time) all the other vehicle with data from sensor_fusion
        for(vector<double> data : sensor_data){
            int id = (int)data[0];
            sensedId.push_back(id);

            if(this->close_vehicles.find(id) != this->close_vehicles.end())
            {
                this->close_vehicles[id].UpdateOtherVehicleStatus(data[1],data[2],data[5],data[6],data[3],data[4]);
            }
            else{
                this->close_vehicles[id] = OtherVehicle(id,data[1],data[2],data[5],data[6],data[3],data[4]);
            }
        }
		
		// Remove vehicle from the close vehicle list if not sensed anymore.
        if(this->close_vehicles.size() > 0){
            vector<int> ids_to_be_removed;
            for (std::map<int,OtherVehicle>::iterator it=this->close_vehicles.begin(); it!=this->close_vehicles.end(); ++it){
                int key = it->first;
                std::vector<int>::iterator it2 = find (sensedId.begin(), sensedId.end(), key);
                if (it2 == sensedId.end())
                    ids_to_be_removed.push_back(key);
            }
            for(int id :ids_to_be_removed)
                this->close_vehicles.erase(id);
        }
    }
}


MainVehicle::~MainVehicle() {}