#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>
#include "roadMap.h"

/**************************
 * Initializes Vehicle object
 **************************/

RoadMap::RoadMap(){}
RoadMap::RoadMap(vector<double> x_waypoints,vector<double> y_waypoints,vector<double> s_waypoints){
    this->rough_x = x_waypoints;
    this->rough_y = y_waypoints;
    this->rough_s = s_waypoints;
}

/**************************
 * Initialize the spline objects with the waypoints loaded 
 **************************/
void RoadMap::InitSpline(){

    this->spline_x.set_points(this->rough_s, this->rough_x);
    this->spline_y.set_points(this->rough_s, this->rough_y);  

}

/**************************
 * Compute a set of refined waypoints starting from the splines computed from the original waypoints 
 **************************/
void RoadMap::RefineSpline(){

    const int samples = int(this->rough_s[this->rough_s.size()-1]);
    this->fined_x.reserve(samples);
    this->fined_y.reserve(samples);
    this->fined_s.reserve(samples);
    for (int i = 0; i < samples; i++) {
        this->fined_x.push_back(this->spline_x(i));
        this->fined_y.push_back(this->spline_y(i));
        this->fined_s.push_back(i);
    }  
}


RoadMap::~RoadMap() {}