#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>
#include "roadMap.h"

/**
 * Initializes Vehicle object
 */

RoadMap::RoadMap(){}
RoadMap::RoadMap(vector<double> x_waypoints,vector<double> y_waypoints,vector<double> s_waypoints){
    this->rough_x = x_waypoints;
    this->rough_y = y_waypoints;
    this->rough_s = s_waypoints;
}

void RoadMap::InitSpline(){
    /*
    vector<double> waypoint_t;
    double waypoints_size = x_waypoints.size();
    for (double i=0; i<waypoints_size; i++)
    {
        double t = i / waypoints_size; 
        waypoint_t.push_back(t);
    }
    */

    this->spline_x.set_points(this->rough_s, this->rough_x);
    this->spline_y.set_points(this->rough_s, this->rough_y);  
    /*
    this->spline_x.set_points(waypoint_t, this->rough_x);
    this->spline_y.set_points(waypoint_t, this->rough_y);
    this->spline_s.set_points(waypoint_t, this->rough_s); 
    */   
}

// refine path with spline.
void RoadMap::RefineSpline(){
    /*
    for (size_t i = 0; i < SPLINE_SAMPLES; ++i) {
        this->fined_x.push_back(this->spline_x(i));
        this->fined_y.push_back(this->spline_y(i));
        this->fined_s.push_back(i);
    }
    */
    const int samples = int(this->rough_s[this->rough_s.size()-1]);
    this->fined_x.reserve(samples);
    this->fined_y.reserve(samples);
    this->fined_s.reserve(samples);
    for (int i = 0; i < samples; i++) {
        this->fined_x.push_back(this->spline_x(i));
        this->fined_y.push_back(this->spline_y(i));
        this->fined_s.push_back(i);
    }
    /*
    for (int i = 0; i < SPLINE_SAMPLES; ++i) {
        double t = (double)i / (double)SPLINE_SAMPLES;
        this->fined_x.push_back(this->spline_x(i));
        this->fined_y.push_back(this->spline_y(i));
        this->fined_s.push_back(this->spline_s(i));
    }
    */
}


RoadMap::~RoadMap() {}