#ifndef ROAD_MAP_H
#define ROAD_MAP_H

#define SPLINE_SAMPLES 30000

#include "spline.h"

using namespace std;
using namespace tk;

class RoadMap{

public:

    spline spline_x, spline_y, spline_s;

    vector<double> rough_x, rough_y, rough_s;

    vector<double> fined_x, fined_y, fined_s;
    RoadMap();
    RoadMap(vector<double> x_waypoints, vector<double> y_waypoints, vector<double> s_waypoints);
    virtual ~RoadMap();

    void InitSpline();
    void RefineSpline();


};
#endif /* ROAD_MAP_H_ */