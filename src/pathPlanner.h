#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <vector>
#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include "mainVehicle.h"
#include "roadMap.h"

// MAX_SPEED and MIN_SPEED are both in m/s and are defined in order to avoid speed limit overcame
#define MAX_SPEED 20
#define MIN_SPEED 10

#define KL_ACC_COEFF 4.0
#define FORWARD_COLLISION_BUFFER 20.0
#define BACKWARD_COLLISION_BUFFER 10.0

#define MIN_SPEED_VAR -4.0
#define MAX_SPEED_VAR 4.0

#define PATH_HORIZON 2.5
#define PATH_WP_PERIOD 0.02

#define MAX_TRACK_S 6945.554

// Defines the coefficients to be applied in the cost computation functions;
// KL_COST_FACTOR is the coefficient for the keep lane state; 
// CL_COST_FACTOR_F and CL_COST_FACTOR_R are factors for change lane cost computation and refers to front (_F)
// and rear (_R) distances to the cars in that lane
#define KL_COST_FACTOR  1.0
#define CL_COST_FACTOR_F 1.0
#define CL_COST_FACTOR_R 0.5


using namespace std;

struct telemetry 
{
    int    lane;
    double car_s;
    double car_speed;
};

class PathPlanner{
    
    public:

    PathPlanner(double lw, int nol, int trj_len, int prev_trj_len);
    virtual ~PathPlanner();

    MainVehicle car;
    double laneWidth;
    int numberOfLanes; 
    int horizon; 
    int update_timestamp;
    RoadMap road_map;
    vector<double> currentX;
    vector<double> currentY;

    vector<double> currentD;
    vector<double> currentS;
    
    void InitalizeMap(string map_file, double max_s);
    void ComputeNextTrajectory(string s);
    void UpdateMainVehicle(double x, double y, double s, double d, double yaw, double speed, 
                           vector<vector<double>> sensor_fusion);

    /**********************************
    * Methods for best trajectory identification
    **********************************/
    
    Path ComputeStupidTrajectory(vector<double> previous_path_x, vector<double> previous_path_y,
								 double car_s, double car_d, double yaw);
    string ComputeMinCostState(telemetry t);
    double KLStateCost(telemetry t);
    double CLStateCost(telemetry t, int lane);

    Path ComputeKLPath(telemetry t);
    Path ComputeCLLPath(telemetry t);
    Path ComputeCLRPath(telemetry t);

    void DistanceToCarsInLane(int lane, OtherVehicle *f_v, OtherVehicle *b_v,
									   double *f_d, double *b_d);
    void ComputeMinimumJerkPath(Path *p);
    vector<double> ComputeMinimumJerk(vector<double> start, vector<double> end);
    double Lane2D(int lane);
    int D2lane(double d);
    double AdjustDistance2Speed(double start_speed, double rough_front_dist, 
								OtherVehicle forward_vehicle);
    double ComputeCarSpeed(OtherVehicle v);

    double distance(double x1, double y1, double x2, double y2);
    int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
    int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
    vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
    vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

};
#endif /* PATHPLANNER_H_ */