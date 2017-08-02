#include <vector>
#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <map>
#include "json.hpp"
#include "pathPlanner.h"
#include "mainVehicle.h"
#include "roadMap.h"
#include "path.h"
#include <utility>
#include "Eigen-3.3/Eigen/Dense"

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

using json = nlohmann::json;

using namespace std;
using namespace Eigen;

PathPlanner::PathPlanner(double lw, int nol, int horizon, int update_timestamp) 
{

    this->laneWidth = lw;
    this->numberOfLanes = nol;

    this->horizon = horizon;
    this->update_timestamp = update_timestamp;
    car = MainVehicle(-1, .0, .0, .0, .0, .0, .0);
}

PathPlanner::~PathPlanner() {}

void PathPlanner::InitalizeMap(string map_file, double max_s)
{

    cout << "----------INITIALITATION OF THE MAP----------"<<endl;
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	ifstream in_map_(map_file.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	/*************************
	* Initialize the roadMap object with with the waypoints form the csv file
	*************************/
	this->road_map = RoadMap(map_waypoints_x,map_waypoints_y,map_waypoints_s);
	this->road_map.InitSpline();
    this->road_map.RefineSpline();
}

void PathPlanner::UpdateMainVehicle(double x, double y, double s, double d, double yaw, double speed, 
                                    vector<vector<double>> sensor_fusion)
									{
    
    // Update main car's localization Data
    car.UpdateMainVehicleStatus(x, y, s, d, yaw, speed);
    car.ComputeCurrentLane(this->laneWidth);
    //car.UpdateYawAndSpeed(yaw, speed);
    car.UpdateSensorFusion(sensor_fusion);

}

void PathPlanner::ComputeNextTrajectory(string s)
{
    
    auto j = json::parse(s);
    /*******************************
    * Update current car status (x,y,s,d,yaw and speed) and the data coming from sensor fusion
    *******************************/
    auto sensor_fusion = j[1]["sensor_fusion"];
    UpdateMainVehicle(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"],
                      sensor_fusion);

    // Previous path data given to the Planner
    vector<double> previous_path_x = j[1]["previous_path_x"];
    vector<double> previous_path_y = j[1]["previous_path_y"];
    // Previous path's end s and d values 
    double end_path_s = j[1]["end_path_s"];
    double end_path_d = j[1]["end_path_d"];

	Path target_path;
	//target_path = IncludePrevPathInfo(vector<double> previous_path_x, vector<double> previous_path_y)

	if(previous_path_x.size() == 0){
		telemetry t {D2lane(j[1]["d"]),j[1]["s"],j[1]["speed"]};
		target_path = ComputeKLPath(t);
	}
	// Nearing the end of driven path	
	else if (previous_path_x.size() < 125)
	{
		telemetry t {D2lane(this->car.current_path.D_end[0]),this->car.current_path.S_end[0],
					 this->car.current_path.S_end[1]};
		
		
		string action = ComputeMinCostState(t);
		cout << action << endl;
		if (action == "KL")
			target_path = ComputeKLPath(t);
		if (action == "CLL")
			target_path = ComputeCLLPath(t);
		if (action == "CLR")
			target_path = ComputeCLRPath(t);

		cout << target_path.S_start[0] << " - "<< target_path.S_start[1] << endl;
		cout << target_path.S_end[0] << " - "<< target_path.S_end[1] << endl;

		vector<double> path_x = previous_path_x;
		vector<double> path_y = previous_path_y;

		for (int i=0; i<target_path.X.size(); i++)
		{
			path_x.push_back(target_path.X[i]);
			path_y.push_back(target_path.Y[i]);
		}
		target_path.X = path_x;
		target_path.Y = path_y;
	}

	// Just resend the rest of the path back to sim
	else
	{
		target_path = this->car.current_path;
		target_path.X = previous_path_x;
		target_path.Y = previous_path_y;
	}

	//target_path = ComputeBestPath()
	
	/*
	cout<<"computing stupid trj"<<endl;
	car.current_path = ComputeStupidTrajectory(previous_path_x, previous_path_y,
							(double)j[1]["s"], (double)j[1]["d"], (double)j[1]["yaw"]);
	*/
	car.current_path = target_path;
}

/***************************
* Function used to compute the state with min cost between keep Lane, change lane right and change lane left 
***************************/
string PathPlanner::ComputeMinCostState(telemetry t)
{
	double KL_cost  = KLStateCost(t);
	double CLL_cost = 1e9;
	double CLR_cost = 1e9;
	if(t.lane != 1)
        CLL_cost  = CLLStateCost(t);
	if(t.lane != 3)
    	CLR_cost = CLRStateCost(t);
	
    cout << "costs: KL->" << KL_cost << " - CLL->" << CLL_cost << " - CLR->" << CLR_cost << endl;

	if(KL_cost <= CLL_cost && KL_cost <= CLR_cost)
		return "KL";
	else if(CLL_cost <= CLR_cost)
		return "CLL";
	return "CLR";
}

/***************************
* Compute the Keelp Lane
***************************/
double PathPlanner::KLStateCost(telemetry t)
{
	double cost = 100000;

	OtherVehicle forward_vehicle, backword_vehicle;
	double forward_dist, backword_dist;
	DistanceToCarsInLane(t.lane, &forward_vehicle, &backword_vehicle,
						 &forward_dist, &backword_dist);

	cout << "Current Lane Distances: " << forward_dist  << backword_dist << endl;
    if (forward_dist != 0.0)
    {
        cost = LANE_CHANGE_COST_AHEAD / forward_dist;
    }
    return cost;
}
/***************************
* Compute the Change Lane Left cost from current position 
***************************/
double PathPlanner::CLLStateCost(telemetry t)
{	
	double cost = 100000;
    if (t.lane != 1)
    {
		OtherVehicle forward_vehicle, backword_vehicle;
		double forward_dist, backword_dist;
		DistanceToCarsInLane(t.lane-1, &forward_vehicle, &backword_vehicle,
							&forward_dist, &backword_dist);


    	if (forward_dist != 0.0 && backword_dist != 0.0)
    	{	
        	cost = LANE_CHANGE_COST_SIDE_F / forward_dist + LANE_CHANGE_COST_SIDE_R / backword_dist;
    	}
	}
    return cost;
}
/***************************
* Compute the Change Lane Right cost from current position 
***************************/
double PathPlanner::CLRStateCost(telemetry t)
{
    double cost = 100000;
    if (t.lane != 3)
    {
		OtherVehicle forward_vehicle, backword_vehicle;
		double forward_dist, backword_dist;
		DistanceToCarsInLane(t.lane+1, &forward_vehicle, &backword_vehicle,
							&forward_dist, &backword_dist);

    	if (forward_dist != 0.0 && backword_dist != 0.0)
    	{	
        	cost = LANE_CHANGE_COST_SIDE_F / forward_dist + LANE_CHANGE_COST_SIDE_R / backword_dist;
    	}
	}
    return cost;
}

/**************************
* Compute a new waypoints list in a keep lane state.
**************************/
Path PathPlanner::ComputeKLPath(telemetry t)
{
	cout << "COMPUTE KL PATH" << endl;
	Path next_path;
	OtherVehicle forward_vehicle, backword_vehicle;
	double forward_dist, backword_dist;
	// Use the position of the sensor fusion data to compute the distance of the cars in the same lane
	DistanceToCarsInLane(t.lane, &forward_vehicle, &backword_vehicle,
						 &forward_dist, &backword_dist);

    double start_speed = t.car_speed;

    if (start_speed > MAX_SPEED)
        start_speed = MAX_SPEED;

    double end_speed = AdjustDistance2Speed(start_speed, forward_dist, forward_vehicle);;

    if (end_speed > MAX_SPEED)
        end_speed = MAX_SPEED;

    if (end_speed < MIN_SPEED)
        end_speed = MIN_SPEED;

	double s_end = t.car_s + PATH_PLAN_SECONDS * 0.5 * (start_speed + end_speed);

	next_path = Path({t.car_s, start_speed, 0}, 
				{s_end, end_speed, 0}, 
				{Lane2D(t.lane), 0, 0},
				{Lane2D(t.lane), 0, 0});
	ComputeMinimumJerkPath(&next_path);
	return next_path;
}

// Determine new setpoints whilst going on the left course 
Path PathPlanner::ComputeCLLPath(telemetry t)
{
	cout << "COMPUTE CLL PATH" << endl;
	double s_end = t.car_s + PATH_PLAN_SECONDS * t.car_speed;

	Path next_path = Path({t.car_s, t.car_speed, 0}, 
					 {s_end, t.car_speed, 0}, 
					 {Lane2D(t.lane), 0, 0},
					 {Lane2D(t.lane-1), 0, 0});

	ComputeMinimumJerkPath(&next_path);
	return next_path;
}

// Determine new setpoints whilst going on the right course 
Path PathPlanner::ComputeCLRPath(telemetry t)
{
	cout << "COMPUTE CLR PATH" << endl;
	double s_end = t.car_s + PATH_PLAN_SECONDS * t.car_speed;

	Path next_path = Path({t.car_s, t.car_speed, 0}, 
					 	  {s_end, t.car_speed, 0}, 
					 	  {Lane2D(t.lane), 0, 0},
					 	  {Lane2D(t.lane+1), 0, 0});

	ComputeMinimumJerkPath(&next_path);
	return next_path;
}

double PathPlanner::AdjustDistance2Speed(double start_speed, double rough_front_dist, 
										 OtherVehicle forward_vehicle)
{
	double vehicle_s_speed =  ComputeCarSpeed(forward_vehicle);
	
	double speed_adj  = KL_ACC_COEFF * rough_front_dist;
	
	speed_adj = speed_adj > MAX_TRACKING_CHANGE ? MAX_TRACKING_CHANGE : speed_adj;
	speed_adj = speed_adj < MIN_TRACKING_CHANGE ? MIN_TRACKING_CHANGE : speed_adj;
	
	double target_speed = start_speed + speed_adj;

	if(target_speed > vehicle_s_speed){
		if(rough_front_dist > KL_DIST_THR*3)
			return target_speed;
		else if(rough_front_dist >= KL_DIST_THR)
		{
			return vehicle_s_speed + (target_speed-vehicle_s_speed)* ((rough_front_dist-KL_DIST_THR)/(3*KL_DIST_THR));
		}
		else if(rough_front_dist < KL_DIST_THR){
			return MIN_SPEED + (vehicle_s_speed - MIN_SPEED)* ((rough_front_dist)/(KL_DIST_THR));
		}
	}
	return target_speed;
}

double PathPlanner::ComputeCarSpeed(OtherVehicle v)
{

	double car_heading  = rad2deg(atan(v.vy/v.vx));
	int waypoint0 = ClosestWaypoint(v.x,v.y,road_map.fined_x,road_map.fined_y);
	int waypoint1 = NextWaypoint(v.x,v.y,car_heading,road_map.fined_x,road_map.fined_y);
	double lane_heading = rad2deg(atan((road_map.fined_y[waypoint1] - road_map.fined_y[waypoint0]) / 
						  (road_map.fined_x[waypoint1] - road_map.fined_x[waypoint0])));
	double delta_theta = car_heading - lane_heading;

	double mag_v = sqrt(pow(v.vx,2) + pow(v.vy,2));
	double v_s = mag_v * cos(delta_theta);
	double v_d = mag_v * sin(delta_theta);
	
	return mag_v;
}



// Determine distance to closest car in front of us in a given lane
void PathPlanner::DistanceToCarsInLane(int lane, OtherVehicle *f_v, OtherVehicle *b_v,
									   double *f_d, double *b_d)
{
    double closest_front = 100000;
	double closest_behind = 100000;
	for (std::map<int,OtherVehicle>::iterator it=this->car.close_vehicles.begin(); it!=this->car.close_vehicles.end(); ++it)
    {
		OtherVehicle close_v = (OtherVehicle)it->second;
		if (D2lane(close_v.d) == lane)
        {	
            double diff = close_v.s - this->car.s;
            if (diff > 0.0 && diff < closest_front){
				*f_v = close_v;
				closest_front = diff;
			}
                
			if (diff < 0.0 && -diff < closest_behind){
				*b_v = close_v;
				closest_behind = -diff;
			}
        }
    }
	*f_d = closest_front;
	*b_d = closest_behind;
}

double PathPlanner::Lane2D(int lane)
{
	double d = (double)(((lane - 1) * this->laneWidth)+(this->laneWidth/2)); 
	if (lane == 1)
		d += 0.1;
	if (lane == 3)
		d -= 0.1;
    return d;
}
int PathPlanner::D2lane(double d)
{	
	int lane = ceil(d/this->laneWidth);
	
	if(lane >0 & lane <4)
		return lane;
	return 0;
}

// Compute minimum jerk path and convert to map coordinates
void PathPlanner::ComputeMinimumJerkPath(Path *p)
{
    // Generate minimum jerk path in Frenet coordinates
    vector<double> next_s_vals = ComputeMinimumJerk(p->S_start, p->S_end);
    vector<double> next_d_vals = ComputeMinimumJerk(p->D_start, p->D_end);
    // Convert Frenet coordinates to map coordinates
    vector<double> next_x_vals = {};
    vector<double> next_y_vals = {};
    for (int i=0; i<next_s_vals.size(); i++)
    {
        vector<double> xy = getXY(fmod(next_s_vals[i], MAX_TRACK_S),
                                  next_d_vals[i],
                                  this->road_map.fined_s,
                                  this->road_map.fined_x,
                                  this->road_map.fined_y);
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }
	p->X = next_x_vals;
    p->Y = next_y_vals;
    p->last_s = next_s_vals[next_s_vals.size()-1];
    p->last_d = next_d_vals[next_d_vals.size()-1];

}

vector<double> PathPlanner::ComputeMinimumJerk(vector<double> start, vector<double> end)
{
    MatrixXd A = MatrixXd(3,3);
    VectorXd b = VectorXd(3);
    VectorXd x = VectorXd(3);

    double t  = PATH_PLAN_SECONDS;
    double t2 = t * t;
    double t3 = t * t2;
    double t4 = t * t3;
    double t5 = t * t4;

    A <<   t3,    t4,    t5,
         3*t2,  4*t3,  5*t4,
         6*t,  12*t2, 20*t3;

    b << end[0] - (start[0] + start[1] * t + 0.5 * start[2] * t2),
         end[1] - (start[1] + start[2] * t),
         end[2] - start[2];

    x = A.inverse() * b;

    double a0 = start[0];
    double a1 = start[1];
    double a2 = start[2] / 2.0;
    double a3 = x[0];
    double a4 = x[1];
    double a5 = x[2];

    vector<double> result;
    for (double t=PATH_PLAN_INCREMENT; t<PATH_PLAN_SECONDS+0.001; t+=PATH_PLAN_INCREMENT) 
    {
        double t2 = t * t;
        double t3 = t * t2;
        double t4 = t * t3;
        double t5 = t * t4;
        double r = a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5;
        result.push_back(r);
    }
    return result;
}

Path PathPlanner::ComputeStupidTrajectory(vector<double> previous_path_x, vector<double> previous_path_y,
										  double car_s, double car_d, double yaw)
{
	Path p;
	int prev_path_size = previous_path_x.size();
	vector<double> car_last_sd{car_s, car_d};
	
	if(prev_path_size > 0){
		
		cout << "Smaller than horizon - timestamp: " << previous_path_x.size()<< endl;
		int prev_trj_used = min((int)(previous_path_x.size()),this->update_timestamp);

		for(int i = 0; i < prev_trj_used; i++){
			p.X.push_back(previous_path_x[i]);
			p.Y.push_back(previous_path_y[i]);
		}

		car_last_sd = getFrenet(previous_path_x[prev_trj_used], previous_path_y[prev_trj_used],
								yaw, this->road_map.fined_x,this->road_map.fined_y);

		cout << car_s << "-" << car_last_sd[0] << endl;
	}

	double dist_inc = 0.30;
	for(int i = 0; i < this->horizon - p.X.size(); i++)
		{	
			vector<double> X_Y = getXY(car_last_sd[0]+(dist_inc*i), +2, road_map.fined_s, road_map.fined_x, road_map.fined_y);
			p.X.push_back(X_Y[0]);
			p.Y.push_back(X_Y[1]);
		}
	
	if (prev_path_size != 0) {
		for (int i = 1 ; i < p.X.size() - prev_path_size; ++i){
			p.X[i ] = p.X[i - 1] + (p.X[i + 1] - p.X[ i ]);
			p.Y[i ] = p.Y[i - 1] + (p.Y[i + 1] - p.Y[i ]);
		}
		
	}
	
	return p;
}

void PathPlanner::ComputeSensorFusion()
{}

double PathPlanner::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int PathPlanner::ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int PathPlanner::NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> PathPlanner::getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> PathPlanner::getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

