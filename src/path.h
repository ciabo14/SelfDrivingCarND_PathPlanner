#ifndef PATH_H
#define PATH_H

#include <vector>
#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>

using namespace std;

// Trajectory data represent the structure which includes the information about s,s_dot, s_dot_dot and 
// d,d_dot, d_dot_dot for a current trajectory or for the target trajectory
/**************************
 * Path object which includes both data for jerk minimization procedure, and also the computed X, Y trajectory  
 **************************/
class Path{
    
    public:

    string name;
	
    /**************************
	* S_start,S_end,D_start and D_end touples have the following meaning:
	* S_start[0] --> current s value
	* S_start[1] --> current s_dot value (s speed)
	* S_start[2] --> current s_dot_dot value (s acceleration)
	* the same for all the other touples
	**************************/
	
    vector<double> S_start {0, 0, 0};
    vector<double> S_end {0, 0, 0};
    double last_s;
    vector<double> D_start {0, 0, 0};
    vector<double> D_end {0, 0, 0};
    double last_d;


    vector<double> X;
    vector<double> Y;
    Path(){};
    Path(vector<double> S_start, vector<double> S_end, vector<double> D_start, vector<double> D_end){
        this->S_start = S_start;
        this->S_end = S_end;
        this->D_start = D_start;
        this->D_end = D_end;
    };
    virtual ~Path(){};
};
#endif /* PATH_H_ */