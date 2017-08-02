#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

class GNB {
public:

    vector<string> possible_labels = {"left","keep","right"};
    
    vector<double> priors ;
    vector<vector<double>> means;
    vector<vector<double>> variances;
    vector<string> classes;

    /*** Constructor ***/
    GNB();

    /*** Destructor ***/
    virtual ~GNB();

    void train(vector<vector<double>> data, vector<string>  labels);

    string predict(vector<double> feature_vector);

    inline vector<string> getUniqueClasses(vector<string> labels);
    inline vector<double> computePriors(vector<string> labels, vector<string> classes);
    inline void computeMeanAndVariance(vector<vector<double>> class_feature, vector<double> *mean, vector<double> *variance);
    inline double getMean(vector<double> data);
    inline double getVariance(vector<double> data, double mean);
    inline double getDensityFucntion(double x, double mean_y, double variance_y);
};

#endif