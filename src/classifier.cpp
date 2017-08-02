#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>
#include "classifier.h"
#include <typeinfo>

/**
 * Initializes GNB
 */
GNB::GNB() {
    this->priors = vector<double>();
    this->means = vector<vector<double>>();
    this->variances = vector<vector<double>>();
    this->classes = vector<string> ();
}

GNB::~GNB() {}

void GNB::train(vector<vector<double>> data, vector<string> labels)
{
    // using default comparison:
    
    this->classes = getUniqueClasses(labels);
    this->priors = computePriors(labels, this->classes);
    
    vector<vector<vector<double>>> features_by_class(this->classes.size());
    for(int i = 0; i <this->classes.size();i++)
        features_by_class[i] = vector<vector<double>>(data[0].size());

    for(int i = 0;i<labels.size();i++){
        for(int j = 0;j<this->classes.size();j++){
            if(labels[i] == this->classes[j]){
                for(int t = 0;t<data[i].size();t++){
                    features_by_class[j][t].push_back(data[i][t]);
                }
                break;
            }
        }
    }
    
    this->means = vector<vector<double>> (features_by_class.size());
    this->variances = vector<vector<double>> (features_by_class.size());

    for(int c = 0;c<features_by_class.size(); c++){
        computeMeanAndVariance(features_by_class[c], &this->means[c], &this->variances[c]);
    }
}
// To be implemented with template in order to be able to apply the NB to all the labels
inline vector<string> GNB::getUniqueClasses(vector<string> labels){
    
    vector<string> classes (labels.size());
    std::sort(labels.begin(), labels.end());
    
    auto last = std::unique_copy ( labels.begin(), labels.end(), classes.begin() );
    classes.erase(last, classes.end()); 
    
    return classes;
}

inline vector<double> GNB::computePriors(vector<string> labels, vector<string> classes){
    
    vector<double> priors (labels.size());
    double count_class = .0;
    for(int i = 0; i<classes.size();i++){
        count_class =  std::count(labels.begin(), labels.end(), classes[i]);
        priors[i] = (float)count_class/labels.size();
    }
    return priors;   
}

inline void GNB::computeMeanAndVariance(vector<vector<double>> features, vector<double> *mean, vector<double> *variance)
{
        vector<double> tmp_mean = vector<double> (features.size());;
        vector<double> tmp_variance = vector<double>(features.size());
        cout<<features.size()<<endl;

        for(int i = 0; i<features.size();i++){
            tmp_mean[i] = getMean(features[i]);
            tmp_variance[i] = getVariance(features[i], tmp_mean[i]);
        }
        
        *mean = tmp_mean;
        *variance = tmp_variance;
}

inline double GNB::getMean(vector<double> data)
{
    double sum = 0.0;
    for(double a : data)
        sum += a;
    return sum/data.size();
}

inline double GNB::getVariance(vector<double> data, double mean)
{
    double temp = 0;
    for(double a :data)
        temp += (a-mean)*(a-mean);
    return temp/(data.size()-1);
}

//Create a function that calculates p(x | y):
inline double GNB::getDensityFucntion(double x, double mean_y, double variance_y)
{
    //Input the arguments into a probability density function
    return 1/(sqrt(2*M_PI*variance_y)) * exp((-pow((x-mean_y),2))/(2*variance_y)); 
}
string GNB::predict(vector<double> feature_vector)
{
    vector<double> class_prob (this->classes.size());
    
    for(int i = 0;i<this->classes.size();i++){
        class_prob[i] = this->priors[i];
        for(int j = 0; j<feature_vector.size();j++){
            double likelihood = getDensityFucntion(feature_vector[j],this->means[i][j],this->variances[i][j]);
            class_prob[i] *= likelihood;
        }
    }

    int max_index = distance(class_prob.begin(), max_element(class_prob.begin(), class_prob.end()));
	return this->classes[max_index];

}