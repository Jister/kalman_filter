#include "ros/ros.h"
#include "Eigen/Dense"
#include "std_msgs/Float32.h"
#include <math.h>

using namespace Eigen;

class SonarFilter
{
public: 
	SonarFilter();
private:
	ros::NodeHandle n;
	ros::Subscriber sonar_sub;
	ros::Publisher sonar_filtered_pub;

	float _A;
	float _H;
	float _R;
	float _Q;
	float P;
	float K;
	float X;
	float X_predict;
	float Z;
	
	bool initialized;

	void sonarCallback(const std_msgs::Float32 msg);
};

SonarFilter::SonarFilter()
{
	sonar_sub = n.subscribe("/sonar", 1, &SonarFilter::sonarCallback, this);
	sonar_filtered_pub = n.advertise<std_msgs::Float32>("/sonar_filtered", 1);
	initialized = false;
	_A = 1;
	_H = 1;
	_Q = 1e-8;
	_R = 1e-2;

}

void SonarFilter::sonarCallback(const std_msgs::Float32 msg)
{
	Z = msg.data;

	if(initialized)
	{
		//predict
		X_predict = _A * X;
		P = _A * P * _A + _Q;
		//correct
		K = P * _H * 1 / (_H * P * 1 / _H + _R);
		X = X_predict + K * (Z - _H * X_predict);
		P = P - K * _H * P;

	}else
	{
		X = msg.data;
		K = 0;
		P = 0;
		initialized = true;
	}

	std_msgs::Float32 sonar_filtered;
	sonar_filtered.data = X;
	sonar_filtered_pub.publish(sonar_filtered);

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "sonar_filter");
	SonarFilter SonarFilter;
	ros::spin();
}