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

	Matrix2f _A;
	Matrix<float,1,2> _H;
	MatrixXf _Q;
	MatrixXf _R;
	MatrixXf P;
	MatrixXf K;
	Vector2f X;
	Vector2f X_predict;
	float Z;
	
	bool initialized;

	void sonarCallback(const std_msgs::Float32 msg);
};

SonarFilter::SonarFilter()
{
	sonar_sub = n.subscribe("/sonar", 1, &SonarFilter::sonarCallback, this);
	sonar_filtered_pub = n.advertise<std_msgs::Float32>("/sonar_filtered", 1);
	initialized = false;
	_A << 1, 1,
		  0, 1;
	_H << 1, 0;
	_Q = MatrixXf::Identity(2,2) * 1e-4;
	_R = MatrixXf::Identity(1,1) * 1e-2;
	P = MatrixXf::Identity(2,2);
	K = MatrixXf::Zero(2,2);

}

void SonarFilter::sonarCallback(const std_msgs::Float32 msg)
{
	Z = msg.data;

	if(initialized)
	{
		//predict
		X_predict = _A * X;
		P = _A * P * _A.transpose() + _Q;
		//correct
		K = P * _H.transpose() * (_H * P * _H.transpose() + _R).inverse();
		X = X_predict + K * (Z - _H * X_predict);
		P = P - K * _H * P;

	}else
	{
		X(0) = msg.data;
		X(1) = 0;
		initialized = true;
	}

	std_msgs::Float32 sonar_filtered;
	sonar_filtered.data = X(0);
	sonar_filtered_pub.publish(sonar_filtered);

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "sonar_filter");
	SonarFilter SonarFilter;
	ros::spin();
}