#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include "std_msgs/Int64.h"
#include <vector>

class EncoderOdometry {
private:
	struct TimeStampedTick {
		TimeStampedTick(long int ticksIn, ros::Time timeIn) : ticks(ticksIn), time(timeIn) {}
		long int ticks;
		ros::Time time;
	};

public: 
	EncoderOdometry();

	void leftSubCallback(const std_msgs::Int64& ticks);
	void rightSubCallback(const std_msgs::Int64& ticks);
	void periodicPublishCallback(const ros::TimerEvent& event);

private:
	ros::NodeHandle nodeHandle;
	ros::Publisher odomPublisher;
	ros::Subscriber leftWheelSubscriber;
	ros::Subscriber rightWheelSubscriber;

	ros::Timer publishTimer;

	std::vector<TimeStampedTick> leftWheelVector;
	std::vector<TimeStampedTick> rightWheelVector;
    double computeVelocity(std::vector<TimeStampedTick>* v);

	int ticksPerRotation;
	double wheelRadius;
	double rearTrack;
};

EncoderOdometry::EncoderOdometry()
{
	std::string leftWheelTopic, rightWheelTopic;
	if(!nodeHandle.getParam("leftWheelTopic", leftWheelTopic))
		leftWheelTopic = "walkernew/rearLeftEncoder"; 
	if(!nodeHandle.getParam("rightWheelTopic", rightWheelTopic))
		rightWheelTopic = "walkernew/rearRightEncoder"; 

	if(!nodeHandle.getParam("wheelRadius", wheelRadius))
		wheelRadius = 0.2413;
	if(!nodeHandle.getParam("rearTrack", rearTrack))
		rearTrack = 0.5;
	if(!nodeHandle.getParam("ticksPerRotation", ticksPerRotation))
		ticksPerRotation = 400;

	leftWheelSubscriber = nodeHandle.subscribe(leftWheelTopic, 1000, &EncoderOdometry::leftSubCallback, this);
	rightWheelSubscriber = nodeHandle.subscribe(rightWheelTopic, 1000, &EncoderOdometry::rightSubCallback, this);
	odomPublisher = nodeHandle.advertise<nav_msgs::Odometry>("odom/encoder", 50);
	publishTimer = nodeHandle.createTimer(ros::Duration(0.1), &EncoderOdometry::periodicPublishCallback, this, false);
}

void EncoderOdometry::leftSubCallback(const std_msgs::Int64& ticks) {
	TimeStampedTick tst(ticks.data, ros::Time::now());
	leftWheelVector.push_back(tst);
}

void EncoderOdometry::rightSubCallback(const std_msgs::Int64& ticks) {
	TimeStampedTick tst(ticks.data, ros::Time::now());
	rightWheelVector.push_back(tst);	
}

/*
	computes a wheel velocity given a set of time stamped wheel 
	O(n) algorithm (least squared line of best fit)
*/
double EncoderOdometry::computeVelocity(std::vector<TimeStampedTick>* v) {
	// x = time, y = ticks
	// first: compute means
	double xMean = 0.0;
	double yMean = 0.0;
	for(int i = 0; i < v->size(); i++) {
		xMean += v->at(i).time.toSec();
		yMean += v->at(i).ticks;
	}
	xMean /= v->size();
	yMean /= v->size();

	/*     n 
          Sum ((xi - xbar) * (yi - ybar))
           i
    m =  --------------------------------					based on least squares method
	       n 
          Sum ((xi - xbar)^2)
           i
	*/

    double numerator = 0.0;
    double denominator = 0.0;

    double xDiff, yDiff;
    for(int i = 0.0; i < v->size(); i++) {
    	xDiff = v->at(i).time.toSec() - xMean;
    	yDiff = v->at(i).ticks - yMean;
    	numerator += (xDiff * yDiff);
    	denominator += (xDiff * xDiff);
    }

    /*     units of m are ticks / second, 
    	   but we must convert to m/s       */
    return numerator / denominator * wheelRadius / ticksPerRotation;
}

void EncoderOdometry::periodicPublishCallback(const ros::TimerEvent& event) {
	static ros::Time lastTime = ros::Time::now();
	ros::Time currentTime = ros::Time::now();

	if(lastTime == currentTime || leftWheelVector.size() == 0 || rightWheelVector.size() == 0) 
		return;

	double leftVelocity = 10 * computeVelocity(&leftWheelVector);
	double rightVelocity = 10 * computeVelocity(&rightWheelVector);
	leftWheelVector.clear();
	rightWheelVector.clear();

	/* Compute estimated movement base_link/base_footprint 
	   (the coordinate system of interest) 					
	*/
	double base_link_velocity = (leftVelocity + rightVelocity) / 2.0;
	double base_link_angular_velocity = (rightVelocity - leftVelocity) / rearTrack;
	ros::Time timeStamp((currentTime.toSec() + lastTime.toSec()) / 2.0);

	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = timeStamp;
	odom_msg.header.frame_id = "odom";
	odom_msg.child_frame_id = "base_footprint";
	
	//Twist is in the robot's coordinate system
	odom_msg.twist.twist.linear.x = base_link_velocity;
	odom_msg.twist.twist.linear.y = 0;
	odom_msg.twist.twist.angular.z = base_link_angular_velocity;
	/* Note: this odometry data is missing a LOT of stuff
			 however it is being fused by the robot_localization package
			 which should be configured to ignore the rest of the data.
			 So we don't bother including it.
	*/
	odomPublisher.publish(odom_msg);	 
	lastTime = currentTime;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_odometry");
    EncoderOdometry encoderOdometry;
    ros::spin();

    return 0;
}