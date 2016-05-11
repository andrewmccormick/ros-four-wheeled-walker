#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <ros/ros.h>
#include <cmath>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
   right_spin_ = n_.advertise<std_msgs::Float64>("/walkernew/right_spinning_wheel_controller/command", 1000);
	 left_spin_= n_.advertise<std_msgs::Float64>("/walkernew/left_spinning_wheel_controller/command", 1000);
   right_turn_ = n_.advertise<std_msgs::Float64>("/walkernew/right_turning_wheel_controller/command", 1000);
   left_turn_ = n_.advertise<std_msgs::Float64>("/walkernew/left_turning_wheel_controller/command", 1000);
    //Topic you want to subscribe
    sub_ = n_.subscribe("/cmd_vel", 1000, &SubscribeAndPublish::callback, this);
  }

  void callback(const geometry_msgs::Twist& twist)
  {
  
    std_msgs::Float64 linearSpeed;
    std_msgs::Float64 turnAngle;

    /* We supply the controllers the speeds of the front wheels with the (negated*) angular velocity
       *negated, since for whatever reason the controller/orientation of joints works out this way 
        TODO: evaluate whether to flip controller to take positive angular velocity, then remove negation here
      First we compute wheel characteristics
    */
    /* TODO: these constants depend on walker configuration, which is editable.
             thus this info should be broadcast somehow over ros master (rosparam server)
             and then retrieved by this node
    */
    const double WHEEL_RADIUS = 0.2413;
    const double WHEEL_BASE = 6 * 0.0254;          // distance between rear axel and front axel
    const double REAR_TRACK = 0.501675;            // distance between two rear wheels
    const double FRONT_TRACK= 0.494083;            // distance between two front wheels

    //output variables, passed in as pointer, aka by reference
    double frontRightAngularSpeed, frontRightAngle, frontLeftAngularSpeed, frontLeftAngle;
    calc_wheel_characteristics(
      WHEEL_RADIUS, WHEEL_BASE, REAR_TRACK, FRONT_TRACK, twist.linear.x, twist.angular.z,
      &frontRightAngularSpeed, &frontRightAngle, &frontLeftAngularSpeed, &frontLeftAngle
      );
  
    //Note: Controllers actually take the inverse of angular speed for wheel turning
  	left_spin_.publish(-1*frontLeftAngularSpeed);
  	left_turn_.publish(frontLeftAngle);	
    right_spin_.publish(-1*frontRightAngularSpeed);
    right_turn_.publish(frontRightAngle);
  }

private:
  /*
    Due to the nature of ackermann steering, each wheel will:
      - turn at a different velocity, and 
      - be pointed a different direction
    In order to figure this out we need
      - wheel radius
      - wheel base (distance from rear axel to front axel)
      - front and rear track (distance from wheel to wheel)
      - desired velocity 
      - desired angular velocity
  */
  void calc_wheel_characteristics
  (double wheelRadius, double wheelBase, double rearTrack, double frontTrack, double velocity, double angularVelocity, //input variables
   double* frontRightAngularSpeed, double* frontRightAngle, double* frontLeftAngularSpeed, double* frontLeftAngle      //output variables
   ) {
        const double MINIMUM_TURN_TOLERANCE = 0.0001;
        if(fabs(angularVelocity) < MINIMUM_TURN_TOLERANCE) {
          (*frontLeftAngularSpeed) = (fabs(velocity) / wheelRadius);
          (*frontRightAngularSpeed) = (fabs(velocity) / wheelRadius);
          (*frontLeftAngle) = (*frontRightAngle) = 0;
        } else {
          ROS_INFO("Hit else statement (ie turning)\n");
          double radiusOfCurvatureOfPath = fabs(velocity) / angularVelocity;
          double horizontalDistance_centreOfCurvatureToFrontLeft  = (radiusOfCurvatureOfPath - 0.5 * frontTrack),
                 horizontalDistance_centreOfCurvatureToFrontRight = (radiusOfCurvatureOfPath + 0.5 * frontTrack);

         *frontLeftAngle  = atan(fabs(wheelBase / horizontalDistance_centreOfCurvatureToFrontLeft));
         *frontRightAngle = atan(fabs(wheelBase / horizontalDistance_centreOfCurvatureToFrontRight));
         if(angularVelocity < 0) {
          *frontLeftAngle  *= -1;
          *frontRightAngle *= -1;
          }

          double distance_centreOfCurvatureToFrontLeft = sqrt(horizontalDistance_centreOfCurvatureToFrontLeft*horizontalDistance_centreOfCurvatureToFrontLeft + wheelBase*wheelBase);
          double distance_centreOfCurvatureToFrontRight = sqrt(horizontalDistance_centreOfCurvatureToFrontRight*horizontalDistance_centreOfCurvatureToFrontRight + wheelBase*wheelBase);

          //angular velocity of wheel         =      speed of wheel centre over ground          / wheel radius 
          *frontLeftAngularSpeed = fabs(distance_centreOfCurvatureToFrontLeft * angularVelocity) / wheelRadius;
          *frontRightAngularSpeed = fabs(distance_centreOfCurvatureToFrontRight * angularVelocity) / wheelRadius;      
        }

        if(velocity < 0) {
          (*frontLeftAngle) += M_PI;
          (*frontRightAngle) += M_PI;
        }
          // *frontLeftAngle = angularVelocity;
          // *frontRightAngle = angularVelocity;

    // if(abs(angularVelocity) > MINIMUM_TURN_TOLERANCE) {

    //   double horizontalDistance_centreOfCurvatureToFrontLeft = (radiusOfCurvatureOfPath - 0.5 * frontTrack),
    //          horizontalDistance_centreOfCurvatureToFrontRight = (radiusOfCurvatureOfPath + 0.5 * frontTrack);

    //   *frontLeftAngle = atan(wheelBase / horizontalDistance_centreOfCurvatureToFrontLeft);
    //   *frontRightAngle = atan(wheelBase / horizontalDistance_centreOfCurvatureToFrontRight);

    //   if(velocity < 0) {
    //     (*frontLeftAngle) += M_PI;
    //     (*frontRightAngle) += M_PI;
    //   }

    //   double distance_centreOfCurvatureToFrontLeft = sqrt(horizontalDistance_centreOfCurvatureToFrontLeft*horizontalDistance_centreOfCurvatureToFrontLeft + wheelBase*wheelBase);
    //   double distance_centreOfCurvatureToFrontRight = sqrt(horizontalDistance_centreOfCurvatureToFrontRight*horizontalDistance_centreOfCurvatureToFrontRight + wheelBase*wheelBase);

    //   //                                    speed of wheel centre over ground           / wheel radius = angular velocity of wheel
    //   *frontLeftAngularSpeed = abs(distance_centreOfCurvatureToFrontLeft * angularVelocity) / wheelRadius;
    //   *frontRightAngularSpeed = abs(distance_centreOfCurvatureToFrontRight * angularVelocity) / wheelRadius;      
    // } else {
    //     *frontLeftAngle = 0;
    //     *frontRightAngle = 0;
    //     *frontLeftAngularSpeed = velocity / wheelRadius;
    //     *frontRightAngularSpeed = velocity / wheelRadius;
    // }
  }

  ros::NodeHandle n_; 
  ros::Publisher right_spin_;
  ros::Publisher left_spin_;
  ros::Publisher right_turn_;
  ros::Publisher left_turn_;
  ros::Subscriber sub_;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "base_controller");
    SubscribeAndPublish object;
    ros::spin();

    return 0;
}

