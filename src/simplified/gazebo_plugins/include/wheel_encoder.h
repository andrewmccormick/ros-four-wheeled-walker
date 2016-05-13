#ifndef WHEEL_ENCODER_PLUGIN_H 
#define WHEEL_ENCODER_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <boost/bind.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h> 
#include <ros/ros.h>

namespace gazebo 
{

class WheelEncoderPlugin : public ModelPlugin 
{
public:
	WheelEncoderPlugin();
	virtual ~WheelEncoderPlugin();
protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();
  virtual void Reset();
  virtual void Update();
private:
  physics::ModelPtr model;
  physics::JointPtr wheelJoint;

  ros::NodeHandle* rosNode;
  ros::Publisher encoderTickPublisher;

  event::ConnectionPtr updateConnection;

  unsigned int ticksPerRotation;
  std::string topicName;
  bool reverseDirection;
};

} //gazebo namespace

#endif 