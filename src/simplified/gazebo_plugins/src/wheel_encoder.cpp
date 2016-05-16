#include "wheel_encoder.h"
#include "std_msgs/Int64.h"
#include <sstream>

namespace gazebo 
{

GZ_REGISTER_MODEL_PLUGIN(WheelEncoderPlugin)

WheelEncoderPlugin::WheelEncoderPlugin() {}
WheelEncoderPlugin::~WheelEncoderPlugin() {}

template<typename T> bool retrieveFromSDF(sdf::ElementPtr _sdf, T& t, std::string elementName) {
	bool retrieved = false;
	sdf::ElementPtr element = _sdf->GetElement(elementName);
	if(element) {
		retrieved = element->GetValue()->Get(t);
	}
	return retrieved;
}

void WheelEncoderPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
	this->model = _model;

	//load required parameters from sdf
	std::string jointName;
	if(retrieveFromSDF(_sdf, jointName, "joint_name") && retrieveFromSDF(_sdf, ticksPerRotation, "ticks_per_revolution")) 
	{
		this->wheelJoint = model->GetJoint(jointName);

		if(!this->wheelJoint) {
			gzerr << "WheelEncoder plugin:: couldn't retrieve joint: "  << jointName << ", plugin will not remain active.\n";
			return;
		} else if (this->wheelJoint->GetAngleCount() != 1) {
			gzerr << "WheelEncoder plugin:: Joint: "  << jointName << " has multiple or zero D.O.F, plugin will not remain active.\n";
			return;
		}
	} else {
		gzerr << "WheelEncoder plugin:: missing required parameters joint_name and\\or ticks_per_revolution\n";
		return;
	}
	// load optional parameters from sdf
	if(!retrieveFromSDF(_sdf, topicName, "topic_name")) {
		//use default topic name
		topicName = this->model->GetName() + "/" + jointName + "/encoder";
	}

	reverseDirection = retrieveFromSDF(_sdf, reverseDirection, "reverse_direction") && reverseDirection;

	this->updateConnection =
	event::Events::ConnectWorldUpdateBegin(boost::bind(&WheelEncoderPlugin::Update, this));

	if (!ros::isInitialized())
	{
	  int argc = 0;
	  char** argv = NULL;
	  ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler |
	                                  ros::init_options::AnonymousName);
	}
	rosNode = new ros::NodeHandle();
	encoderTickPublisher = rosNode->advertise<std_msgs::Int64>(topicName, 1000);

	return;
}
void WheelEncoderPlugin::Init() {

}
void WheelEncoderPlugin::Reset() {

}
void WheelEncoderPlugin::Update() {
	gazebo::math::Angle angle;

	if(wheelJoint) {
		angle = wheelJoint->GetAngle(0);
		std_msgs::Int64 msg;
	    msg.data = (int)(ticksPerRotation * angle.Radian() / 2.0 / M_PI);
	    if(reverseDirection)
	    	msg.data *= -1;
		encoderTickPublisher.publish(msg);
	}
	//maybe this is too often??
	ros::spinOnce();
}
	
}