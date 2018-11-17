
#include <ros/ros.h>

#include <ros_esccontrol.h>

#include "Joystick.h"

#define QUEUE_SIZE 5

ros::Publisher * throttlePublisher;

double sidewaysPower = 0;
double forwardsPower = 0;
double verticalPower = 0;


void onButtonChange(std::shared_ptr<joy::ButtonChangeEvent> changeEvent)
{
	std::cout << "TestJoystick: Button " << changeEvent->getButtonNumber() << " is now " << std::boolalpha << changeEvent->isPressed() << std::endl;
}
void onAxisChange(std::shared_ptr<joy::AxisChangeEvent> changeEvent)
{
	std::cout << "TestJoystick: Axis " << changeEvent->getAxisNumber() << " is now " << changeEvent->getValue() << std::endl;

	if(changeEvent->getAxisNumber() == 0)
	{
		sidewaysPower = changeEvent->getValue()/-32767.0;
	}
	else if(changeEvent->getAxisNumber() == 1)
	{
		forwardsPower = changeEvent->getValue()/-32767.0;
	}
	else if(changeEvent->getAxisNumber() == 4)
	{
		verticalPower = changeEvent->getValue()/-32767.0;
	}

	
	ros_esccontrol::setMotor(M_HORIZ_LEFT, (forwardsPower - sidewaysPower), *throttlePublisher);
	ros_esccontrol::setMotor(M_HORIZ_RIGHT, (forwardsPower + sidewaysPower), *throttlePublisher);

	float vertPower = verticalPower/2;
	ros_esccontrol::setMotor(M_VERT_FRONTLEFT, vertPower, *throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_BACKLEFT, vertPower, *throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_BACKRIGHT, vertPower, *throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_FRONTRIGHT, vertPower, *throttlePublisher);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtle_teleop");
	ros::NodeHandle nh("turtle_teleop");


	throttlePublisher = new ros::Publisher(nh.advertise<ros_esccontrol::ESCThrottle>("/esccontrol/esc_throttle", QUEUE_SIZE));

	joy::Joystick joystick("/dev/input/js0");
	joystick.setButtonCallback(std::make_shared<joy::Joystick::ButtonCallback>(&onButtonChange));
	joystick.setAxisCallback(std::make_shared<joy::Joystick::AxisCallback>(&onAxisChange));


	ros::spin();
}
