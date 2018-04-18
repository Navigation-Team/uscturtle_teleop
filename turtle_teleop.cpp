
#include <ros/ros.h>

#include <esccontrol_msgs/ESCThrottle.h>

#include "Joystick.h"

#define QUEUE_SIZE 5

ros::Publisher * throttlePublisher;

void setMotor(int num, float throttle)
{
	if(throttlePublisher != nullptr)
	{
		esccontrol_msgs::ESCThrottle throttleMessage;
		throttleMessage.motor_num = num;
		throttleMessage.power = throttle;
		throttlePublisher->publish(throttleMessage);	
	}
	
}

void onButtonChange(std::shared_ptr<joy::ButtonChangeEvent> changeEvent)
{
	std::cout << "TestJoystick: Button " << changeEvent->getButtonNumber() << " is now " << std::boolalpha << changeEvent->isPressed() << std::endl;
}
void onAxisChange(std::shared_ptr<joy::AxisChangeEvent> changeEvent)
{
	std::cout << "TestJoystick: Axis " << changeEvent->getAxisNumber() << " is now " << changeEvent->getValue() << std::endl;

	if(changeEvent->getAxisNumber() == 1)
	{
		setMotor(1, changeEvent->getValue()/-32767.0);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtle_teleop");
	ros::NodeHandle nh("turtle_teleop");

	joy::Joystick joystick("/dev/input/js2");
	joystick.setButtonCallback(std::make_shared<joy::Joystick::ButtonCallback>(&onButtonChange));
	joystick.setAxisCallback(std::make_shared<joy::Joystick::AxisCallback>(&onAxisChange));

	throttlePublisher = new ros::Publisher(nh.advertise<esccontrol_msgs::ESCThrottle>("/esccontrol/esc_throttle", QUEUE_SIZE));

	ros::spin();
}