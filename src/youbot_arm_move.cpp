#include <iostream>
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "ros/ros.h"
#include <skeleton2youbot/YouBotManipulatorJointAngles.h>

using namespace youbot;
using namespace std;

void posCallback(const skeleton2youbot::YouBotManipulatorJointAngles::ConstPtr& inAngles);


int i = 0;
YouBotManipulator* myYouBotArm = 0;
bool myYouBotHasArm = false;
ros::NodeHandle* myNodeHandle = 0;

int main(int argc, char** argv) {

	ros::init(argc, argv, "youbot_base_move");
	ros::NodeHandle theNodeHandle;
	myNodeHandle = &theNodeHandle;
	ros::Subscriber theSubscriber = theNodeHandle.subscribe("cmd_ref_pos", 10, posCallback);

	ROS_INFO("Programa youbot_manipulator_move iniciado");
	//ROS_INFO("YOUBOT_CONFIGURATIONS_DIR: %s",YOUBOT_CONFIGURATIONS_DIR);

	try {
		myYouBotArm = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
		myYouBotArm->doJointCommutation();
		myYouBotArm->calibrateManipulator();
		myYouBotHasArm = true;
	} catch (std::exception& e) {
		LOG(warning) << e.what();
		myYouBotHasArm = false;
	}

	ros::spin();

	cout << "Programa terminado" << endl;
	return 0;

}

void posCallback(const skeleton2youbot::YouBotManipulatorJointAngles::ConstPtr& inAngles) {


	// Indicamos que se ha recibido un mensaje de velocidad

	ROS_INFO("%d - Recibido mensaje YouBotManipulatorJointAngles: A = (%g,%g,%g,%g,%g), \n", i++, inAngles->A1, inAngles->A2, inAngles->A3, inAngles->A4, inAngles->A5);


}
