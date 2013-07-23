#include <iostream>
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "ros/ros.h"
#include <skeleton2youbot/YouBotManipulatorJointAngles.h>
#include <sensor_msgs/LaserScan.h>

using namespace youbot;
using namespace std;

#define PI 3.1415927
/*
#define A1_FOLD_RADIANS		0.100692
#define A1_UNFOLD_RADIANS 	5.84014
#define A2_FOLD_RADIANS		0.0100692
#define A2_UNFOLD_RADIANS 	2.617
#define A3_FOLD_RADIANS		-5.02655
#define A3_UNFOLD_RADIANS 	-0.015708
#define A4_FOLD_RADIANS		0.221239
#define A4_UNFOLD_RADIANS 	3.4292
#define A5_FOLD_RADIANS		0.11062
#define A5_UNFOLD_RADIANS 	5.64159
*/
#define A1_FOLD_RADIANS		0.5
#define A1_UNFOLD_RADIANS 	5.5
#define A2_FOLD_RADIANS		0.5
#define A2_UNFOLD_RADIANS 	2.0
#define A3_FOLD_RADIANS		-4.0
#define A3_UNFOLD_RADIANS 	-0.5
#define A4_FOLD_RADIANS		0.5
#define A4_UNFOLD_RADIANS 	3.0
#define A5_FOLD_RADIANS		0.5
#define A5_UNFOLD_RADIANS 	5.0

void posCallback(const skeleton2youbot::YouBotManipulatorJointAngles::ConstPtr& inAngles);
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& inScanMsg);


int i = 0;
YouBotManipulator* myYouBotArm = 0;
YouBotBase* myYouBotBase = 0;
bool myYouBotHasArm = false;
bool myYouBotHasBase = false;
ros::NodeHandle* myNodeHandle = 0;

int main(int argc, char** argv) {

	ros::init(argc, argv, "youbot_arm_move");
	ros::NodeHandle theNodeHandle;
	myNodeHandle = &theNodeHandle;
	//ros::Subscriber theSubscriber = theNodeHandle.subscribe("cmd_ref_pos", 10, posCallback);
	ros::Subscriber theBaseSubscriber = theNodeHandle.subscribe("scan", 10, lidarCallback);

	ROS_INFO("Programa youbot_arm_move iniciado");
	ROS_INFO("YOUBOT_CONFIGURATIONS_DIR: %s",YOUBOT_CONFIGURATIONS_DIR);

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

double translateRange(float inInput, float inMinInput, float inMaxInput, float inMinOutput, float inMaxOutput) {

	if (inInput < inMinInput) {
		inInput = inMinInput;
	} else if (inInput > inMaxInput) {
		inInput = inMaxInput;
	}

	return inMinOutput + (inInput - inMinInput)*(inMaxOutput - inMinOutput) / (inMaxInput - inMinInput);

}

void posCallback(const skeleton2youbot::YouBotManipulatorJointAngles::ConstPtr& inAngles) {

	JointAngleSetpoint theJointAngle;


	//theJointAngle = translateRange(inAngles->A1, 0, PI, A1_FOLD_RADIANS, A1_UNFOLD_RADIANS) * radian;
	//myYouBotArm->getArmJoint(1).setData(theJointAngle);
	
	theJointAngle = translateRange(inAngles->A2, 0, PI, A2_FOLD_RADIANS, A2_UNFOLD_RADIANS) * radian;
	myYouBotArm->getArmJoint(2).setData(theJointAngle);
	theJointAngle = translateRange(inAngles->A3, 0, PI, A3_FOLD_RADIANS, A3_UNFOLD_RADIANS) * radian;
	myYouBotArm->getArmJoint(3).setData(theJointAngle);

	theJointAngle = translateRange(inAngles->A4, 0, PI, A4_FOLD_RADIANS, A4_UNFOLD_RADIANS) * radian;
	myYouBotArm->getArmJoint(4).setData(theJointAngle);

	theJointAngle = translateRange(inAngles->A5, 0, PI, A5_FOLD_RADIANS, A5_UNFOLD_RADIANS) * radian;
	myYouBotArm->getArmJoint(5).setData(theJointAngle);

	// Indicamos que se ha recibido un mensaje de posición

	ROS_INFO("%d - Recibido mensaje YouBotManipulatorJointAngles: A = (%g,%g,%g,%g,%g), \n", i++, inAngles->A1, inAngles->A2, inAngles->A3, inAngles->A4, inAngles->A5);


}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& inScanMsg) {

	JointAngleSetpoint theJointAngle;

	int i;

	float theRangeMin = inScanMsg->range_min;
	float theRangeMax = inScanMsg->range_max;
	float theAngleMin = inScanMsg->angle_min;
	float theAngleMax = inScanMsg->angle_max;
	float theAngleIncrement = inScanMsg->angle_increment;

	double theClosestPointAngle = theAngleMin;
	float theClosestPointRange = theRangeMax;

	int theNumDirections = (int)(theAngleMax - theAngleMin)/theAngleIncrement;
	for(i = 0; i < theNumDirections; i++) {
		if ((inScanMsg->ranges[i] >= theRangeMin)&&(inScanMsg->ranges[i] <= theRangeMax)) {
			if (inScanMsg->ranges[i] < theClosestPointRange) {
				theClosestPointAngle = theAngleMin + i*theAngleIncrement;
				theClosestPointRange = inScanMsg->ranges[i];
			}
		}
	}

	theClosestPointAngle -= PI/2;

	theJointAngle = translateRange(theClosestPointAngle, 0, PI, A1_FOLD_RADIANS, A1_UNFOLD_RADIANS) * radian;
	myYouBotArm->getArmJoint(1).setData(theJointAngle);


	ROS_INFO("%d - Recibido mensaje LaserScan con objeto más cercano a  %g grados (%g m)\n", i++, theClosestPointAngle, theClosestPointRange);


}
