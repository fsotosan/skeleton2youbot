#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <skeleton2youbot/YouBotManipulatorJointAngles.h>

#define PI 3.1415927

tfScalar getScalarProduct(const tf::Vector3* v1, const tf::Vector3* v2) {
	return v1->x() * v2->x() + v1->y() * v2->y() + v1->z() * v2->z();
}

tfScalar getMagnitude(const tf::Vector3* v1) {
	return sqrt(getScalarProduct(v1,v1));
}

tfScalar getAngle(const tf::Vector3* inVector1, const tf::Vector3* inVector2) {

	tfScalar theScalarProd = getScalarProduct(inVector1,inVector2);
	tfScalar theM1 = getMagnitude(inVector1);
	tfScalar theM2 = getMagnitude(inVector2);

	if ((theM1 > 0)&&(theM2 > 0)) {

		tfScalar theCosTheta = theScalarProd / (theM1 * theM2);
		tfScalar theTheta = acos(theCosTheta);

		//printf("Angle S-E-H: %g (|ES| = %g, |EH| = %g)\n",theta * 180 / PI,mES,mEH);
		return 180*theTheta/PI;

	}

	return 0;

}

int main(int argc, char** argv){

	ros::init(argc, argv, "skeleton2youbot");
	ros::NodeHandle theNodeHandle;


	tf::TransformListener theListener;

	ros::Publisher thePublisher = theNodeHandle.advertise<skeleton2youbot::YouBotManipulatorJointAngles>("cmd_ref_vel",10); // Publicaremos en el canal 'cmd_pos_ref'. Buffer de 10 mensajes

	while (theNodeHandle.ok()){

		tf::StampedTransform 	theTransformElbowShoulder,
								theTransformElbowHand,
								theTransformShoulderHip,
								theTransformHipKnee,
								theTransformFootKnee;

		ros::Rate theRate(5.0);


		try{


			//ros::Time theTime = ros::Time::now();
			ros::Time theTime = ros::Time(0);
			theListener.waitForTransform("/right_elbow_1", "/right_shoulder_1", theTime,ros::Duration(3.0));
			theListener.lookupTransform("/right_elbow_1", "/right_shoulder_1", theTime, theTransformElbowShoulder);
			theListener.lookupTransform("/right_elbow_1", "/right_hand_1", theTime, theTransformElbowHand);
			//theListener.lookupTransform("/right_shoulder_1", "/right_hip_1", theTime, theTransformShoulderHip);
			theListener.lookupTransform("/torso_1", "/right_hip_1", theTime, theTransformShoulderHip);
			theListener.lookupTransform("/right_hip_1", "/right_knee_1", theTime, theTransformHipKnee);
			theListener.lookupTransform("/right_foot_1", "/right_knee_1", theTime, theTransformFootKnee);

			tf::Vector3 theVectorElbowHand = theTransformElbowHand.getOrigin();
			tf::Vector3 theVectorElbowShoulder = theTransformElbowShoulder.getOrigin();
			tf::Vector3 theVectorShoulderElbow = -1 * theVectorElbowShoulder;
			tf::Vector3 theVectorShoulderHip = theTransformShoulderHip.getOrigin();
			tf::Vector3 theVectorHipShoulder = -1 * theVectorShoulderHip;
			tf::Vector3 theVectorHipKnee = theTransformHipKnee.getOrigin();
			tf::Vector3 theVectorKneeHip = -1 * theVectorHipKnee;
			tf::Vector3 theVectorFootKnee = theTransformFootKnee.getOrigin();
			tf::Vector3 theVectorKneeFoot = -1 * theVectorFootKnee;

			tfScalar theAngleHandElbowShoulder = getAngle(&theVectorElbowHand,&theVectorElbowShoulder);
			tfScalar theAngleElbowShoulderHip = getAngle(&theVectorShoulderElbow,&theVectorShoulderHip);
			tfScalar theAngleShoulderHipKnee = getAngle(&theVectorHipShoulder,&theVectorHipKnee);
			tfScalar theAngleHipKneeFoot = getAngle(&theVectorKneeHip,&theVectorKneeFoot);

			printf("HES: %g, ESH: %g, SHK: %g, HKF: %g\n",theAngleHandElbowShoulder,theAngleElbowShoulderHip,theAngleShoulderHipKnee,theAngleHipKneeFoot);

			skeleton2youbot::YouBotManipulatorJointAngles theMsg;

			theMsg.A1 = theAngleHandElbowShoulder;
			theMsg.A2 = theAngleElbowShoulderHip;
			theMsg.A3 = theAngleShoulderHipKnee;
			theMsg.A4 = theAngleHipKneeFoot;

			thePublisher.publish(theMsg);

		} catch (tf::TransformException ex) {

			ROS_ERROR("%s",ex.what());

		}

		theRate.sleep();
	}

	return 0;

};
