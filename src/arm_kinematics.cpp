#include "../include/arm_kinematics.h"

//time (seconds)
JointAngleSetpoint update_joints(double time, YouBotManipulator* myYouBotManipulator){
	JointAngleSetpoint r = (sin(time*1.8 + M_PI/2) * M_PI/4 + 102  * M_PI / 180) * radian;
	myYouBotManipulator->getArmJoint(4).setData(r);
	//r = (sin(time) * M_PI/8 - M_PI/8 - 2.4) * radian;
	//myYouBotManipulator->getArmJoint(3).setData(r);
	/*r = (sin(time) * M_PI/16 - M_PI/16 + 1.5) * radian;
	myYouBotManipulator->getArmJoint(2).setData(r);*/
	return (sin(time*1.8 + M_PI/2) * M_PI/4 + 102  * M_PI / 180) * radian;
}