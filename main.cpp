#include "main.h"



int main(){
    /* configuration flags for different system configuration (e.g. base without arm)*/
    bool youBotHasBase = false;
    bool youBotHasArm = false;

    /* define velocities */
	double translationalVelocity = 0.05; //meter_per_second
	double rotationalVelocity = 0.2; //radian_per_second

	/* create handles for youBot base and manipulator (if available) */
	YouBotBase* myYouBotBase = 0;
	YouBotManipulator* myYouBotManipulator = 0;

	//kinematics
	Arm_kinematics arm_kinematics;

    try {
		myYouBotBase = new YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
		myYouBotBase->doJointCommutation();

		youBotHasBase = true;
	} catch (std::exception& e) {
		LOG(warning) << e.what();
		youBotHasBase = false;
	}

	try {
		myYouBotManipulator = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
		myYouBotManipulator->doJointCommutation();
		myYouBotManipulator->calibrateManipulator();

		youBotHasArm = true;
	} catch (std::exception& e) {
		LOG(warning) << e.what();
		youBotHasArm = false;
	}


    /*
	* Variable for the base.
	* Here "boost units" is used to set values in OODL, that means you have to set a value and a unit.
	*/
	quantity<si::velocity> longitudinalVelocity = 0 * meter_per_second;
	quantity<si::velocity> transversalVelocity = 0 * meter_per_second;
	quantity<si::angular_velocity> angularVelocity = 0 * radian_per_second;

	/* Variable for the arm. */
	JointAngleSetpoint desiredJointAngle;

	try {
		/*
		 * Simple sequence of commands to the youBot:
		 */

		if (youBotHasArm) {
			//goal theta array
			std::vector<double> theta_array_goal = {0, 0, 0, 0, 0};

			//orientation
			double orient = theta_array_goal.at(1) + theta_array_goal.at(2) + theta_array_goal.at(3);

			//get forward goal coors by goal theta array
			std::vector<double> coor = arm_kinematics.get_coors(arm_kinematics.forward(theta_array_goal));
			std::cout << "coor = " << std::endl;
				for(size_t i = 0; i < 3; i++)
		            std::cout << coor.at(i) << std::endl;
			try{
				//get inverse theta array
				std::vector<double> theta_array = arm_kinematics.inverse(coor, ALBOW_UP, ALBOW_UP, abs(orient), 0);

				std::cout << "Theta array =" << std::endl;
				for(size_t i = 0; i < 5; i++)
		            std::cout << theta_array.at(i) << std::endl;

				std::cout << "done" << std::endl;

				//send
				myYouBotManipulator->setJointData(arm_kinematics.get_youbot_angles(theta_array));
				SLEEP_MILLISEC(3000);

				theta_array.clear();
			}
			catch(const char* msg) {
				std::cout << msg << std::endl;
			}

			theta_array_goal.clear();
			coor.clear();
		}
        } catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		std::cout << "unhandled exception" << std::endl;
	}

	/* clean up */
	if (myYouBotBase) {
		delete myYouBotBase;
		myYouBotBase = 0;
	}
	if (myYouBotManipulator) {
		delete myYouBotManipulator;
		myYouBotManipulator = 0;
	}

	LOG(info) << "Done.";

	return 0;
}
