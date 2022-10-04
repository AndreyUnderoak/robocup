#include "main.h"

//TIME
#include <chrono>
//TIME


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

		//TIME
		using std::chrono::high_resolution_clock;
		using std::chrono::duration_cast;
		using std::chrono::duration;
		using std::chrono::microseconds;
		//TIME


		std::vector<double> theta_array_goal = {0.5, 0.5, 0.5, 0.5, 0.5};

		// //orientation
		// double orient = theta_array_goal.at(1) + theta_array_goal.at(2) + theta_array_goal.at(3);

		//get forward goal coors by goal theta array
		std::vector<double> coor = arm_kinematics.get_coors(arm_kinematics.forward(theta_array_goal));
		std::cout << "coor = " << std::endl;
		std::vector<double> coor_s = arm_kinematics.get_coors(arm_kinematics.forward_1(theta_array_goal));

		// for(size_t i = 0; i < 3; i++)
		// 	std::cout << coor.at(i) << std::endl;

		// boost::posix_time::time_duration time_diff_array[2];
		// std::cout << "coor_s = " << std::endl;
		// for(size_t i = 0; i < 3; i++)
		// 	std::cout << coor_s.at(i) << std::endl;
		// сравнение forward и forward_1
		Matrix<double, 4, 4> T_S = arm_kinematics.forward_1(theta_array_goal);
		
		//get inverse theta array
		double orient = theta_array_goal.at(1) + theta_array_goal.at(2) + theta_array_goal.at(3);
		//Theta 1
		double t1 = - (arm_kinematics.theta_1(coor.at(X_COOR), coor.at(Y_COOR), ALBOW_UP));
		//R05
		Matrix<double, 3, 3> r05 = arm_kinematics.orientation(orient, t1, theta_array_goal.at(4), ALBOW_UP);
		
		//TIME
		auto an_start = high_resolution_clock::now();
		std::vector<double> theta_array = arm_kinematics.inverse(r05, coor, ALBOW_UP, ALBOW_UP, abs(orient), theta_array_goal.at(4));
		auto an_stop = high_resolution_clock::now();
		auto an_total = duration_cast<microseconds>(an_stop - an_start);
		//TIME

		Matrix<double, 3, 3> r05_s = arm_kinematics.get_r(T_S);

		//TIME
		auto s_start = high_resolution_clock::now();
		std::vector<double> theta_array_S = arm_kinematics.inverse_1(r05_s, coor_s, ALBOW_UP, ALBOW_UP);
		auto s_stop = high_resolution_clock::now();
		auto s_total = duration_cast<microseconds>(s_stop - s_start);
		//TIME
		
		std::cout << "AN THETA --------------------------------------" << std::endl;
		for(size_t i = 0; i < 5; i++)
			std::cout << theta_array.at(i) << std::endl;
		
		std::cout << "TIME : " << an_total.count() << std::endl;
		
		std::cout << "S THETA ---------------------------------------" << std::endl;
		for(size_t i = 0; i < 5; i++)
			std::cout << theta_array_S.at(i) << std::endl;

		std::cout << "TIME : " << s_total.count() << std::endl;

		std::cout << "done" << std::endl;
		
		//theta_array.clear();

		//TEEEEEEEEEEEEEEEEEEEEEEEST
		size_t num = 0;
		uint64_t an_total_summ = 0;
		uint64_t s_total_summ = 0;

		for(double j1 = -M_PI/2; j1 < M_PI/2; j1 += M_PI/4){
			for(double j2 = -M_PI/2; j2 < M_PI/2; j2 += M_PI/4){
				for(double j3 = -M_PI/2; j3 < M_PI/2; j3 += M_PI/4){
					for(double j4 = -M_PI/2; j4 < M_PI/2; j4 += M_PI/4){
						for(double j5 = -M_PI/2; j5 < M_PI/2; j5 += M_PI/4){
							num++;

							theta_array_goal[0] = j1;
							theta_array_goal[1] = j2;
							theta_array_goal[2] = j3;
							theta_array_goal[3] = j4;
							theta_array_goal[4] = j5;
							
							coor = arm_kinematics.get_coors(arm_kinematics.forward(theta_array_goal));
							coor_s = arm_kinematics.get_coors(arm_kinematics.forward_1(theta_array_goal));

							T_S = arm_kinematics.forward_1(theta_array_goal);
							
							//get inverse theta array
							orient = theta_array_goal.at(1) + theta_array_goal.at(2) + theta_array_goal.at(3);
							//Theta 1
							t1 = - (arm_kinematics.theta_1(coor.at(X_COOR), coor.at(Y_COOR), ALBOW_UP));
							//R05
							r05 = arm_kinematics.orientation(orient, t1, theta_array_goal.at(4), ALBOW_UP);
							
							//TIME
							an_start = high_resolution_clock::now();
							theta_array = arm_kinematics.inverse(r05, coor, ALBOW_UP, ALBOW_UP, abs(orient), theta_array_goal.at(4));
							an_stop = high_resolution_clock::now();
							an_total = duration_cast<microseconds>(an_stop - an_start);
							//TIME

							r05_s = arm_kinematics.get_r(T_S);

							//TIME
							s_start = high_resolution_clock::now();
							theta_array_S = arm_kinematics.inverse_1(r05_s, coor_s, ALBOW_UP, ALBOW_UP);
							s_stop = high_resolution_clock::now();
							s_total = duration_cast<microseconds>(s_stop - s_start);
							//TIME
							
							an_total_summ += an_total.count();
							s_total_summ += s_total.count();
						}
					}
				}
			}
		}

		std::cout << "AFTER " << num << " TESTS" << std::endl;

		std::cout << (an_total_summ < s_total_summ ? "AN" : "S") << " FASTER" << std::endl;

		std::cout << "AN TOTAL TIME : " << an_total_summ << std::endl;
		std::cout << "S TOTAL TIME : " << s_total_summ << std::endl;


		//TEEEEEEEEEEEEEEEEEEEEEEEST EEEEEEEEEEEEEEEND
		if (youBotHasArm) {
			/*
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

				// std::cout << "Theta array =" << std::endl;
				// for(size_t i = 0; i < 5; i++)
		        //     std::cout << theta_array.at(i) << std::endl;

				std::cout << "done" << std::endl;

				//send
				// myYouBotManipulator->setJointData(arm_kinematics.get_youbot_angles(theta_array));
				SLEEP_MILLISEC(3000);

				theta_array.clear();
				
			}
			catch(const char* msg) {
				std::cout << msg << std::endl;
			}*/

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
