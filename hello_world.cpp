#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include <math.h>
#include <ctime>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>



#define SEC_TO_MICROSEC 1000000

using namespace youbot;

JointAngleSetpoint update_joints(double, YouBotManipulator* );
double to_rad(double a);
vector<JointAngleSetpoint> joint_colomn();
void change_joint_coor(int inc_dec, int num, YouBotManipulator* myYouBotManipulator);
int getch(void);

int main() {
	/* configuration flags for different system configuration (e.g. base without arm)*/
	bool youBotHasBase = false;
	bool youBotHasArm = false;

	/* define velocities */
	double translationalVelocity = 0.05; //meter_per_second
	double rotationalVelocity = 0.2; //radian_per_second

	/* create handles for youBot base and manipulator (if available) */
	YouBotBase* myYouBotBase = 0;
	YouBotManipulator* myYouBotManipulator = 0;

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

		if (youBotHasBase) {

			/* forward */
			longitudinalVelocity = translationalVelocity * meter_per_second;
			transversalVelocity = 0 * meter_per_second;
			myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "drive forward";
			SLEEP_MILLISEC(2000);

			/* backwards */
			longitudinalVelocity = -translationalVelocity * meter_per_second;
			transversalVelocity = 0 * meter_per_second;
			myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "drive backwards";
			SLEEP_MILLISEC(2000);

			/* left */
			longitudinalVelocity = 0 * meter_per_second;
			transversalVelocity = translationalVelocity * meter_per_second;
			angularVelocity = 0 * radian_per_second;
			myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "drive left";
			SLEEP_MILLISEC(2000);

			/* right */
			longitudinalVelocity = 0 * meter_per_second;
			transversalVelocity = -translationalVelocity * meter_per_second;
			angularVelocity = 0 * radian_per_second;
			myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "drive right";
			SLEEP_MILLISEC(2000);

			/* stop base */
			longitudinalVelocity = 0 * meter_per_second;
			transversalVelocity = 0 * meter_per_second;
			angularVelocity = 0 * radian_per_second;
			myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "stop base";
		}

		if (youBotHasArm) {


			myYouBotManipulator->setJointData(joint_colomn());

			SLEEP_MILLISEC(2000);

			boost::posix_time::ptime timer_start  = boost::posix_time::microsec_clock::local_time();
			boost::posix_time::ptime timer_now  = boost::posix_time::microsec_clock::local_time();
			const boost::posix_time::ptime timer_const_start  = boost::posix_time::microsec_clock::local_time();
			boost::posix_time::time_duration diff = timer_start - timer_const_start;
			vector<JointSensedAngle> sData;
			char key(' ');

			while (key != 'x') {
				timer_start  = boost::posix_time::microsec_clock::local_time();
				diff = timer_start - timer_const_start;
				
				key = getch();
				cout << key;
				switch (key)
				{
				case 'q':
					change_joint_coor(1, 1, myYouBotManipulator);
					break;
				case 'a':
					change_joint_coor(-1, 1, myYouBotManipulator);
					break;
				case 'w':
					change_joint_coor(1, 2, myYouBotManipulator);
					break;
				case 's':
					change_joint_coor(-1, 2, myYouBotManipulator);
					break;
				case 'e':
					change_joint_coor(-1, 3, myYouBotManipulator);
					break;
				case 'd':
					change_joint_coor(1, 3, myYouBotManipulator);
					break;
				case 'r':
					change_joint_coor(1, 4, myYouBotManipulator);
					break;
				case 'f':
					change_joint_coor(-1, 4, myYouBotManipulator);
					break;
				case 't':
					change_joint_coor(1, 5, myYouBotManipulator);
					break;
				case 'g':
					change_joint_coor(-1, 5, myYouBotManipulator);
					break;
				case 'z':
					myYouBotManipulator->setJointData(joint_colomn());
					break;
				}
				

				boost::posix_time::ptime timer_now  = boost::posix_time::microsec_clock::local_time();
				diff = timer_now - timer_const_start;


				SLEEP_MICROSEC(2000 - diff.total_microseconds());
			}

			
			
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

int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

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

void change_joint_coor(int inc_dec, int num, YouBotManipulator* myYouBotManipulator){
	try{
		std::vector<JointSensedAngle> jdata;
		myYouBotManipulator->getJointData(jdata);
		std::vector<JointAngleSetpoint> j2data;
		for(int i = 0; i < 5; i++){
			j2data.push_back(jdata[i].angle);
		}
		j2data[num -1].angle+=0.1*inc_dec*radian;
		myYouBotManipulator->setJointData(j2data);
	}
	catch(std::out_of_range& e){
		LOG(info) << "STOP IDIOT.";
	}
}

double to_rad(double a){
	return a * M_PI / 180;
}

vector<JointAngleSetpoint> joint_colomn(){
	vector<JointAngleSetpoint> jData = {
		169  * M_PI / 180 * radian,
		65   * M_PI / 180 * radian,
		-146 * M_PI / 180 * radian,
		102  * M_PI / 180 * radian,
		167  * M_PI / 180 * radian
	};
	return jData;
}

