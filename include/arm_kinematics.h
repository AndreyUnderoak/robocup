#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include <math.h>
#include <ctime>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

using namespace youbot;

#define SEC_TO_MICROSEC 1000000

const vector<JointAngleSetpoint> joint_colomn = {
                    169  * M_PI / 180 * radian,
                    65   * M_PI / 180 * radian,
                    -146 * M_PI / 180 * radian,
                    102  * M_PI / 180 * radian,
                    167  * M_PI / 180 * radian
                };

JointAngleSetpoint update_joints(double, YouBotManipulator* );