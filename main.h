#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include <math.h>
#include <ctime>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include "include/arm_kinematics.h"

using namespace youbot;

#define SEC_TO_MICROSEC 1000000

