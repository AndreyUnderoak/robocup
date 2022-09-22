#pragma once

#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <eigen3/Eigen/Dense>

#include "arm_kinematics.hpp"

class Arm_planer{
    public:
        bool make_trajectory(std::vector<double> coordinates_start, std::vector<double> coordinates_finish);

        std::vector<std::vector<double>> get_trajectory_by_cubic(std::vector<double> coordinates_start, coordinates_finish, time_start, time_finish, step);

        void trajectory_to_youbot(std::vector<std::vector<double>> theta_arrays, time_start, time_finish, step);
}