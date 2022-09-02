#pragma once

#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace youbot;

#define X_COOR 0
#define Y_COOR 1
#define Z_COOR 2

#define ALBOW_UP 1
#define ALBOW_DOWN 2


class Arm_kinematics{
    public:
        Arm_kinematics();

        Matrix<double, 4, 4> forward(std::vector<double> theta_array);
        std::vector<double>  inverse(std::vector<double> coordinates, uint8_t conf_t_1, uint8_t conf_t_3, double ee_y_orientation, double ee_z_orientation);

        std::vector<JointAngleSetpoint> get_youbot_angles(std::vector<double> theta_array);
        
        struct link {
            bool flip; double offset; double d; double a; double alpha;
            link(bool flip, double offset, double d, double a, double alpha);
        };

        struct joint {
            double min; double max;
            joint(double min,double max);
            bool in_range(double angle);
        };

        //length of the YOUBOT's links
        std::vector<double> links_length;

        //YOUBOT angles
        std::vector<double> youbot_angles;

        std::vector<link> links;
        std::vector<joint> joints;

        //A & H matrix for FORWARD
        Matrix<double, 4, 4> a_matrix(double theta, size_t link_num);
        Matrix<double, 4, 4> h_matrix(std::vector<double> theta_array, size_t start, size_t finish);
        std::vector<double> get_coors(Matrix<double, 4, 4> h_matrix);

        //Thetas for INVERSE
        double theta_1(double x, double y, uint8_t conf_t_1);
        double theta_2(std::vector<double> coordinates, uint8_t conf_t_1, uint8_t conf_t_3);
        double theta_3(std::vector<double> coordinates, uint8_t conf_t_1, uint8_t conf_t_3);
        double theta_4(Matrix<double, 3, 3> r35);
        double theta_5(Matrix<double, 3, 3> r35);

        Matrix<double, 3, 3> orientation(double ee_y_orientation, double theta_1, double ee_z_orientation, uint8_t conf_t_1);
        
        std::vector<double> get_p(std::vector<double> coordinates, Matrix<double, 3, 3> r05);

        Matrix<double, 3, 3> inverse_orientation(std::vector<double> theta_array, Matrix<double, 3, 3> r05);

        double to_radians(double angle){
            return angle * M_PI / 180;
        }
};