#pragma once

#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <eigen3/Eigen/Dense>

using namespace Eigen;

class Arm_kinematics{
    public:
        Matrix<double, 4, 4> forward(std::vector<double> theta_array);
        std::vector<double>              inverse();
        struct link {
            bool flip;
            double offset;
            double d;
            double a;
            double alpha;

            link(bool flip, 
                 double offset, 
                 double d, 
                 double a, 
                 double alpha){
                    this->flip   = flip;
                    this->offset = offset;
                    this->d      = d;
                    this->a      = a;
                    this->alpha  = alpha;
                };
        };

        struct joint {
            double min;
            double max;

            joint(double min,
                  double max){
                    this->max = max;
                    this->min = min;
                  }

            bool in_range(double angle){
                (angle > this->min) && (angle < this->max) ? return true : return false; 
            }
        }

        //length of the YOUBOT's links
        const double links_length[5] = {0.033, 0.147, 0.155, 0.135, 0.2175};

        //YOUBOT angles
        const double youbot_angles[5] = {169, 65, -146, 102, 167};

        link links[5] = {
            link(true,  0,       this->links_length[1],    this->links_length[0], -M_PI/2),
            link(false, -M_PI/2, 0,                        this->links_length[2], -M_PI),
            link(false, 0,       0,                        this->links_length[3],  M_PI),
            link(false, -M_PI/2, 0,                        0,                      M_PI/2),
            link(false, 0,       -(this->links_length[4]), 0,                      0)};

        std::vector<joint> joints = {
            joint(np.radians(-169), np.radians(169)),
            joint(np.radians(-65 ), np.radians(90 )),
            joint(np.radians(-151), np.radians(146)),
            joint(np.radians(-102), np.radians(102)),
            joint(np.radians(-167), np.radians(167))
        };

        Matrix<double, 4, 4> a_matrix(double theta, size_t link_num);

        Matrix<double, 4, 4> h_matrix(std::vector<double> theta_array, size_t start, size_t finish);

        double to_radians(double angle){

        }
};