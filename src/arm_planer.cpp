#include "../include/arm_planer.hpp"

bool Arm_planer::make_trajectory(std::vector<double> coordinates_start, std::vector<double> coordinates_finish){
    std::vector<double> theta_arrays;

    //TODO time generation
    
    //get array of theta by time step
    theta_arrays =  this->get_trajectory_by_cubic(coordinates_start, coordinates_finish, time_start, time_finish, step);

    //to youbot
    this->trajectory_to_youbot(theta_arrays, time_start, time_finish, step);
}

std::vector<std::vector<double>> Arm_planer::get_trajectory_by_cubic(){

}

void Arm_planer::trajectory_to_youbot(std::vector<std::vector<double>> theta_arrays, time_start, time_finish, step){

    for(size_t i = 0; i < theta_arrays.size(); i++)

}