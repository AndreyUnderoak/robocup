#include "main.h"
#include <vector>

double to_rad(double a){
	return a * M_PI / 180;
}

int main(){
    Arm_kinematics arm_kinematics;
    // Arm_kinematics& arm_kc = arm_kinematics;
    double theta_array[5] = {to_rad(169), to_rad(65), to_rad(-146), to_rad(102), to_rad(167)};
    std::vector<double> ver = {to_rad(169), to_rad(65), to_rad(-146), to_rad(102), to_rad(167)};
    // std::cout << ver << std::endl;
    std::cout << arm_kinematics.forward(ver) << std::endl;

	return 0;
}
