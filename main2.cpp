#include "main.h"
#include <vector>

double to_rad(double a){
	return a * M_PI / 180;
}

int main(){
    Arm_kinematics arm_kinematics;
    // Arm_kinematics& arm_kc = arm_kinematics;
    std::vector<double> ver = {M_PI/4,M_PI/4,-M_PI/8,M_PI/8,M_PI/4};

    std::cout <<"theta array = " << std::endl;

    for(size_t i = 0; i < 5; i++)
        std::cout << ver.at(i) << std::endl;

    std::vector<double> coor = arm_kinematics.get_coors(arm_kinematics.forward(ver));

    std::cout <<"coor goal = " << std::endl;
    for(size_t i = 0; i < 3; i++)
        std::cout << coor.at(i) << std::endl;
    // std::cout << ver << std::endl;
    try{
        std::vector<double> theta_array = arm_kinematics.inverse(coor, ALBOW_UP, ALBOW_DOWN, M_PI/2, M_PI/4);
        std::cout <<"theta array I = " << std::endl;
        for(size_t i = 0; i < 5; i++)
            std::cout << theta_array.at(i) << std::endl;

        std::cout <<"theta Y = " << std::endl;
        std::vector<double> coor_y = arm_kinematics.get_youbot_angles(theta_array);
        for(size_t i = 0; i < 5; i++)
            std::cout << coor_y.at(i) << std::endl;

        std::cout <<"coor I = " << std::endl;
        std::vector<double> coor_i = arm_kinematics.get_coors(arm_kinematics.forward(theta_array));
        for(size_t i = 0; i < 3; i++)
            std::cout << coor_i.at(i) << std::endl;
    }
    catch(const char* msg) {
        std::cout << msg << std::endl;
    }
    

	return 0;
}
