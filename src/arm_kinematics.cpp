#include "../include/arm_kinematics.hpp"


Matrix<double, 4, 4> Arm_kinematics::a_matrix(double theta, size_t link_num){
	Matrix<double, 4, 4> m;

	m(0,0) =   cos(theta + this->links[link_num].offset);
	m(0,1) = - sin(theta + this->links[link_num].offset) * cos(this->links[link_num].alpha);
	m(0,2) =   sin(theta + this->links[link_num].offset) * sin(this->links[link_num].alpha);
	m(0,3) =   this->links[link_num].a*cos(theta + this->links[link_num].offset);

	m(1,0) = sin(theta + this->links[link_num].offset);
	m(1,1) = cos(theta + this->links[link_num].offset) * cos(this->links[link_num].alpha);
	m(1,2) = - cos(theta + this->links[link_num].offset) * sin(this->links[link_num].alpha);
	m(1,3) = this->links[link_num].a*sin(theta + this->links[link_num].offset);

	m(2,0) = 0; 
	m(2,1) = sin(this->links[link_num].alpha);
	m(2,2) = cos(this->links[link_num].alpha);
	m(2,3) = this->links[link_num].d;

	m(3,0) = 0; 
	m(3,1) = 0;
	m(3,2) = 0;
	m(3,3) = 1;

	return m;
};

Matrix<double, 4, 4> Arm_kinematics::h_matrix(std::vector<double> theta_array, size_t start, size_t finish){
			Matrix<double, 4, 4> m = this->a_matrix( - theta_array.at(start), start);

			for(size_t i = start + 1; i < finish; i++) 
				m = m * this->a_matrix( theta_array.at(i), i);

            return m;
};

Matrix<double, 4, 4> Arm_kinematics::forward(std::vector<double> theta_array){
	return this->h_matrix(theta_array, 0, 5);
}





