#include "../include/arm_kinematics.hpp"

Arm_kinematics::Arm_kinematics()
{

	//---------YOUBOT PARAMETERS START------------
	this->links_length = {0.033, 0.147, 0.155, 0.135, 0.2175};

	this->youbot_angles = {169, 65, -146, 102, 167};

	this->links = {
            link(true,  0,       this->links_length.at(1),    this->links_length.at(0), -M_PI/2),
            link(false, -M_PI/2, 0,                           this->links_length.at(2),  0),
            link(false, 0,       0,                           this->links_length.at(3),  0),
            link(false, M_PI/2,  0,                           0,                        -M_PI/2),
            link(false, 0,       -(this->links_length.at(4)), 0,                         0)};
	
	this->joints = {
            joint(to_radians(-169), to_radians(169)),
            joint(to_radians(-65 ), to_radians(90 )),
            joint(to_radians(-151), to_radians(146)),
            joint(to_radians(-102), to_radians(102)),
            joint(to_radians(-167), to_radians(167))
        };
	//-----------YOUBOT PARAMETERS END-------------
	

}

Matrix<double, 4, 4> Arm_kinematics::forward(std::vector<double> theta_array){
	return this->h_matrix(theta_array, 0, 5);
}

std::vector<double> Arm_kinematics::inverse(std::vector<double> coordinates, uint8_t conf_t_1, uint8_t conf_t_3, double ee_y_orientation, double ee_z_orientation){
	std::vector<double> theta_array;

	//Theta 1
	theta_array.push_back(- (theta_1(coordinates.at(X_COOR), coordinates.at(Y_COOR), conf_t_1)));
	
	//R05
	Matrix<double, 3, 3> r05 = orientation(ee_y_orientation, theta_array.at(0), ee_z_orientation, conf_t_1);

	//Go to P
	std::vector<double> p = get_p(coordinates, r05);

	//Theta 2, 3
	theta_array.push_back( theta_2(p, - theta_array.at(0), conf_t_1, conf_t_3));
	theta_array.push_back( theta_3(p, - theta_array.at(0), conf_t_1, conf_t_3));

	std::vector<double> theta_array_s = theta_array;
	//---------------S-----------------
	double phis = phi(r05, p, theta_array_s.at(0));
	theta_array_s.push_back(theta_4_s(phis,  theta_array_s.at(1),  theta_array_s.at(2)));
	theta_array_s.push_back(theta_5_s(phis, r05, p, theta_array_s.at(1), theta_array_s.at(2)));
	//---------------S END------------

	//---------------AN------------------
	//R35
	Matrix<double, 3, 3> r35 = inverse_orientation(theta_array, r05);

	//Theta 4, 5
	theta_array.push_back(theta_4(r35));
	theta_array.push_back(theta_5(r35));

	// for(size_t i = 0; i < 5; i++)
	// 	if(!(this->joints.at(i).in_range(theta_array.at(i))))
	// 		throw "ERROR: out of joints range";

	//------------AN END----------------

	p.clear();

	std::cout << "-----------------------" << std::endl;

	std::cout << "ANDREY_NANCY OZK  = " << std::endl;

	for(size_t i = 0; i < 5; i++)
		std::cout << theta_array.at(i) << std::endl;
	
	std::cout << "SASHA OZK   =  " << std::endl;
	
	for(size_t i = 0; i < 5; i++)
		std::cout << theta_array_s.at(i) << std::endl;

	std::cout << "-----------------------" << std::endl;
	
	return theta_array;
}

Arm_angles Arm_kinematics::inverse_get_all(std::vector<double> coordinates, double ee_y_orientation, double ee_z_orientation){
	Arm_angles aa;
	for(size_t conf_t_1 = 1; conf_t_1 < 3; conf_t_1 ++){
		for(size_t conf_t_3 = 1; conf_t_3 < 3; conf_t_3 ++){
			try{
				std::vector<double> theta_array = this->inverse(coordinates, conf_t_1, conf_t_3, ee_y_orientation, ee_z_orientation);
				aa.add_conf(theta_array, conf_t_1, conf_t_3);
				theta_array.clear();
			}
			catch(const char* msg) {
			}	
		}
	}
	return aa;
}



std::vector<JointAngleSetpoint> Arm_kinematics::get_youbot_angles(std::vector<double> theta_array){
	std::vector<JointAngleSetpoint> youbot_theta_array;
	for(size_t i = 0; i < 5; i++)
		youbot_theta_array.push_back((theta_array.at(i) + this->to_radians(this->youbot_angles.at(i))) * radian);	

	return youbot_theta_array;

}

Matrix<double, 4, 4> Arm_kinematics::a_matrix(double theta, size_t link_num){
	Matrix<double, 4, 4> m;
	
	m(0,0) =   cos(theta + this->links.at(link_num).offset);
	m(0,1) = - sin(theta + this->links.at(link_num).offset) * cos(this->links.at(link_num).alpha);
	m(0,2) =   sin(theta + this->links.at(link_num).offset) * sin(this->links.at(link_num).alpha);
	m(0,3) =   this->links.at(link_num).a*cos(theta + this->links.at(link_num).offset);

	m(1,0) =   sin(theta + this->links.at(link_num).offset);
	m(1,1) =   cos(theta + this->links.at(link_num).offset) * cos(this->links.at(link_num).alpha);
	m(1,2) = - cos(theta + this->links.at(link_num).offset) * sin(this->links.at(link_num).alpha);
	m(1,3) =   this->links.at(link_num).a*sin(theta + this->links.at(link_num).offset);

	m(2,0) =   0; 
	m(2,1) =   sin(this->links.at(link_num).alpha);
	m(2,2) =   cos(this->links.at(link_num).alpha);
	m(2,3) =   this->links.at(link_num).d;

	m(3,0) =   0; 
	m(3,1) =   0;
	m(3,2) =   0;
	m(3,3) =   1;

	return m;
};

Matrix<double, 4, 4> Arm_kinematics::h_matrix(std::vector<double> theta_array, size_t start, size_t finish){
	Matrix<double, 4, 4> m = this->a_matrix( - theta_array.at(start), start);

	for(size_t i = start + 1; i < finish; i++) 
		m = m * this->a_matrix( theta_array.at(i), i);

	return m;
};


double Arm_kinematics::theta_1(double x, double y, uint8_t conf_t_1){
	if (conf_t_1 == ALBOW_UP)
		return atan2(y, x);
	else{
		double t_1 = atan2(y,x);
		if(t_1 < 0)
			return (t_1 + M_PI);
		else	
			return (t_1 - M_PI); 
	}
		
}
double Arm_kinematics::theta_2(std::vector<double> coordinates, double theta_1, uint8_t conf_t_1, uint8_t conf_t_3){
	double xp       = coordinates.at(X_COOR) * cos(theta_1) + coordinates.at(Y_COOR) * sin(theta_1);
	double t_3_temp = theta_3(coordinates, theta_1, conf_t_1, conf_t_3);
	double beta     = atan2(this->links_length.at(3)*sin(t_3_temp), this->links_length.at(2)+this->links_length.at(3)*cos(t_3_temp));

	return atan2(xp - this->links_length.at(0), coordinates.at(Z_COOR) - this->links_length.at(1)) - beta;
}
double Arm_kinematics::theta_3(std::vector<double> coordinates, double theta_1, uint8_t conf_t_1, uint8_t conf_t_3){
	double xp       =   coordinates.at(X_COOR) * cos(theta_1) + coordinates.at(Y_COOR) * sin(theta_1);
	double p_square =   (xp - this->links_length.at(0))*(xp - this->links_length.at(0)) + (coordinates.at(Z_COOR) - this->links_length.at(1))*(coordinates.at(Z_COOR) - this->links_length.at(1));
	double cosT_3   = - (this->links_length.at(2)*(this->links_length.at(2)) + this->links_length.at(3)*(this->links_length.at(3)) - p_square) / (2 * (this->links_length.at(2)) * (this->links_length.at(3)));
	
	// if (abs(cosT_3) > 1.0001)
	// 	throw "ERROR: out of links range";
	if (abs(cosT_3) > 1)
		cosT_3 = round(cosT_3);
	if (conf_t_1 == 1){
		if (conf_t_3 == ALBOW_DOWN)
			return - atan2(sqrt(1 - cosT_3*cosT_3), cosT_3);
		else
			return   atan2(sqrt(1 - cosT_3*cosT_3), cosT_3);
		
	}
	else{
		if (conf_t_3 == ALBOW_DOWN)
			return   atan2(sqrt(1 - cosT_3*cosT_3), cosT_3);
		else
			return - atan2(sqrt(1 - cosT_3*cosT_3), cosT_3);
	}
}
double Arm_kinematics::theta_4(Matrix<double, 3, 3> r35){
	return atan2(r35(0,2), -r35(1,2)) + M_PI/2;
}

double Arm_kinematics::theta_5(Matrix<double, 3, 3> r35){
	return atan2(r35(2,0), r35(2,1));
}
// ----------------------------FUNCTIONS FOR S--------------------------------------------------------
double Arm_kinematics::phi(Matrix<double, 3, 3> r, std::vector<double> p, double theta_1){
	std::vector<double> z5;
	std::vector<double> z0 = {0,0,1};
	z5[0] = r(0,2);
	z5[1] = r(1,2);
	z5[2] = r(2,2);
	Matrix<double, 3, 3> Rz0 = Matrix3d::Zero();
	Rz0(0,0) =   cos(theta_1);
	Rz0(0,1) = - sin(theta_1);
	Rz0(1,0) =   sin(theta_1);
	Rz0(1,1) =   cos(theta_1);
	Rz0(2,2) =   1;
	std::vector<double> z5 = Rz0 * z5;
	double cosphi = for(i=0; i < 3; i++) res+=z5[i] * z0[i];
	double sinphi = sqrt(1 - cosphi*cosphi)*-((z5[0] > 0) - (z5[0] < 0));
	double phi  = atan2(sinphi, cosphi);
	double phi_temp = phi;
	if (phi_temp == 0) phi = m.pi / 2
    if (phi_temp == m.pi) || (phi_temp == -m.pi) phi = -m.pi / 2
	if (phi_temp < 0) phi_temp = -phi_temp;
	phi = M_PI/2 - phi_temp;
	return phi;
}

double Arm_kinematics::theta_4_s(double phi, double theta_2, double theta_3){
	return 1;
}

double Arm_kinematics::theta_5_s(double phi, Matrix<double, 3, 3> r, std::vector<double> p, double theta_2, double theta_3){
	return 1;
}
//--------------------------FUNCTIONS FOR S END----------------------------------------------
Matrix<double, 3, 3> Arm_kinematics::orientation(double ee_y_orientation, double theta_1, double ee_z_orientation, uint8_t conf_t_1){
	theta_1 = -theta_1;
	if (conf_t_1 == ALBOW_DOWN)
		theta_1 += M_PI;

	Matrix<double, 3, 3> m1 = Matrix3d::Zero(); 
	Matrix<double, 3, 3> m2 = Matrix3d::Zero(); 
	Matrix<double, 3, 3> m3 = Matrix3d::Zero();

	m1(0,0) =   cos(theta_1);
	m1(0,1) = - sin(theta_1);
	m1(1,0) =   sin(theta_1);
	m1(1,1) =   cos(theta_1);
	m1(2,2) =   1;

	m2(0,0) =   cos(ee_y_orientation - M_PI);
	m2(0,2) =   sin(ee_y_orientation - M_PI);
	m2(1,1) =   1;
	m2(2,0) = - sin(ee_y_orientation - M_PI);
	m2(2,2) =   cos(ee_y_orientation - M_PI);

	if (conf_t_1 == ALBOW_DOWN)
		ee_z_orientation -= M_PI;

	m3(0,0) =   cos(ee_z_orientation);
	m3(0,1) = - sin(ee_z_orientation);
	m3(1,0) =   sin(ee_z_orientation);
	m3(1,1) =   cos(ee_z_orientation);
	m3(2,2) =   1;
		
	Matrix<double, 3, 3> m4 = m1 * m2;
	m4 = m4 * m3;
	
	return m4;
}

std::vector<double> Arm_kinematics::get_p(std::vector<double> coordinates, Matrix<double, 3, 3> r05){
	Matrix<double, 3, 1> coordinates_m = {
		coordinates.at(X_COOR),
		coordinates.at(Y_COOR),
		coordinates.at(Z_COOR)
	};

	Matrix<double, 3, 1> ooz_m ={
		0,
		0,
		- (this->links_length.at(4))
	};

	Matrix<double, 3, 1> p = coordinates_m - (r05 * ooz_m);

	std::vector<double> p_v = {
		p(0,0),
		p(1,0),
		p(2,0)
	};
	
	return p_v;
}

Matrix<double, 3, 3> Arm_kinematics::inverse_orientation(std::vector<double> theta_array, Matrix<double, 3, 3> r05){
	Matrix<double, 4, 4> h03 = h_matrix(theta_array, 0, 3);
	Matrix<double, 3, 3> tr03;

	for(size_t i = 0; i < 3; i++)
		for(size_t j = 0; j < 3; j++)
			tr03(i,j) = h03(j,i);

	Matrix<double, 3, 3> r35 = tr03 * r05;

	return r35;
}

Arm_kinematics::link::link(bool flip, 
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

Arm_kinematics::joint::joint(double min,
							 double max){
								 this->max = max;
								 this->min = min;
							};

bool Arm_kinematics::joint::in_range(double angle){
	return ((angle > this->min) && (angle < this->max)) ? true : false; 
}

std::vector<double> Arm_kinematics::get_coors(Matrix<double, 4, 4> h_matrix){
	std::vector<double> coordinates = {
		h_matrix(0,3),
		h_matrix(1,3),
		h_matrix(2,3)
	};
	return coordinates;
}

double Arm_kinematics::to_radians(double angle){
	return angle * M_PI / 180;
}	


void Arm_angles::add_conf(std::vector<double> theta_array, uint8_t conf_t_1, uint8_t conf_t_3){
	this->theta_arrays.insert({(conf_t_1*10 + conf_t_3), theta_array});
}
std::vector<double> Arm_angles::get_theta_array(uint8_t conf_t_1, uint8_t conf_t_3){
	if(this->is_exist(conf_t_1, conf_t_3))
		return this->theta_arrays.at((conf_t_1*10 + conf_t_3));
	else
		throw "ERROR: no configurations here";
}
bool Arm_angles::is_exist(uint8_t conf_t_1, uint8_t conf_t_3){
	return (this->theta_arrays.count(conf_t_1*10 + conf_t_3) > 0) ? true : false;
}
std::vector<double> Arm_angles::get_any(){
	if(!(this->theta_arrays.empty())){
		auto it=this->theta_arrays.end();
		it--;
		return it->second;
	}
	else
		throw "ERROR: no configurations here";

}