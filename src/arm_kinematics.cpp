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
Matrix<double, 4, 4> Arm_kinematics::forward_1(std::vector<double> theta_array){
	return this->h_matrix_1(theta_array);
}


std::vector<double> Arm_kinematics::inverse(Matrix<double, 3, 3> r05, std::vector<double> coordinates, uint8_t conf_t_1, uint8_t conf_t_3, double ee_y_orientation, double ee_z_orientation){
	std::vector<double> theta_array;

	//Theta 1
	theta_array.push_back(- (theta_1(coordinates.at(X_COOR), coordinates.at(Y_COOR), conf_t_1)));
	
	// //R05
	// Matrix<double, 3, 3> r05 = orientation(ee_y_orientation, theta_array.at(0), ee_z_orientation, conf_t_1);

	//Go to P
	std::vector<double> p = get_p(coordinates, r05);

	//Theta 2, 3
	theta_array.push_back( theta_2(p, - theta_array.at(0), conf_t_1, conf_t_3));
	theta_array.push_back( theta_3(p, - theta_array.at(0), conf_t_1, conf_t_3));

	boost::posix_time::ptime timer_start_an  = boost::posix_time::microsec_clock::local_time();

	//R35
	Matrix<double, 3, 3> r35 = inverse_orientation(theta_array, r05);

	//Theta 4, 5
	theta_array.push_back(theta_4(r35));
	theta_array.push_back(theta_5(r35));

	// for(size_t i = 0; i < 5; i++)
	// 	if(!(this->joints.at(i).in_range(theta_array.at(i))))
	// 		throw "ERROR: out of joints range";


	p.clear();

	return theta_array;
}
std::vector<double> Arm_kinematics::inverse_AN(Matrix<double, 4, 4> T, std::vector<double> theta_array_goal, uint8_t conf_t_1, uint8_t conf_t_3){
	std::vector<double> coor = get_coors(T);
	// double orient = theta_array_goal.at(1) + theta_array_goal.at(2) + theta_array_goal.at(3);

	// //Theta 1
	// double t1 = - (theta_1(coordinates.at(X_COOR), coordinates.at(Y_COOR), conf_t_1));
	
	// //R05
	// Matrix<double, 3, 3> r05 = orientation(ee_y_orientation, t1, ee_z_orientation, conf_t_1);

	// std::vector<double> theta_array = inverse(r05, coor, ALBOW_UP, ALBOW_UP, abs(orient), theta_array_goal.at(4));
	return coor;
}

std::vector<double> Arm_kinematics::inverse_1(Matrix<double, 3, 3> r05, std::vector<double> coors, uint8_t conf_t_1, uint8_t conf_t_3) {
	std::vector<double> theta_array;
	// THETA 1
	double theta_1 = theta_1_s(coors[0], coors[1], conf_t_1);
	theta_array.push_back(theta_1);

	// PHI
	double phi = Arm_kinematics::phi(r05, coors, theta_1);
	std::cout << " phi = " << phi << std::endl;
	// THETA 2 and 3
	std::vector<double> t23 = Arm_kinematics::theta_23_s(r05, coors, theta_1, conf_t_3);
	theta_array.push_back(t23[0]);
	theta_array.push_back(t23[1]);
	// THETA 4
	double theta_4 = theta_4_s(r05, phi, theta_1, t23[0], t23[1]);
	theta_array.push_back(theta_4);
	// THETA 5
	double theta_5 = theta_5_s(r05, theta_1, t23[0], t23[1], theta_4);
	theta_array.push_back(theta_5);

	return theta_array;
}

Arm_angles Arm_kinematics::inverse_get_all(std::vector<double> coordinates, double ee_y_orientation, double ee_z_orientation){
	Arm_angles aa;
	// for(size_t conf_t_1 = 1; conf_t_1 < 3; conf_t_1 ++){
	// 	for(size_t conf_t_3 = 1; conf_t_3 < 3; conf_t_3 ++){
	// 		try{
	// 			std::vector<double> theta_array = this->inverse(coordinates, conf_t_1, conf_t_3, ee_y_orientation, ee_z_orientation);
	// 			aa.add_conf(theta_array, conf_t_1, conf_t_3);
	// 			theta_array.clear();
	// 		}
	// 		catch(const char* msg) {
	// 		}	
	// 	}
	// }
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
// SASHA DH PARAMETERS ---------------------------------------------------------------------------
std::vector<double> as = { 33, 155, 135, 0, 0 };
std::vector<double> alphas = { M_PI / 2, 0, 0, M_PI / 2, M_PI };
std::vector<double> ds = { -147, 0, 0, 0, 217 };
std::vector<double> offsets = { 0, -M_PI / 2, 0, M_PI / 2, M_PI / 2 };
// -------------------------------------------------------------------------------------------------

// ----------------------------FUNCTIONS FOR S--------------------------------------------------------
Matrix<double, 4, 4> Arm_kinematics::a_matrix_1(double a, double alpha, double d, double theta, double offset) {
	double newtheta = theta + offset;
	Matrix<double, 4, 4> A;
	A << cos(newtheta), -sin(newtheta) * cos(alpha), sin(newtheta)* sin(alpha), a* cos(newtheta),
		sin(newtheta), cos(newtheta)* cos(alpha), -cos(newtheta) * sin(alpha), a* sin(newtheta),
		0, sin(alpha), cos(alpha), d,
		0, 0, 0, 1;
	return A;
}
Matrix<double, 4, 4> Arm_kinematics::h_matrix_1(std::vector<double> theta_array) {
	Matrix<double, 4, 4> A1 = this->a_matrix_1(as[0], alphas[0], ds[0], theta_array[0], offsets[0]);
	Matrix<double, 4, 4> A2 = this->a_matrix_1(as[1], alphas[1], ds[1], theta_array[1], offsets[1]);
	Matrix<double, 4, 4> A3 = this->a_matrix_1(as[2], alphas[2], ds[2], theta_array[2], offsets[2]);
	Matrix<double, 4, 4> A4 = this->a_matrix_1(as[3], alphas[3], ds[3], theta_array[3], offsets[3]);
	Matrix<double, 4, 4> A5 = this->a_matrix_1(as[4], alphas[4], ds[4], theta_array[4], offsets[4]);
	return A1 * A2 * A3 * A4 * A5;
};
Matrix<double, 3, 3> Arm_kinematics::get_r(Matrix<double, 4, 4> T) {
	Matrix<double, 3, 3> r;
	r << T(0, 0), T(0, 1), T(0, 2),
		T(1, 0), T(1, 1), T(1, 2),
		T(2, 0), T(2, 1), T(2, 2);
	return r;
}
Matrix<double, 3, 1> Arm_kinematics::get_z5(Matrix<double, 3, 3> r, double theta_1){
    Matrix<double, 3, 1> z5;
    z5(0,0) = r(0,2);
    z5(1,0) = r(1,2);
    z5(2,0) = r(2,2);
	Matrix<double, 3, 3> Rz0;
	Rz0 << cos(-theta_1), -sin(-theta_1), 0,
		sin(-theta_1), cos(-theta_1), 0,
		0, 0, 1;
	Matrix<double, 3, 1> z5_rotated = Rz0 * z5;
    return z5_rotated;
}
double Arm_kinematics::phi(Matrix<double, 3, 3> r, std::vector<double> p, double theta_1){
    Matrix<double, 3, 1> z5 = get_z5(r, theta_1);
    std::vector<double> z0 = {0,0,1};
    double cosphi = 0;
	for(size_t i=0; i < 3; i++) {
		cosphi += z5(i, 0) * z0[i];
	}
    double sinphi = sqrt(1 - cosphi*cosphi)*-((z5(0,0) > 0) - (z5(0,0) < 0));
    double phi  = atan2(sinphi, cosphi);
	std::cout << "phi old " << phi*180/M_PI << std::endl;
	
    double phi_temp = phi;
    if (phi_temp == 0) phi = M_PI / 2;
    if ((phi_temp == M_PI) || (phi_temp == -M_PI)) phi = -M_PI / 2;
    if (phi_temp < 0) phi_temp = -phi_temp;
    phi = M_PI/2 - phi_temp;
	
    return phi;
}
double Arm_kinematics::theta_1_s(double x, double y, uint8_t conf_t_1) {
	double theta_1 = atan2(y, x);
	if (conf_t_1 == ALBOW_DOWN) {
		if (theta_1 < 0) {
			theta_1 += M_PI;
		}
		else {
			theta_1 -= M_PI;
		}
	}
	return theta_1;
}
std::vector<double> Arm_kinematics::theta_23_s(Matrix<double, 3, 3> r, std::vector<double> p, double theta_1, uint8_t conf_t_3) {
	Matrix<double, 3, 1> z5 = get_z5(r, theta_1);
	std::cout << " z5 = " << z5 << std::endl;
	double phi = Arm_kinematics::phi(r, p, theta_1);
	double x_new = cos(-theta_1) * p[0] - sin(-theta_1) * p[1];
	std::vector<double> x_axis = { cos(theta_1), sin(theta_1), 0 };
	double xw;
	if (z5(0, 0) < 0) {
		xw = x_new - ds[4] * cos(phi);
	}
	else {
		xw = x_new + ds[4] * cos(phi);
	}
	double zw = p[2] + ds[4] * sin(phi) - ds[0];
	double d = sqrt(zw * zw + (xw - as[0]) * (xw - as[0]));
	double ct3 = (d * d - as[1] * as[1] - as[2] * as[2]) / (2 * as[1] * as[2]);
	std::cout << " cos(theta3) = " << ct3 << std::endl;
	double st3;
	if (conf_t_3 == ALBOW_UP) {
		st3 = sqrt(1 - ct3 * ct3);
	}
	else {
		st3 = -sqrt(1 - ct3 * ct3);
	}
	double theta_3 = atan2(st3, ct3);
	double alpha = atan2(-zw, xw - as[0]);
	double beta = -atan2(as[2] * st3, as[1] + as[2] * ct3);
	double theta_2 = M_PI / 2 - alpha + beta;
	std::vector<double> theta23 = { theta_2, theta_3 };
	return theta23;
}
double Arm_kinematics::theta_4_s(Matrix<double, 3, 3> r, double phi, double theta_1, double theta_2, double theta_3){
    Matrix<double, 3, 1> z5 = get_z5(r, theta_1);
    double theta_4;
    if (z5(0,0) < 0) {
        theta_4 = M_PI / 2 - theta_2 - theta_3 - phi;
    } else {
        theta_4 = phi - M_PI / 2 - theta_2 - theta_3;
    }
    return theta_4;
}
double Arm_kinematics::theta_5_s(Matrix<double, 3, 3> R, double theta_1, double theta_2, double theta_3, double theta_4){
    double theta_5;
    Matrix<double, 4, 4> A1 = a_matrix_1(as[0], alphas[0], ds[0], theta_1, offsets[0]);
    Matrix<double, 4, 4> A2 = a_matrix_1(as[1], alphas[1], ds[1], theta_2, offsets[1]);
    Matrix<double, 4, 4> A3 = a_matrix_1(as[2], alphas[2], ds[2], theta_3, offsets[2]);
    Matrix<double, 4, 4> A4 = a_matrix_1(as[3], alphas[3], ds[3], theta_4, offsets[3]);
    Matrix<double, 4, 4> A14 = A1*A2*A3*A4;
	Matrix<double, 3, 3> R14;
	R14 << A14(0,0), A14(0,1), A14(0,2),
	       A14(1,0), A14(1,1), A14(1,2),
		   A14(2,0), A14(2,1), A14(2,2);
	Matrix<double, 3, 3> R14inv = R14.transpose();  
    Matrix<double, 3, 3> R5 = R14inv*R;
    theta_5 = atan2(R5(1, 0), R5(0, 0)) - M_PI / 2;
    if (abs(abs(theta_5) - M_PI) < 0.01) theta_5 = 0;
    return theta_5;
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