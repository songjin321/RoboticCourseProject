#include <cmath>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

#define  pi 3.14159265358f

using namespace std;



class kinematics
{
public:
	struct Theta { double theta1, theta2, theta3; };
	double r11, r12, r13;
	double r21, r22, r23;
	double r31, r32, r33;
	double t1, t2, t3;
	double nx, ox, ax, px;
	double ny, oy, ay, py;
	double nz, oz, az, pz;
	vector<Theta> theta;

	void Forward_Link_Kinematics(int links, double angle1 = 0, double angle2 = 0, double angle3 = 0);
	void Forward_Kinematics(double angle1, double angle2, double angle3,  Matrix4d &result_SE3);
	void Show_Forward_Result_RT(void);
	void Inverse_Kinematics(double theta_o, double px, double py, Theta thetaresult);
	void Show_Inverse_Result_Theta(Theta theta_result);
	double Form_postive_negative_180(double ang);
	void Form_result(Theta &theta_result);


	bool Check_valid(double theta_e, double theta4, double px, double py);
	bool Check_valid(double theta_e, double theta4, double px, double py, double &theta1_, double &theta2_, double &theta3_);
	bool Check_valid(const Matrix4d &Mat_SE3, double &theta1_, double &theta2_, double &theta3_, double &theta4_);
	static bool  Valid_Angle(double joint_pos1, double joint_pos2, double joint_pos3, double joint_pos4);//360
	int Valid_Space();
	void convert_SE3_tp(const Matrix4d &Mat_SE3, double &theta_e_, double &theta4_, double &px_, double & pz_);
};



void kinematics::Forward_Kinematics(double angle1, double angle2, double angle3, Matrix4d &result_SE3)
{
	r11 = cos(angle1 + angle2 + angle3);     r12 = -sin(angle1 + angle2 + angle3);       r13 = 0;
	r21 = sin(angle1 + angle2 + angle3);     r22 = cos(angle1 + angle2 + angle3);        r23 = 0;
	r31 = 0;                                 r32 = 0;                                    r33 = 1;
	t1 = (11.0*cos(angle1 + angle2)) + (17.9*cos(angle1))  + (1.3*cos(angle1 + angle2 + angle3 ) );
	t2 = (11.0*sin(angle1 + angle2)) + (17.9*sin(angle1))  + (1.3*sin(angle1 + angle2 + angle3 ) );
	t3 = 0.0;

	result_SE3(0, 0) = r11;		result_SE3(0, 1) = r12;		result_SE3(0, 2) = r13;		result_SE3(0, 3) = t1;
	result_SE3(1, 0) = r21;		result_SE3(1, 1) = r22;		result_SE3(1, 2) = r23;		result_SE3(1, 3) = t2;
	result_SE3(2, 0) = r31;		result_SE3(2, 1) = r32;		result_SE3(2, 2) = r33;		result_SE3(2, 3) = t3;
	result_SE3(3, 0) = 0.0;		result_SE3(3, 1) = 0.0;		result_SE3(3, 2) = 0.0;		result_SE3(3, 3) = 1.0;

}


void kinematics::Forward_Link_Kinematics(int links, double angle1, double angle2, double angle3)
{
	switch (links)
	{
	case 1:
		r11 = cos(angle1);           r12 = -sin(angle1);       r13 = 0.0;
		r21 = sin(angle1);           r22 = cos(angle1);        r23 = 0.0;
		r31 = 0.0;                   r32 = 0.0;                r33 = 1.0;
		t1 = (17.9*cos(angle1));     t2 = (17.9*sin(angle1));  t3 = 0.0;  //a_1= 17.9  a_2=11 a_3=1.3
		break;
	case 2:
		r11 = cos(angle1 + angle2);      r12 = -sin(angle1 + angle2);     r13 = 0.0;
		r21 = sin(angle1 + angle2);      r22 = cos(angle1 + angle2);      r23 = 0.0;
		r31 = 0.0;                       r32 = 0.0;                       r33 = 1.0;
		t1 = (11.0* cos(angle1 + angle2))  + (17.9*cos(angle1));
		t2 = (11.0* sin(angle1 + angle2))  + (17.9*sin(angle1));
		t3 = 0.0;
		break;
	case 3:
		r11 = cos(angle1 + angle2 + angle3);   r12 = -sin(angle1 + angle2 + angle3);   r13 = 0.0;
		r21 = sin(angle1 + angle2 + angle3);   r22 =  cos(angle1 + angle2 + angle3);   r23 = 0.0;
		r31 = 0.0;                             r32 =0.0;                               r33 = 1.0;
		t1 = (1.3*cos(angle1 + angle2 + angle3)) + (11.0*cos(angle1 + angle2))  + (17.9*cos(angle1)) ;
		t2 = (1.3*sin(angle1 + angle2 + angle3)) + (11.0*sin(angle1 + angle2))  + (17.9*sin(angle1)) ;
		t3 = 0.0;
		break;
	}
}


void kinematics::Show_Forward_Result_RT(void)
{
	printf("Forward_Result_RT:\n");
	printf("%10f %10f %10f %10f\n", r11, r12, r13, t1);
	printf("%10f %10f %10f %10f\n", r21, r22, r23, t2);
	printf("%10f %10f %10f %10f\n", r31, r32, r33, t3);
}


void kinematics::Inverse_Kinematics(double theta_o, double px, double py,Theta theta_result)
{
	theta.clear();
	double theta1, theta2, theta3;
    double a, b, c, an1, an2, m, n, p;
	double c1, s1, c2, s2,  k1;
	a = 17.9;    b = 11;    c = 1.3;

	an1 = c*cos(theta_o);
	an2 = c*sin(theta_o);
	n = px - an1;
	m = py - an2;
	p = n*n + m*m;

	c2 =( p - a*a - b*b) / (2 * a * b);
	k1 = 1.0 - c2*c2;
    s2 = sqrt(k1);
	theta2 = atan2(s2, c2);

	s1 = ((a + b*c2)*m - b*s2*n) / p;
	c1 = ((a + b*c2)*n + b*s2*m) / p;
	theta1 = atan(s1/c1);

	theta3 = theta_o  - theta2 - theta1 ;

	theta_result.theta1 = theta1;
	theta_result.theta2 = theta2;
	theta_result.theta3 = theta3;

	Matrix4d result_SE3;
	printf("\nInverse_Result:\n");
	printf("Rad: %10f %10f %10f \n", theta_result.theta1, theta_result.theta2, theta_result.theta3);
	printf("Ang: %10f %10f %10f \n", theta_result.theta1*180.0 / pi, theta_result.theta2*180.0 / pi, theta_result.theta3*180/pi);
	Forward_Kinematics(theta_result.theta1, theta_result.theta2, theta_result.theta3, result_SE3);
	Show_Forward_Result_RT();
}


int main()
{  
/*
	double angle1 = 10 * pi / 180;
	double angle2 = 45*pi/180;
	double angle3 = 20*pi/180;
	double theta_o = 75*pi/180;
	double px = 24.273864;
	double py = 13.374678;
	Matrix4d result_SE3;
	Matrix4d result_SE2;
	kinematics::Theta pn1;
   	Kinematics.Forward_Kinematics(angle1, angle2,  angle3, result_SE3);
   	Kinematics.Show_Forward_Result_RT();
*/
// input:theta_o, px, py output:pn1
	kinematics Kinematics;
  	Kinematics.Inverse_Kinematics(theta_o, px, py, pn1);
   	cout << result_SE3 << endl;
   getchar();
}

