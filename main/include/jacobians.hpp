#ifndef JACOBIANS_HPP
#define JACOBIANS_HPP
#include "utility.hpp"
#include <Eigen/Dense>

typedef Eigen::Matrix4f Mat4f;
typedef Eigen::Matrix<float, 9, 3> Mat93f;
typedef Eigen::Matrix<float, 4, 3> Mat43f;
typedef Eigen::Matrix<float, 3, 9> Mat39f;

void jacob_exp_map_from_so3_to_SO3(Mat93f &J__) {
	Vec3f e1, e2, e3;
	Mat3f E1, E2, E3;
	e1 << 1,0,0;
	e2 << 0,1,0;
	e3 << 0,0,1;
	hat_operator(e1, E1);
	hat_operator(e2, E2);
	hat_operator(e3, E3);
	J__.block(0,0,3,3) = E1;
	J__.block(3,0,3,3) = E2;
	J__.block(6,0,3,3) = E3;
}

void jacob_exp_map_from_so3_to_SU2(const Vec3f &__k, Mat43f &J__) {
	float theta, theta_2, s_2, c_2, s_2_theta, const_0;
	Mat4f d_exp_kq__d_wtheta;
	Mat43f d_wtheta__d_w;
	theta = __k.norm();
	theta_2 = theta / 2.f;
	s_2 = sin(theta_2);
	c_2 = cos(theta_2);
	s_2_theta = s_2 / theta;
	const_0 = ((c_2/(2*theta)) - s_2/(theta*theta));
	d_exp_kq__d_wtheta << 
		0,0,0,-s_2/2.f,
		s_2_theta,0,0,__k(0)*const_0,
		0,s_2_theta,0,__k(1)*const_0,
		0,0,s_2_theta,__k(2)*const_0;
	d_wtheta__d_w.block(0,0,3,3) = Mat3f::Identity();
	d_wtheta__d_w.row(3) = __k.normalized();
	J__ = d_exp_kq__d_wtheta * d_wtheta__d_w;
}

void jacob_log_map_from_SO3_to_so3(const Mat3f &__R, Mat39f &J__) {
	float tr, cos_theta, theta, sin_theta, b;
	Vec3f R_minus_RT_vee, a;
	tr = __R.trace();
	cos_theta = (tr - 1) / 2.f;
	if (cos_theta > 0.999999f) {
		J__ << 
			0.f,0.f,0.f,0.f,0.f,0.5f,0.f,-0.5f,0.f,
			0.f,0.f,-0.5f,0.f,0.f,0.f,0.5f,0.f,0.f,
			0.f,0.5f,0.f,-0.5f,0.f,0.f,0.f,0.f,0.f;
		return;
	}
	theta = acos(cos_theta);
	sin_theta = sqrt(1-(cos_theta*cos_theta));
	//(R-R.T)vee
	R_minus_RT_vee << 
		__R(2,1)-__R(1,2),
		__R(0,2)-__R(2,0),
		__R(1,0)-__R(0,1);
	a = R_minus_RT_vee * (theta*cos_theta - sin_theta) / (4*(sin_theta*sin_theta*sin_theta));
	b = theta / (2.f*sin_theta);
	J__ << 
		a(0),0.f,0.f, 0.f,a(0),b, 0.f,-b,a(0),
		a(1),0.f,-b, 0.f,a(1),0.f, b,0.f,a(1),
		a(2),b,0.f, -b,a(2),0.f, 0.f,0.f,a(2);
}

#endif
