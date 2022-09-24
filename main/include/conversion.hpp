#ifndef CONVERSION_HPP
#define CONVERSION_HPP
#include "utility.hpp"

typedef Eigen::Vector3f Vec3f;
typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Quaternionf Quat;

void R_from_euler_ang(const EulerAng &__euler_ang, Mat3f &R__) {
	float roll = __euler_ang.r();
	float pitch = __euler_ang.p();
	float yaw = __euler_ang.y();
	float c_x = cos(roll);
	float c_y = cos(pitch);
	float c_z = cos(yaw);
	float s_x = sin(roll);
	float s_y = sin(pitch);
	float s_z = sin(yaw);
	Mat3f R_x {{1.f, 0.f, 0.f},
						 {0.f, c_x, -s_x},
						 {0.f, s_x, c_x}};
	Mat3f R_y {{c_y, 0, s_y},
						 {0.f, 1.f, 0.f},
						 {-s_y, 0.f, c_y}};
	Mat3f R_z {{c_z, -s_z, 0.f},
						 {s_z, c_z, 0.f},
						 {0.f, 0.f , 1.f}};
	R__ = R_z * R_y * R_x;
}

void q_from_euler_ang(const EulerAng &__euler_ang, Quat &q__) {
	float r_2, p_2, y_2, c_x, c_y, c_z, s_x, s_y, s_z;
	r_2 = __euler_ang.r()/2.f;
	p_2 = __euler_ang.p()/2.f;
	y_2 = __euler_ang.y()/2.f;
	c_x = cos(r_2);
	c_y = cos(p_2);
	c_z = cos(y_2);
	s_x = sin(r_2);
	s_y = sin(p_2);
	s_z = sin(y_2);
	q__ = Quat(c_x*c_y*c_z + s_x*s_y*s_z, s_x*c_y*c_z - c_x*s_y*s_z, c_x*s_y*c_z + s_x*c_y*s_z,  c_x*c_y*s_z - s_x*s_y*c_z);
}

void R_from_q(const Quat &__q, Mat3f &R__) {
	R__ << 1-2*(__q.y()*__q.y())-2*(__q.z()*__q.z()), 2*__q.x()*__q.y()-2*__q.w()*__q.z(), 2*__q.x()*__q.z() + 2*__q.w()*__q.y(),
  2*__q.x()*__q.y()+2*__q.w()*__q.z(), 1-2*(__q.x()*__q.x())-2*(__q.z()*__q.z()), 2*__q.y()*__q.z()-2*__q.w()*__q.x(),
  2*__q.x()*__q.z()-2*__q.w()*__q.y(), 2*__q.y()*__q.z()+2*__q.w()*__q.x(), 1-2*(__q.x()*__q.x())-2*(__q.y()*__q.y());
}

#endif
