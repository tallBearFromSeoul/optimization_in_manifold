#ifndef EXP_AND_LOG_MAP_HPP
#define EXP_AND_LOG_MAP_HPP
#include "utility.hpp"
#include <Eigen/Dense>

// R = I_3 + sin(theta)*K + (1-cos(theta))*K^2
// where the skew_symmetric and symmetric decomposition of R is :
// R = (R - R.T)/2 + (R + R.T)/2
// any square matrix can be written uniquely as a skew-symmetric matrix
// sin(theta)*K  : (R-R.T)/2 -> skew_symmetric
// I_3 + (1-cos(theta))*K^2 : (R+R.T)/2 -> symmetric
void exp_map_from_so3_to_SO3(const Vec3f &__k, Mat3f &R__) {
	Mat3f K;
	skew_symmetric(__k, K);
	float theta = __k.norm();
	// try to add the C++11 feature for backtracing of exception handling
	if (theta == 0.f)
		throw std::invalid_argument("ZeroDivisionError");
	R__ = Mat3f::Identity() + (sin(theta)/theta)*K + ((1.f-cos(theta))/(theta*theta)) * K*K;
}

void log_map_from_SO3_to_so3(const Mat3f &__R, Vec3f &k__) {
	float tr = __R.trace();
	// angle of rotation theta (rad)
	float theta = acos((tr-1.f)*0.5f);
	Mat3f R_log = (theta / (2.f*sin(theta))) * (__R - __R.transpose());
	// axis of rotation vector k
	vee_operator(R_log, k__);
}

void exp_map_from_so3_to_SU2(const Vec3f &__k, Quat &q__) {
	float theta = __k.norm();
	float theta_2 = theta / 2.f;
	float s_2 = sin(theta_2);
	if (theta == 0)
		throw std::invalid_argument("ZeroDivisionError");
	if (__k.sum() == 0) {
		q__ = Quat(1,0,0,0);
	} else {
		q__ = Quat(cos(theta_2), s_2*__k(0)/theta, s_2*__k(1)/theta, s_2*__k(2)/theta);
	}
}

void log_map_from_SU2_to_so3(const Quat &__q, Vec3f &k__) {
	float qr = __q.w();
	Vec3f qv = __q.vec();
	if (qv.sum() == 0)
		throw std::invalid_argument("ZeroDivisionError");
	k__ = 2*acos(qr)*qv/qv.norm();
}

#endif

