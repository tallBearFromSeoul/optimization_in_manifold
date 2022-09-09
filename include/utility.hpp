#ifndef UTILITY_HPP
#define UTILITY_HPP
#include <Eigen/Dense>

typedef Eigen::Vector3f Vec3f;
typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Quaternionf Quat;

const float _PI = 3.14159265359f;
const float _PI_OVER_180 = 0.01745329251f;
const float _180_OVER_PI = 57.2957795131f;

inline Vec3f deg2rad(const Vec3f &__deg) {
	return {__deg(0)*_PI_OVER_180, __deg(1)*_PI_OVER_180, __deg(2)*_PI_OVER_180};
}

inline Vec3f rad2deg(const Vec3f &__rad) {
	return {__rad(0)*_180_OVER_PI, __rad(1)*_180_OVER_PI, __rad(2)*_180_OVER_PI};
}

inline void vee_operator(const Mat3f &__K, Vec3f &k__) {
	if (-__K != __K.transpose())
		throw std::invalid_argument("NotSymmetricMatrix");
	k__ << -__K(1,2), __K(0,2), -__K(0,1);
}

// also known as hat operator
inline void skew_symmetric(const Vec3f &__k, Mat3f &K__) {
	K__ << 0, -__k(2), __k(1),
				 __k(2), 0, -__k(0),
				-__k(1), __k(0), 0;
}

inline void hat_operator(const Vec3f &__k, Mat3f &K__) {
	skew_symmetric(__k, K__);
}

inline void quaternion_power(const Quat &__q, float __n, Quat &q__) {
	float theta = acos(__q.w());
	q__.w() = cos(__n*theta);
	q__.vec() = __q.vec().normalized()*sin(__n*theta);
}

#endif
