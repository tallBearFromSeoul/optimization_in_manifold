#ifndef UTILITY_HPP
#define UTILITY_HPP
#include <Eigen/Dense>
#include <iostream>

typedef Eigen::Vector3f Vec3f;
typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Quaternionf Quat;

const float _PI = 3.14159265359f;
const float _PI_OVER_180 = 0.01745329251f;
const float _180_OVER_PI = 57.2957795131f;

inline float deg2rad(float __deg) {return __deg*_PI_OVER_180;}

inline float rad2deg(float __rad) {return __rad*_180_OVER_PI;}

struct EulerAng {
	float _r, _p, _y;
	float _r_deg, _p_deg, _y_deg;
	EulerAng() {}
	EulerAng(float __r, float __p, float __y, bool __is_deg) { 
		if (__is_deg) {
			_r_deg = __r;
			_p_deg = __p;
			_y_deg = __y;
			_r = deg2rad(__r);
			_p = deg2rad(__p);
			_y = deg2rad(__y);
		} else {
			_r = __r;
			_p = __p;
			_y = __y;
			_r_deg = rad2deg(__r);
			_p_deg = rad2deg(__p);
			_y_deg = rad2deg(__y);
		}
	} 
	float r() const {return _r;}
	float p() const {return _p;}
	float y() const {return _y;}
	float r_deg() const {return _r_deg;}
	float p_deg() const {return _p_deg;}
	float y_deg() const {return _y_deg;}
	friend std::ostream& operator<<(std::ostream& __os, const EulerAng &__euler_ang) {
		__os<<"euler angle in rad : "<<__euler_ang.r()<<" / "<< __euler_ang.p()<<" / "<<__euler_ang.y()<<"\n";
		__os<<"euler angle in deg : "<<__euler_ang.r_deg()<<" / "<<__euler_ang.p_deg()<<" / "<<__euler_ang.y_deg()<<"\n";
		return __os;
	}
};

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

inline void project_v_to_k(const Vec3f &__k, Vec3f &v__) {
	v__ = __k.dot(v__)*__k/(v__.dot(v__));
}

inline void rotate_v_with_k(const Vec3f &__k, Vec3f &v__) {
	float theta = __k.norm();
	Vec3f k_norm = __k.normalized();
	v__ = cos(theta)*v__ + sin(theta)*k_norm.cross(v__) + (1-cos(theta))*k_norm.dot(v__)*k_norm;
	/*
	std::cout<<"k :\n"<<__k<<"\n";
	std::cout<<"k norm :\n"<<k_norm<<"\n";
	std::cout<<"theta :\n"<<theta<<"\n";
	std::cout<<"theta_deg :\n"<<rad2deg(theta)<<"\n";
	std::cout<<"v_rot :\n"<<v__<<"\n";
	*/
}


#endif
