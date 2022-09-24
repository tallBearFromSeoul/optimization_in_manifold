
/*
 * conversion.hpp
// NEED TO INCORPORATE QUADRANTS CHECKING AND EGDE CASE CHECKING
void euler_ang_from_R(const Mat3f &__R, Vec3f &euler_ang__) {
	euler_ang__(0) = atan2(__R(2,1), __R(2,2));
	euler_ang__(1) = atan2(-__R(2,0), sqrt(__R(2,1)*__R(2,1) + __R(2,2)*__R(2,2)));
	euler_ang__(2) = atan2(__R(1,0), __R(0,0));
}

// NEED TO INCORPORATE QUADRANTS CHECKING
void euler_ang_from_q(const Quat &__q, Vec3f &euler_ang__) {
	euler_ang__(0) = atan2(2*(__q.w()*__q.x() + __q.y()*__q.z()), 1-2*(__q.x()*__q.x() + __q.y()*__q.y()));
	euler_ang__(1) = asin(2*(__q.w()*__q.y() - __q.z()*__q.x()));
	euler_ang__(2) = atan2(2*(__q.w()*__q.z() + __q.x()*__q.y()), 1-2*(__q.y()*__q.y() + __q.z()*__q.z()));
}

*/


