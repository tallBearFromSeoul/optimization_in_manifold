#include "cpp_utility.hpp"
#include "conversion.hpp"
#include "slerp.hpp"
#include "jacobians.hpp"
#include "se3.hpp"

int main(int argc, char *argv[]) {
	std::cout<<"Bazel build Successful\n";

	/*
	Mat93f J0;
	Mat43f J1;
	Mat3f R0;
	Mat39f J2;
	std::cout<<"k :\n"<<k<<"\n";
	jacob_exp_map_from_so3_to_SO3(J0);
	std::cout<<"J0 :\n"<<J0<<"\n";
	jacob_exp_map_from_so3_to_SU2(k, J1);
	std::cout<<"J1 :\n"<<J1<<"\n";
	
	std::cout<<euler_ang<<"\n";
	R_from_euler_ang(euler_ang, R0);
	std::cout<<"R0 :\n"<<R0<<"\n";
	jacob_log_map_from_SO3_to_so3(R0, J2);
	std::cout<<"J2 :\n"<<J2<<"\n";
	*/
	
	std::cout<<"\n\n";

	Vec3f k0;
	Quat q0, q1;
	std::vector<Quat> qs_interp;
	EulerAng euler_ang_init(0.f,0.f,0.f,true),
					 euler_ang0(0.f,0.f,30.f,true),
					 euler_ang1(0.f,0.f,0.f,true);
	q_from_euler_ang(euler_ang0, q0);
	q_from_euler_ang(euler_ang1, q1);
	slerp_q_direct(q0, q1, 4, qs_interp);
	print_vector(qs_interp);
	
	float incr = 0.1f;
	SE3_ptr p0 = std::make_shared<SE3>(euler_ang_init, Vec3f::Zero());
	log_map_from_SU2_to_so3(q0, k0);
	KinematicChain kinematic_chain(k0, p0);
	for (int i=1; i<qs_interp.size()-1; i++) {
		Vec3f k;
		log_map_from_SU2_to_so3(qs_interp[i], k);
		kinematic_chain.add_internal_twist(k, incr);
	}
	kinematic_chain.add_end_effector(incr);
	kinematic_chain.print_chain();
	kinematic_chain.csv_out_chain(0);
	
	return 0;
}
