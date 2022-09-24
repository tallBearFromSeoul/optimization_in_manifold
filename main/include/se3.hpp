#ifndef SE3_HPP
#define SE3_HPP
#include "conversion.hpp"
#include "slerp.hpp"
#include <fstream>

typedef Eigen::RowVector3f RowVec3f;

class Rotation {
	private:

	protected:
		const bool _has_euler;
		EulerAng _euler;
		Mat3f _R;
		Quat _q;
		Vec3f _k;
		Vec3f _dir {1.f,0.f,0.f};

	public:
	Rotation(const EulerAng &__euler): _has_euler(true), _euler(__euler) {
		R_from_euler_ang(__euler, _R);
		q_from_euler_ang(__euler, _q);
		log_map_from_SU2_to_so3(_q, _k);
		rotate_v_with_k(_k, _dir);
	}
	Rotation(const Quat &__q) : _has_euler(false), _euler(EulerAng()), _q(__q) {
		log_map_from_SU2_to_so3(__q, _k);
		exp_map_from_so3_to_SO3(_k, _R);
		std::cout<<"R :\n"<<_R<<"\n";
		Mat3f R2;
		R_from_q(__q, R2);
		std::cout<<"R2 :\n"<<R2<<"\n";
		rotate_v_with_k(_k, _dir);
	}
	Rotation(const Vec3f &__k) : _has_euler(false), _euler(EulerAng()), _k(__k) {
		exp_map_from_so3_to_SO3(__k, _R);
		exp_map_from_so3_to_SU2(__k, _q);
		rotate_v_with_k(_k, _dir);
	}

	EulerAng euler() const {
		if (_has_euler)
			return _euler;
		else
			return EulerAng(0,0,0,false);
	}
	Mat3f R() const {return _R;}
	Quat q() const {return _q;}
	Vec3f k() const {return _k;}
	Vec3f dir() const {return _dir;}
};

class SE3;
typedef std::shared_ptr<SE3> SE3_ptr;
class SE3 : public Rotation{
	private:
		Vec3f _tran;
		Mat4f _T;
	protected:

	public:
		SE3(const EulerAng &__euler, const Vec3f &__tran) : Rotation(__euler), _tran(__tran) {
			_T.block(0,0,3,3) = _R;
			_T.block(0,3,3,1) = __tran;
			_T.block(3,0,1,3) = RowVec3f::Zero();
			_T(3,3) = 1.f;
		}
		SE3(const Quat &__q, const Vec3f &__tran) : Rotation(__q), _tran(__tran) {
			_T.block(0,0,3,3) = _R;
			_T.block(0,3,3,1) = __tran;
			_T.block(3,0,1,3) = RowVec3f::Zero();
			_T(3,3) = 1.f;
		}
		/*
		SE3(const Vec3f &__k) : Rotation(__k) {
			_tran = Vec3f::Zero();
			_T = Mat4f::Zero();
			_T.block(0,0,3,3) = _R;
			_T(3,3) = 1.f;
		}
		SE3(const Vec3f &__k, float __tx) : Rotation(__k) {
			_T = Mat4f::Zero();
			_T.block(0,0,3,3) = _R;
			_T(0,3) = __tx;
			_T(3,3) = 1.f;
		}
		*/
		SE3(const Vec3f &__k, const Vec3f &__tran) : Rotation(__k), _tran(__tran) {
			_T.block(0,0,3,3) = _R;
			_T.block(0,3,3,1) = __tran;
			_T.block(3,0,1,3) = RowVec3f::Zero();
			_T(3,3) = 1.f;
		}
		Rotation rot() const {return *this;}
		Vec3f tran() const {return _tran;}
		Mat4f T() const {return _T;}
		
		SE3_ptr twist_pose(const Vec3f &__k) {
			float a, b, c;
			Vec3f k2;
			std::cout<<"\ntwist pose :\n";
			std::cout<<"k0 :\n"<<_k<<"\n";
			std::cout<<"k1 :\n"<<__k<<"\n";
			a = _k.norm();
			b = __k.norm();
			c = acos(cos(a)*cos(b)-sin(a)*sin(b)*_k.dot(__k));
			k2 = (cos(a)*sin(b)*__k + cos(b)*sin(a)*_k + sin(a)*sin(b)*_k.cross(__k));
			std::cout<<"k2 :\n"<<k2<<"\n\n";
			return std::make_shared<SE3>(k2, _tran);
		}
		SE3_ptr tran_pose(float __tx) {
			Vec3f tran(__tx,0.f,0.f);
			rotate_v_with_k(_k, tran);
			tran += _tran;
			//project_v_to_k(_k, tran);
			return std::make_shared<SE3>(_k,tran);
		}

		SE3_ptr twist_and_tran_pose(const Vec3f &__k, float __tx) {
			float a, b, c;
			Vec3f k2, tran(__tx,0.f,0.f);
			rotate_v_with_k(_k, tran);
			std::cout<<"tran norm :\n"<<tran.norm()<<"\n";
			std::cout<<"tran prev :\n"<<tran<<"\n";
			tran += _tran;
			std::cout<<"tran af :\n"<<tran<<"\n";
	
			a = _k.norm();
			b = __k.norm();
			c = acos(cos(a)*cos(b)-sin(a)*sin(b)*_k.dot(__k));
			k2 = (cos(a)*sin(b)*__k + cos(b)*sin(a)*_k + sin(a)*sin(b)*_k.cross(__k));
			
			//project_v_to_k(_k, tran);
			return std::make_shared<SE3>(k2, tran);
		}
};

class KinematicChain {
	private:
		std::vector<Vec3f> _twists;
		std::vector<float> _trans;
		std::vector<SE3_ptr> _chain; 

	public:
		KinematicChain(const Vec3f &__base_twist, const SE3_ptr &__base_pose) {
			_twists.push_back(__base_twist);
			SE3_ptr se3 = __base_pose->twist_pose(__base_twist);
			_chain.push_back(se3);
			if(!_trans.size())
				_trans.clear();
		}
		void add_internal_twist(const Vec3f &__twist, float __tx) {
			_twists.push_back(__twist);
			_trans.push_back(__tx);
			std::cout<<"\nadd_internal_twist() :\n";
			std::cout<<"chain back pose :\n"<<_chain.back()->T()<<"\n";
			SE3_ptr se3 = _chain.back()->twist_and_tran_pose(__twist, __tx);
			std::cout<<"after twist_and_tran() :\n"<<se3->T()<<"\n";
			_chain.push_back(se3);
			assert(_twists.size() == _trans.size()+1);
		}
		void add_end_effector(float __tx) {
			_trans.push_back(__tx);
			SE3_ptr se3 = _chain.back()->tran_pose(__tx);
			_chain.push_back(se3);
			assert(_twists.size() == _trans.size());
		}

		std::vector<SE3_ptr> chain() const {return _chain;}
		void print_chain() const {
			std::cout<<"kinematic chain :\n";
			for (int i=0; i<_chain.size(); i++) {
				std::cout<<"pose #"<<i<<" :\n"<<_chain[i]->T()<<"\n";
			}
		}
		void csv_out_chain(int __id) const {
			const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",");
			std::ofstream csv("./csv_data/chain"+std::to_string(__id)+".csv");
			csv<<"index,axis_angle_vector,directional_vector,n_rows,n_cols,pose_matrix\n";
			for (int i=0; i<_chain.size(); i++) {
				Mat4f T = _chain[i]->T();
				Vec3f k = _chain[i]->k();
				Vec3f dir = _chain[i]->dir();
				csv<<i<<","<<k.format(CSVFormat)<<","<<dir.format(CSVFormat)<<",";
				csv<<T.rows()<<","<<T.cols()<<","<<T.format(CSVFormat)<<"\n";
			}
			csv.close();
		}
};

#endif
