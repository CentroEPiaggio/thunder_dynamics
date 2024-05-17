#include "../library/FrameOffset.h"

namespace thunder_ns{
	
	FrameOffset::FrameOffset(){
		
		ypr.resize(3);
		translation.resize(3);
		gravity.resize(3);        
	}
	
	FrameOffset::FrameOffset(const std::vector<double>& ypr_,const std::vector<double>& pos_,const std::vector<double>& g_){
		
		ypr.resize(3);
		translation.resize(3);
		gravity.resize(3);  
		
		// check      
		if(ypr_.size()==3 && pos_.size()==3 && g_.size()==3){
			ypr = ypr_;
			translation = pos_;
			gravity = g_;              
		}else{
			throw std::runtime_error("in costructor FrameOffset: invalid dimension of arguments");
		}
	};
	
	FrameOffset::FrameOffset(const std::vector<double>& ypr_,const std::vector<double>& pos_){
		
		ypr.resize(3);
		translation.resize(3);
		gravity.resize(3);  
		
		// check      
		if(ypr_.size()==3 && pos_.size()==3){
			ypr = ypr_;
			translation = pos_;              
		}else{
			throw std::runtime_error("in costructor FrameOffset: invalid dimension of arguments");
		}
	};
	
	std::vector<double> FrameOffset::get_ypr(){return ypr;}
	
	casadi::SX FrameOffset::get_rotation(){
		
		casadi::SX R = casadi::SX::eye(3);
		double cy = cos(ypr[0]);
		double sy = sin(ypr[0]);
		double cp = cos(ypr[1]);
		double sp = sin(ypr[1]);
		double cr = cos(ypr[2]);
		double sr = sin(ypr[2]);

		//template rotation yaw-pitch-roll
		R(0,0)=cy*cp;
		R(0,1)=cy*sp*sr-sy*cr;
		R(0,2)=cy*sp*cr-sy*sr;
		R(1,0)=sy*cp;
		R(1,1)=sy*sp*sr+cy*cr;
		R(1,2)=sy*sp*cr-cy*sr;
		R(2,0)=-sp;
		R(2,1)=cp*sr;
		R(2,2)=cp*cr;

		return R;
	};
	
	casadi::SX FrameOffset::get_translation(){
		
		casadi::SX pos(3,1);

		pos(0,0) = translation[0];
		pos(1,0) = translation[1];
		pos(2,0) = translation[2];
		
		return pos;
	}
	
	casadi::SX FrameOffset::get_transform(){

		casadi::SX rotTr = casadi::SX::eye(4);
		
		double cy = cos(ypr[0]);
		double sy = sin(ypr[0]);
		double cp = cos(ypr[1]);
		double sp = sin(ypr[1]);
		double cr = cos(ypr[2]);
		double sr = sin(ypr[2]);
		double dx = translation[0];
		double dy = translation[1];
		double dz = translation[2];

		//template R yaw-pitch-roll
		rotTr(0,0)=cy*cp;
		rotTr(0,1)=cy*sp*sr-sy*cr;
		rotTr(0,2)=cy*sp*cr-sy*sr;
		rotTr(1,0)=sy*cp;
		rotTr(1,1)=sy*sp*sr+cy*cr;
		rotTr(1,2)=sy*sp*cr-cy*sr;
		rotTr(2,0)=-sp;
		rotTr(2,1)=cp*sr;
		rotTr(2,2)=cp*cr;
		rotTr(0,3) = dx;
		rotTr(1,3) = dy;
		rotTr(2,3) = dz;

		return rotTr;
	}
	
	casadi::SX FrameOffset::get_gravity(){
		
		casadi::SX SXg(3,1);
		SXg(0,0) = gravity[0];
		SXg(1,0) = gravity[1];
		SXg(2,0) = gravity[2];
		return SXg;
	}
	
	void FrameOffset::set_gravity(const std::vector<double>& g_){gravity = g_;}
	
	void FrameOffset::set_translation(const std::vector<double>& pos_){translation = pos_;}
	
	void FrameOffset::set_ypr(const std::vector<double>& ypr_){ypr = ypr_;}
	
	casadi::SX FrameOffset::hat(const std::vector<double>& v){
		
		// chech
		if(v.size() != 3 ){
			std::cout<<"in function hat of class FrameOffset invalid dimension of input"<<std::endl;
		}
		
		casadi::SX vhat(3,3);
		vhat(0,0) = 0;
		vhat(0,1) = -v[2];
		vhat(0,2) = v[1];
		vhat(1,0) = v[2];
		vhat(1,1) = 0;
		vhat(1,2) = -v[0];
		vhat(2,0) = -v[1];
		vhat(2,1) = v[0];
		vhat(2,2) = 0;

		return vhat;
	}

}