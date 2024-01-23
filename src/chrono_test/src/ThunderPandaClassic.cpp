#include "../library/ThunderPandaClassic.h"
//#include "../library/gen_regr_fun.h"
#include "../library/regr_fun_3R_classic.h"

namespace thunder_ns{
    
    thunderPandaClassic::thunderPandaClassic(){};

    thunderPandaClassic::thunderPandaClassic(const int nj_):num_joints(nj_){ 
        resizeVariables();
    }
   
    void thunderPandaClassic::init(const int nj_){
        num_joints = nj_;
        resizeVariables();
    }

    void thunderPandaClassic::resizeVariables(){
        
        q = Eigen::VectorXd::Zero(num_joints);
        dq = Eigen::VectorXd::Zero(num_joints);
        dqr = Eigen::VectorXd::Zero(num_joints);
        ddqr = Eigen::VectorXd::Zero(num_joints);
        param = Eigen::VectorXd::Zero(10*num_joints);

        reg_gen.resize(num_joints,10*num_joints);
        jac_gen.resize(6,num_joints);
        pinvJac_gen.resize(num_joints,6);
        pinvJacPos_gen.resize(num_joints,3);
        dotPinvJac_gen.resize(num_joints,6);
        dotPinvJacPos_gen.resize(num_joints,3);
        kin_gen.resize(4,4);
        mass_gen.resize(num_joints,num_joints);
        coriolis_gen.resize(num_joints,num_joints);
        gravity_gen.resize(num_joints,1);
    }
    
    void thunderPandaClassic::setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_,const Eigen::VectorXd& dqr_,const Eigen::VectorXd& ddqr_){
        if(q_.size() == num_joints && dq_.size()== num_joints && dqr_.size()==num_joints && ddqr_.size()==num_joints){
            q = q_;
            dq = dq_;
            dqr = dqr_;
            ddqr = ddqr_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        computeKin_gen();
        computeJac_gen();
        computeReg_gen();
        computeMass_gen();
        computeCoriolis_gen();
        computeGravity_gen();
    }
    
    void thunderPandaClassic::setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_,const Eigen::VectorXd& param_){
        if(q_.size() == num_joints && dq_.size()== num_joints && param_.size()== 10*num_joints){
            q = q_;
            dq = dq_;
            param = param_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        computeMass_gen();
        computeCoriolis_gen();
        computeGravity_gen();
    }

    void thunderPandaClassic::setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_){
        if(q_.size() == num_joints && dq_.size()== num_joints){
            q = q_;
            dq = dq_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        computeJac_gen();
        //computeDotJac_gen();
        computePinvJac_gen();
        computeDotPinvJac_gen();
        computePinvJacPos_gen();
        computeDotPinvJacPos_gen();
        computeKin_gen();
        //computeMass_gen();
        //computeCoriolis_gen();
        //computeGravity_gen();
    }
    
    void thunderPandaClassic::setArguments(const Eigen::VectorXd& q_){
        if(q_.size() == num_joints){
            q = q_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        computeJac_gen();
        //computePinvJac_gen();
        computeKin_gen();
        //computeMass_gen();
        //computeGravity_gen();
    }
    
    void thunderPandaClassic::setInertialParam(const Eigen::VectorXd& param_){
        if(param_.size() == 10*num_joints){
            param = param_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        computeMass_gen();
        computeCoriolis_gen();
        computeGravity_gen();
    }
    
    void thunderPandaClassic::computeReg_gen(){
        
        long long p3[regr_classic_fun_SZ_IW];
        double p4[regr_classic_fun_SZ_W];

        const double* input_[] = {q.data(), dq.data(), dqr.data(), ddqr.data()};
        double* output_[] = {reg_gen.data()};
        
        int check = regr_classic_fun(input_, output_, p3, p4, 0);
        
    }
    
    void thunderPandaClassic::computeMass_gen(){
        
        long long p3[mass_fun_SZ_IW];
        double p4[mass_fun_SZ_W];

        const double* input_[] = {q.data(), param.data()};
        double* output_[] = {mass_gen.data()};
        
        int check = mass_fun(input_, output_, p3, p4, 0);
        
    }
    
    void thunderPandaClassic::computeCoriolis_gen(){
        
        long long p3[coriolis_fun_SZ_IW];
        double p4[coriolis_fun_SZ_W];

        const double* input_[] = {q.data(), dq.data(), param.data()};
        double* output_[] = {coriolis_gen.data()};
        
        int check = coriolis_fun(input_, output_, p3, p4, 0);
    }

    void thunderPandaClassic::computeGravity_gen(){
        
        long long p3[gravity_fun_SZ_IW];
        double p4[gravity_fun_SZ_W];

        const double* input_[] = {q.data(), param.data()};
        double* output_[] = {gravity_gen.data()};
        int check = gravity_fun(input_, output_, p3, p4, 0);
    }

    void thunderPandaClassic::computeJac_gen(){
        
        long long p3[jac_fun_SZ_IW];
        double p4[jac_fun_SZ_W];

        const double* input_[] = {q.data()};
        double* output_[] = {jac_gen.data()};

        int check = jac_fun(input_, output_, p3, p4, 0);
    }

    void thunderPandaClassic::computeDotJac_gen(){
        
        long long p3[dotJac_fun_SZ_IW];
        double p4[dotJac_fun_SZ_W];

        const double* input_[] = {q.data(), dq.data()};
        double* output_[] = {dotJac_gen.data()};

        int check = dotJac_fun(input_, output_, p3, p4, 0);
    }

    void thunderPandaClassic::computePinvJac_gen(){
        
        long long p3[pinvJac_fun_SZ_IW];
        double p4[pinvJac_fun_SZ_W];

        const double* input_[] = {q.data()};
        double* output_[] = {pinvJac_gen.data()};

        int check = pinvJac_fun(input_, output_, p3, p4, 0);

    }
    
    void thunderPandaClassic::computePinvJacPos_gen(){
        
        long long p3[pinvJacPos_fun_SZ_IW];
        double p4[pinvJacPos_fun_SZ_W];

        const double* input_[] = {q.data()};
        double* output_[] = {pinvJacPos_gen.data()};

        int check = pinvJacPos_fun(input_, output_, p3, p4, 0);

    }

    void thunderPandaClassic::computeDotPinvJac_gen(){
        
        long long p3[dotPinvJac_fun_SZ_IW];
        double p4[dotPinvJac_fun_SZ_W];

        const double* input_[] = {q.data(),dq.data()};
        double* output_[] = {dotPinvJac_gen.data()};

        int check = dotPinvJac_fun(input_, output_, p3, p4, 0);

    }

    void thunderPandaClassic::computeDotPinvJacPos_gen(){
        
        long long p3[dotPinvJacPos_fun_SZ_IW];
        double p4[dotPinvJacPos_fun_SZ_W];

        const double* input_[] = {q.data(),dq.data()};
        double* output_[] = {dotPinvJacPos_gen.data()};

        int check = dotPinvJacPos_fun(input_, output_, p3, p4, 0);

    }

    void thunderPandaClassic::computeKin_gen(){
        
        long long p3[kin_fun_SZ_IW];
        double p4[kin_fun_SZ_W];

        const double* input_[] = {q.data()};
        double* output_[] = {kin_gen.data()};

        int check = kin_fun(input_, output_, p3, p4, 0);

    }


}