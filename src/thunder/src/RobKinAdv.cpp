#include "../library/RobKinAdv.h"

/* Function name used to generate code */
#define DOT_JAC_STRING "dotJac_fun"
#define PINV_JAC_STRING "pinvJac_fun"
#define PINV_JAC_POS_STRING "pinvJacPos_fun"
#define DOT_PINV_JAC_STRING "dotPinvJac_fun"
#define DOT_PINV_JAC_POS_STRING "dotPinvJacPos_fun"

/* File name of generated code */
#define GENERATED_FILE "kinematics_fun.cpp"

/* Define number of function generable */
#define NUMBER_FUNCTIONS 7

namespace regrob{
    
    RobKinAdv::RobKinAdv(){};

    RobKinAdv::RobKinAdv(
        const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, FrameOffset& base_frame,FrameOffset& ee_frame, const double mu)
        : RobKinBasic(numJoints,jointsType,DHtable,base_frame,ee_frame), _mu_(mu){}
    
    RobKinAdv::RobKinAdv(
        const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, FrameOffset& base_frame, const double mu)
        : RobKinBasic(numJoints,jointsType,DHtable,base_frame), _mu_(mu){}

    void RobKinAdv::init(
        const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, FrameOffset& base_frame,FrameOffset& ee_frame, const double mu) {
        
        _numJoints_ = numJoints;
        _jointsType_ = jointsType;
        _DHtable_ = DHtable;
        _lab2L0_ = base_frame;
        _Ln2EE_ = ee_frame;

        _mu_ = mu;

        initVarsFuns();
        compute();
    }

    void RobKinAdv::init(
        const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, FrameOffset& base_frame, const double mu) {
        
        FrameOffset no_EE({0,0,0},{0,0,0});
        _numJoints_ = numJoints;
        _jointsType_ = jointsType;
        _DHtable_ = DHtable;
        _lab2L0_ = base_frame;
        _Ln2EE_= no_EE;

        _mu_ = mu;

        initVarsFuns();
        compute();
    }

    void RobKinAdv::initVarsFuns(){
        
        _q_ = casadi::SX::sym("_q_", _numJoints_,1);
        _dq_ = casadi::SX::sym("_dq_", _numJoints_,1);
        
        q = Eigen::VectorXd::Zero(_numJoints_,1);
        dq = Eigen::VectorXd::Zero(_numJoints_,1);
        
        args.resize(2);
        
        for(int i=0;i<2;i++){
            args[i].resize(_numJoints_,1);
        }
        
        kinematic_res.resize(1);
        jacobian_res.resize(1);
        dotJacobian_res.resize(1);
        pinvJacobian_res.resize(1);
        pinvJacobianPos_res.resize(1);
        dotPinvJacobian_res.resize(1);
        dotPinvJacobianPos_res.resize(1);

        init_casadi_functions();
    }

    void RobKinAdv::DHDotJac() {
        
        casadi::SX Jn = DHJacEE(); 
        casadi::SX dJn = casadi::SX::jtimes(Jn,_q_,_dq_);

        casadi::Function dotJac_fun(DOT_JAC_STRING,{_q_,_dq_},{densify(dJn)});

        dotJacobian_fun = dotJac_fun;
    }
    
    void RobKinAdv::DHPinvJac() {
        
        casadi::SX Jn = DHJacEE();
        
        casadi::SX invJn_dumped = casadi::SX::inv(casadi::SX::mtimes({Jn,Jn.T()}) + casadi::SX::eye(6)*_mu_);
        casadi::SX pinvJn = casadi::SX::mtimes({Jn.T(),invJn_dumped});      

        casadi::Function pinvJac_fun(PINV_JAC_STRING,{_q_},{densify(pinvJn)});

        pinvJacobian_fun = pinvJac_fun;
    }

    void RobKinAdv::DHPinvJacPos() {
        
        casadi::Slice rows_3(0, 3);      // select k versor of T()
        casadi::Slice all_c;      // select k versor of T()

        casadi::SX Jn = DHJacEE();
        casadi::SX Jn3 = Jn(rows_3,all_c);
        casadi::SX invJn_dumped = casadi::SX::inv(casadi::SX::mtimes({Jn3,Jn3.T()}) + casadi::SX::eye(3)*_mu_);
        casadi::SX pinvJn = casadi::SX::mtimes({Jn3.T(),invJn_dumped});      

        casadi::Function pinvJacPos_fun(PINV_JAC_POS_STRING,{_q_},{densify(pinvJn)});

        pinvJacobianPos_fun = pinvJacPos_fun;
    }
   
    void RobKinAdv::DHDotPinvJac() {
        
        casadi::SX Jn = DHJacEE();
        
        casadi::SX invJn_dumped = casadi::SX::inv(casadi::SX::mtimes({Jn,Jn.T()}) + casadi::SX::eye(6)*_mu_);
        casadi::SX pinvJn = casadi::SX::mtimes({Jn.T(),invJn_dumped});
        casadi::SX dot_pinvJn = casadi::SX::jtimes(pinvJn,_q_,_dq_);        

        casadi::Function dotPinvJac_fun(DOT_PINV_JAC_STRING,{_q_,_dq_},{densify(dot_pinvJn)});

        dotPinvJacobian_fun = dotPinvJac_fun;
    }

    void RobKinAdv::DHDotPinvJacPos() {
        
        casadi::Slice rows_3(0, 3);      // select k versor of T()
        casadi::Slice all_c;      // select k versor of T()

        casadi::SX Jn = DHJacEE();
        casadi::SX Jn3 = Jn(rows_3,all_c);
        casadi::SX invJn_dumped = casadi::SX::inv(casadi::SX::mtimes({Jn3,Jn3.T()}) + casadi::SX::eye(3)*_mu_);
        casadi::SX pinvJn = casadi::SX::mtimes({Jn3.T(),invJn_dumped});
        casadi::SX dot_pinvJn = casadi::SX::jtimes(pinvJn,_q_,_dq_);        

        casadi::Function dotPinvJac_fun(DOT_PINV_JAC_POS_STRING,{_q_,_dq_},{densify(dot_pinvJn)});

        dotPinvJacobianPos_fun = dotPinvJac_fun;
    }

    void RobKinAdv::init_casadi_functions() {
        
        DHKin();
        DHJac();
        DHDotJac();
        DHPinvJac();
        DHPinvJacPos();
        DHDotPinvJac();
        DHDotPinvJacPos();
    }
  
    void RobKinAdv::setArguments(const Eigen::VectorXd& q_){
        if(q_.size() == _numJoints_){
            q = q_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        compute();
    }

    void RobKinAdv::setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_){
        if(q_.size() == _numJoints_ && dq_.size()==_numJoints_){
            q = q_;
            dq = dq_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        compute();
    }

    void RobKinAdv::compute(){
        for(int i=0;i<_numJoints_;i++){
            args[0](i,0) = q(i);
            args[1](i,0) = dq(i);
        }

        kinematic_fun.call({args[0]},kinematic_res);
        jacobian_fun.call({args[0]},jacobian_res);
        dotJacobian_fun.call({args[0],args[1]},dotJacobian_res);
        pinvJacobian_fun.call({args[0]},pinvJacobian_res);
        pinvJacobianPos_fun.call({args[0]},pinvJacobianPos_res);
        dotPinvJacobian_fun.call({args[0],args[1]},dotPinvJacobian_res);
        dotPinvJacobianPos_fun.call({args[0],args[1]},dotPinvJacobianPos_res);       
    }

    Eigen::MatrixXd RobKinAdv::getDotJacobian(){
        const int nrow = 6;
        const int ncol = _numJoints_;
        Eigen::MatrixXd dotJacfull(nrow,ncol);
        std::vector<casadi::SXElem> dotjac_elements = dotJacobian_res[0].get_elements();
        std::transform(dotjac_elements.begin(), dotjac_elements.end(), dotJacfull.data(), mapFunction);
        
        return dotJacfull;
    }

    Eigen::MatrixXd RobKinAdv::getPinvJacobian(){
        const int nrow = _numJoints_;
        const int ncol = 6;
        Eigen::MatrixXd pinvJacfull(nrow,ncol);
        std::vector<casadi::SXElem> pinvjac_elements = pinvJacobian_res[0].get_elements();
        std::transform(pinvjac_elements.begin(), pinvjac_elements.end(), pinvJacfull.data(), mapFunction);
    
        return pinvJacfull;
    }
    
    Eigen::MatrixXd RobKinAdv::getPinvJacobianPos(){
        const int nrow = _numJoints_;
        const int ncol = 3;
        Eigen::MatrixXd pinvJacfull(nrow,ncol);
        std::vector<casadi::SXElem> pinvjac_elements = pinvJacobianPos_res[0].get_elements();
        std::transform(pinvjac_elements.begin(), pinvjac_elements.end(), pinvJacfull.data(), mapFunction);
    
        return pinvJacfull;
    }

    Eigen::MatrixXd RobKinAdv::getDotPinvJacobian(){
        const int nrow = _numJoints_;
        const int ncol = 6;
        Eigen::MatrixXd dotPinvJacfull(nrow,ncol);
        std::vector<casadi::SXElem> dotpinvjac_elements = dotPinvJacobian_res[0].get_elements();
        
        std::transform(dotpinvjac_elements.begin(), dotpinvjac_elements.end(), dotPinvJacfull.data(), mapFunction);
    
        return dotPinvJacfull;
    }

    Eigen::MatrixXd RobKinAdv::getDotPinvJacobianPos(){
        const int nrow = _numJoints_;
        const int ncol = 3;
        Eigen::MatrixXd dotPinvJacfull(nrow,ncol);
        std::vector<casadi::SXElem> dotpinvjac_elements = dotPinvJacobianPos_res[0].get_elements();
        
        std::transform(dotpinvjac_elements.begin(), dotpinvjac_elements.end(), dotPinvJacfull.data(), mapFunction);
    
        return dotPinvJacfull;
    }
 
    void RobKinAdv::generate_code(std::string& savePath){

        // Options for c-code auto generation
        casadi::Dict opts = casadi::Dict();
        opts["cpp"] = true;
        opts["with_header"] = true;
        
        // generate functions in c code
        casadi::CodeGenerator myCodeGen = casadi::CodeGenerator(GENERATED_FILE, opts);

        myCodeGen.add(kinematic_fun);
        myCodeGen.add(jacobian_fun);
        myCodeGen.add(dotJacobian_fun);
        myCodeGen.add(pinvJacobian_fun);
        myCodeGen.add(pinvJacobianPos_fun);
        myCodeGen.add(dotPinvJacobian_fun);
        myCodeGen.add(dotPinvJacobianPos_fun);

        myCodeGen.generate(savePath);
    }

    std::vector<std::string> RobKinAdv::getFunctionsName() {
        
        std::vector<std::string> orig_name_funs;
        std::vector<std::string> all_name_funs;
        int dim_orig_funs;

        orig_name_funs = RobKinBasic::getFunctionsName();

        dim_orig_funs = orig_name_funs.size();
        all_name_funs.resize(NUMBER_FUNCTIONS);

        for (int i=0; i<dim_orig_funs; i++){
            all_name_funs[i] = orig_name_funs[i];
        }
        all_name_funs[dim_orig_funs] = DOT_JAC_STRING; 
        all_name_funs[dim_orig_funs+1] = PINV_JAC_STRING;
        all_name_funs[dim_orig_funs+2] = PINV_JAC_POS_STRING;
        all_name_funs[dim_orig_funs+3] = DOT_PINV_JAC_STRING;
        all_name_funs[dim_orig_funs+4] = DOT_PINV_JAC_POS_STRING;

        return all_name_funs;
    }

    std::vector<casadi::Function> RobKinAdv::getCasadiFunctions() {
        
        std::vector<casadi::Function> casadi_funs;

        casadi_funs.resize(NUMBER_FUNCTIONS);

        casadi_funs[0] = kinematic_fun;
        casadi_funs[1] = jacobian_fun;
        casadi_funs[2] = dotJacobian_fun; 
        casadi_funs[3] = pinvJacobian_fun;
        casadi_funs[4] = pinvJacobianPos_fun;
        casadi_funs[5] = dotPinvJacobian_fun;
        casadi_funs[6] = dotPinvJacobianPos_fun;

        return casadi_funs;
    }

}