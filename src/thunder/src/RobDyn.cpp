#include "../library/RobDyn.h"

/* Function name used to generate code */
#define MASS_STRING "mass_fun"
#define CORIOLIS_STRING "coriolis_fun"
#define GRAVITY_STRING "gravity_fun"

/* File name of generated code */
#define GENERATED_FILE "dynamics_fun.cpp"

/* Define number of function generable */
#define NUMBER_FUNCTIONS 5

namespace thunder_ns{
    
    RobDyn::RobDyn(){};

    RobDyn::RobDyn(
        const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, FrameOffset& base_frame,FrameOffset& ee_frame)
        : RobKinBasic(numJoints,jointsType,DHtable,base_frame,ee_frame){}
    
    RobDyn::RobDyn(
        const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, FrameOffset& base_frame)
        : RobKinBasic(numJoints,jointsType,DHtable,base_frame){}
    
    void RobDyn::init(
        const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, FrameOffset& base_frame,FrameOffset& ee_frame) {
        
        _numJoints_ = numJoints;
        _jointsType_ = jointsType;
        _DHtable_ = DHtable;
        _lab2L0_ = base_frame;
        _Ln2EE_ = ee_frame;

        initVarsFuns();
        compute();
    }
    
    void RobDyn::init(
        const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, FrameOffset& base_frame) {
        
        FrameOffset no_EE({0,0,0},{0,0,0});
        _numJoints_ = numJoints;
        _jointsType_ = jointsType;
        _DHtable_ = DHtable;
        _lab2L0_ = base_frame;
        _Ln2EE_= no_EE;

        initVarsFuns();
        compute();
    }

    void RobDyn::initVarsFuns(){
        
        _q_ = casadi::SX::sym("_q_", _numJoints_,1);
        _dq_ = casadi::SX::sym("_dq_", _numJoints_,1);
        _param_ = casadi::SX::sym("_param_", 10*_numJoints_,1);

        q = Eigen::VectorXd::Zero(_numJoints_);
        dq = Eigen::VectorXd::Zero(_numJoints_);
        param = Eigen::VectorXd::Zero(10*_numJoints_);

        _mass_vec_.resize(_numJoints_);
        _distCM_.resize(_numJoints_);
        _J_3x3_.resize(_numJoints_);

        for (int i=0; i<_numJoints_;i++){
            _distCM_[i] = casadi::SX::zeros(3,1);
            _J_3x3_[i] = casadi::SX::zeros(3,3);
        }

        args.resize(3);
        for(int i=0;i<2;i++){
            args[i].resize(_numJoints_,1);
        }
        args[2].resize(10*_numJoints_,1);

        kinematic_res.resize(1);
        jacobian_res.resize(1);
        mass_res.resize(1);
        coriolis_res.resize(1);
        gravity_res.resize(1);

        setInertialParameters();
        init_casadi_functions();
    }

    void RobDyn::setInertialParameters(){
        
        casadi::SX tempI(3,3);

        for(int i=0; i<_numJoints_; i++){
            
            _mass_vec_[i] = _param_(i*10,0);
            for(int j=0; j<3; j++){
                _distCM_[i](j,0) = _param_(i*10+j+1,0);
            }

            tempI(0,0) = _param_(i*10+4,0);
            tempI(0,1) = _param_(i*10+5,0);
            tempI(0,2) = _param_(i*10+6,0);
            tempI(1,0) = tempI(0,1);
            tempI(1,1) = _param_(i*10+7,0);
            tempI(1,2) = _param_(i*10+8,0);
            tempI(2,0) = tempI(0,2);
            tempI(2,1) = tempI(1,2);
            tempI(2,2) = _param_(i*10+9,0);

            _J_3x3_[i] = tempI;
        }
    }

    std::tuple<casadi::SXVector,casadi::SXVector> RobDyn::DHJacCM(const casadi::SXVector& T0i_vec) {

        casadi::SX Jci_pos(3, _numJoints_); // matrix of velocity jacobian
        casadi::SX Ji_or(3, _numJoints_);   // matrix of omega jacobian
        casadi::SXVector Ji_v(_numJoints_); // vector of matrix Ji_v
        casadi::SXVector Ji_w(_numJoints_); // vector of matrix Ji_w
        casadi::Slice r_tra_idx(0, 3);      // select translation vector of T()
        casadi::Slice r_rot_idx(0, 3);      // select k versor of T()
        casadi::Slice allRows;              // Select all rows

        for (int i = 0; i < _numJoints_; i++) {

            casadi::SX k0(3,1);             // versor of joint i
            casadi::SX O_0i(3,1);           // distance of joint i from joint 0
            casadi::SX T_0i(4,4);           // matrix tranformation of joint i from joint 0
            
            casadi::SX O_Ci(3,1);
            casadi::SX R0i;
            
            k0(2,0) = 1;
            T_0i = T0i_vec[i];
            O_0i = T_0i(r_tra_idx, 3);
            
            R0i = T_0i(r_rot_idx,r_rot_idx);
            O_Ci = O_0i + mtimes(R0i,_distCM_[i]);
            
            // First column of jacobian
            if (_jointsType_[0] == 'P') {
                Jci_pos(allRows,0) = k0;
            } else if (_jointsType_[0] == 'R') {
                Jci_pos(allRows,0) = mtimes(hat(k0),O_Ci);
                Ji_or(allRows,0) = k0;
            } else {
                throw std::runtime_error("DHJac: Error joint type");
            }

            // Rest of columns of jacobian
            for (int j = 1; j <= i; j++) {
                // Init variables of column j-th of jacobian each cycle
                casadi::SX kj_1(3,1);             // versor of joint j-1
                casadi::SX O_j_1Ci(3,1);           // distance of joint i from joint j-1
                casadi::SX T_0j_1(4,4);           // matrix tranformation of joint i from joint j-1

                T_0j_1 = T0i_vec[j-1];
                kj_1 = T_0j_1(r_rot_idx, 2);
                O_j_1Ci = O_Ci - T_0j_1(r_tra_idx, 3);

                if (_jointsType_[j] == 'P') {
                    Jci_pos(allRows, j) = kj_1;
                } else if (_jointsType_[j] == 'R') {
                    Jci_pos(allRows, j) = mtimes(hat(kj_1),O_j_1Ci);
                    Ji_or(allRows, j) = kj_1;
                } else {
                    throw std::runtime_error("DHJac: Error joint type");
                }
            }

            // Add offset from world-frame transformation
            Jci_pos = mtimes(_lab2L0_.get_rotation(),Jci_pos);
            Ji_or = mtimes(_lab2L0_.get_rotation(),Ji_or);
            
            Ji_v[i] = Jci_pos;
            Ji_w[i] = Ji_or;
        }

        return std::make_tuple(Ji_v, Ji_w);
    }
    
    casadi::SX RobDyn::dq_select(const casadi::SX& dq_) {
        
        int n = dq_.size1();
        
        casadi::Slice allRows;
        casadi::SX mat_dq = casadi::SX::zeros(n, n * n);
        for (int i = 0; i < n; i++) {
            casadi::Slice sel_col(i*n,(i+1)*n);   // Select columns
            mat_dq(allRows, sel_col) = casadi::SX::eye(n)*dq_(i);
        }
        
        return mat_dq;
    }
    
    casadi::SX RobDyn::stdCmatrix(const casadi::SX& B, const casadi::SX& q_, const casadi::SX& dq_, const casadi::SX& dq_sel_) {
        
        int n = q_.size1();

        casadi::SX jac_B = jacobian(B,q_);
        
        casadi::SX C123 = reshape(mtimes(jac_B,dq_),n,n);
        casadi::SX C132 = mtimes(dq_sel_,jac_B);
        casadi::SX C231 = C132.T();

        casadi::SX C(n,n);
        C = (C123 + C132 - C231)/2;


/*         casadi::SX C1(n, n);
        casadi::SX C2(n, n);
        casadi::SX C3(n, n);

        for (int h = 0; h < n; h++) {
            for (int j = 0; j < n; j++) {
                for (int k = 0; k < n; k++) {
                    casadi::SX dbhj_dqk = jacobian(B(h, j), q_(k));
                    casadi::SX dbhk_dqj = jacobian(B(h, k), q_(j));
                    casadi::SX dbjk_dqh = jacobian(B(j, k), q_(h));
                    C1(h, j) = C1(h, j) + 0.5 * (dbhj_dqk) * dq_(k);
                    C2(h, j) = C2(h, j) + 0.5 * (dbhk_dqj) * dq_(k);
                    C3(h, j) = C3(h, j) + 0.5 * (- dbjk_dqh) * dq_(k);
                }
            }
        } */

        return C;
    }
    
    casadi::SXVector RobDyn::Dynamic(
        const casadi::SX& q_, const casadi::SX& dq_, 
        const std::string jointsType_, const Eigen::MatrixXd& DHtable_, FrameOffset& base_frame){
        
        casadi::SX dq_sel_ = dq_select(dq_);

        const int nj = q_.size1();
        
        casadi::SXVector Ti(nj);
        casadi::SXVector T0i(nj);
        std::tuple<casadi::SXVector, casadi::SXVector> T_tuple;

        casadi::SXVector Jci(nj);
        casadi::SXVector Jwi(nj);
        std::tuple<casadi::SXVector, casadi::SXVector> J_tuple;

        casadi::SX g = base_frame.get_gravity();

        casadi::SX M(nj,nj);
        casadi::SX C(nj,nj);
        casadi::SX G(nj,1);
        
        casadi::SX Mi(nj,nj);
        casadi::SX Gi(1,nj);
        casadi::SX mi(1,1);
        casadi::SX Ii(3,3);
        casadi::Slice selR(0,3);

        T_tuple = DHFwKinJoints();
        T0i = std::get<0>(T_tuple);
        //Ti  = std::get<1>(T_tuple);

        J_tuple = DHJacCM(T0i);
        Jci = std::get<0>(J_tuple);
        Jwi = std::get<1>(J_tuple);
        
        for (int i=0; i<nj; i++) {
            
            casadi::SX R0i = T0i[i](selR,selR);

            mi = _mass_vec_[i];
            Ii = _J_3x3_[i];
            Mi = mi * casadi::SX::mtimes({Jci[i].T(), Jci[i]}) + casadi::SX::mtimes({Jwi[i].T(),R0i,Ii,R0i.T(),Jwi[i]});
            M = M + Mi;

            Gi = -mi * casadi::SX::mtimes({g.T(),Jci[i]});
            G = G + Gi.T();
        }
        
        C = stdCmatrix(M,q_,dq_,dq_sel_);

        return {M,C,G};
    }
    
    void RobDyn::init_casadi_functions(){
        DHKin();
        DHJac();
        mass_coriolis_gravity();
    }

    void RobDyn::mass_coriolis_gravity(){
        
        casadi::SXVector result;
        
        result = Dynamic(_q_, _dq_, _jointsType_, _DHtable_, _lab2L0_);
        
        SX_mass = result[0];
        SX_coriolis = result[1];
        SX_gravity = result[2];

        mass_fun = casadi::Function(MASS_STRING, {_q_, _param_}, {densify(SX_mass)});
        coriolis_fun = casadi::Function(CORIOLIS_STRING, {_q_, _dq_, _param_}, {densify(SX_coriolis)});
        gravity_fun = casadi::Function(GRAVITY_STRING, {_q_, _param_}, {densify(SX_gravity)});
        
    }
    
    void RobDyn::setArguments(const Eigen::VectorXd& q_){
        if(q_.size() == _numJoints_){
            q = q_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        compute();
    }

    void RobDyn::setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_){
        if(q_.size() == _numJoints_ && dq_.size()==_numJoints_){
            q = q_;
            dq = dq_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        compute();
    }

    void RobDyn::setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_, const Eigen::VectorXd& param_){
        if(q_.size() == _numJoints_ && dq_.size()==_numJoints_ && param_.size() == 10*_numJoints_){
            q = q_;
            dq = dq_;
            param = param_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        compute();
    }

    void RobDyn::compute(){

        for(int i=0;i<_numJoints_;i++){
            args[0](i,0) = q(i);
            args[1](i,0) = dq(i);
        }

        for(int i=0;i<10*_numJoints_;i++){
            args[2](i,0) = param(i);
        }

        kinematic_fun.call({args[0]},kinematic_res);
        jacobian_fun.call({args[0]},jacobian_res);
        mass_fun.call({args[0],args[2]},mass_res);
        coriolis_fun.call({args[0],args[1],args[2]},coriolis_res);
        gravity_fun.call({args[0],args[2]},gravity_res);
    }
 
    Eigen::MatrixXd RobDyn::getMass(){
        
        Eigen::MatrixXd Mfull(_numJoints_,_numJoints_);
        std::vector<casadi::SXElem> m_elements = mass_res[0].get_elements();
        std::transform(m_elements.begin(), m_elements.end(), Mfull.data(), mapFunction);
        
        return Mfull;
    }

    Eigen::MatrixXd RobDyn::getCoriolis(){
        
        Eigen::MatrixXd Cfull(_numJoints_,_numJoints_);
        std::vector<casadi::SXElem> c_elements = coriolis_res[0].get_elements();
        std::transform(c_elements.begin(), c_elements.end(), Cfull.data(), mapFunction);
        
        return Cfull;
    }

    Eigen::MatrixXd RobDyn::getGravity(){
        
        Eigen::MatrixXd Gfull(_numJoints_,1);
        std::vector<casadi::SXElem> g_elements = gravity_res[0].get_elements();
        std::transform(g_elements.begin(), g_elements.end(), Gfull.data(), mapFunction);
        
        return Gfull;
    }

    void RobDyn::generate_code(std::string& savePath){
        
        // Options for c-code auto generation
        casadi::Dict opts = casadi::Dict();
        opts["cpp"] = true;
        opts["with_header"] = true;
        
        // generate functions in c code
        casadi::CodeGenerator myCodeGen = casadi::CodeGenerator(GENERATED_FILE, opts);

        myCodeGen.add(mass_fun);
        myCodeGen.add(coriolis_fun);
        myCodeGen.add(gravity_fun);

        myCodeGen.generate(savePath);
    }

    std::vector<std::string> RobDyn::getFunctionsName() {

        std::vector<std::string> orig_name_funs;
        std::vector<std::string> all_name_funs;
        int dim_orig_funs;

        orig_name_funs = RobKinBasic::getFunctionsName();

        dim_orig_funs = orig_name_funs.size();
        //std::cout<<"dim: "<<dim_orig_funs<<std::endl;
        all_name_funs.resize(NUMBER_FUNCTIONS);

        for (int i=0; i<dim_orig_funs; i++){
            all_name_funs[i] = orig_name_funs[i];
        }        
        
        all_name_funs[dim_orig_funs+1] = MASS_STRING;
        all_name_funs[dim_orig_funs+2] = CORIOLIS_STRING;
        all_name_funs[dim_orig_funs+3] = GRAVITY_STRING;
        
        return all_name_funs;
    }

    std::vector<casadi::Function> RobDyn::getCasadiFunctions() {
        std::vector<casadi::Function> casadi_funs;
        casadi_funs.resize(NUMBER_FUNCTIONS);

        casadi_funs[0] = kinematic_fun;
        casadi_funs[1] = jacobian_fun;
        casadi_funs[2] = mass_fun;
        casadi_funs[3] = coriolis_fun;
        casadi_funs[4] = gravity_fun;

        return casadi_funs;
    }

}