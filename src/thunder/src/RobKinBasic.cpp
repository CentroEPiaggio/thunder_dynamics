# include "../library/RobKinBasic.h"

/* Function name used to generate code */
#define KIN_STRING "kin_fun"
#define JAC_STRING "jac_fun"

/* File name of generated code */
#define GENERATED_FILE "kin_basic_fun.cpp"

/* Define number of function generable */
#define NUMBER_FUNCTIONS 2

namespace thunder_ns{
        
    RobKinBasic::RobKinBasic(){};

    RobKinBasic::RobKinBasic(
        const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, FrameOffset& base_frame,FrameOffset& ee_frame)
       :_numJoints_(numJoints), _jointsType_(jointsType), _DHtable_(DHtable), _lab2L0_(base_frame), _Ln2EE_(ee_frame){

        initVarsFuns();
        compute();
    }
    
    RobKinBasic::RobKinBasic(
        const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, FrameOffset& base_frame)
       :_numJoints_(numJoints), _jointsType_(jointsType), _DHtable_(DHtable), _lab2L0_(base_frame){

        FrameOffset no_EE({0,0,0},{0,0,0});
        _Ln2EE_= no_EE;
        initVarsFuns();
        compute();
    }

    void RobKinBasic::init(
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
 
     void RobKinBasic::init(
        const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, FrameOffset& base_frame,FrameOffset& ee_frame) {
        
        _numJoints_ = numJoints;
        _jointsType_ = jointsType;
        _DHtable_ = DHtable;
        _lab2L0_ = base_frame;
        _Ln2EE_ = ee_frame;

        initVarsFuns();
        compute();
    }

    void RobKinBasic::initVarsFuns(){
        
        if (_DHtable_.rows() != _numJoints_ || _DHtable_.cols() != 4){
            throw std::runtime_error("DHTemplate: Error size of DH table");
        }
        if ((int)_jointsType_.size()!=_numJoints_){
            throw std::runtime_error("DHFwkinJoints: Error size of joints string");
        }

        _q_ = casadi::SX::sym("_q_", _numJoints_,1);

        q = Eigen::VectorXd::Zero(_numJoints_,1);
        
        args.resize(1);    
        args[0].resize(_numJoints_,1);
    
        kinematic_res.resize(1);
        jacobian_res.resize(1);

        init_casadi_functions();
    }
  
    casadi::SX RobKinBasic::DHTemplate(const Eigen::MatrixXd& rowDHTable, const casadi::SX qi, char jointType) {
        
        double a;
        double alpha;
        casadi::SX d;
        casadi::SX theta;
        
        casadi::SX ct;      // cos(theta)
        casadi::SX st;      // sin(theta)
        double ca;          // cos(alpha)
        double sa;          // sin(alpha)
        
        casadi::SX Ti(4,4); // output
        
        // Check
        if (jointType == 'P') {
            d = rowDHTable(2) + qi;
            theta = rowDHTable(3);
        }
        else if (jointType == 'R') {
            d = rowDHTable(2);
            theta = rowDHTable(3) + qi;
        }
        else {
            throw std::runtime_error("DHTemplate: Error joint type");
        }

        a = rowDHTable(0);
        alpha = rowDHTable(1);
        
        ca = cos(alpha);
        sa = sin(alpha);
        ct = cos(theta);
        st = sin(theta);
        
        Ti(0,0) = ct;
        Ti(0,1) = -ca*st;
        Ti(0,2) = sa*st;
        Ti(0,3) = a*ct;
        Ti(1,0) = st;
        Ti(1,1) = ca*ct;
        Ti(1,2) = -sa*ct;
        Ti(1,3) = a*st;
        Ti(2,1) = sa;
        Ti(2,2) = ca;
        Ti(2,3) = d;
        Ti(3,3) = 1;

        return Ti;
    }
    
    std::tuple<casadi::SXVector,casadi::SXVector> RobKinBasic::DHFwKinJoints() {

        casadi::SXVector Ti(_numJoints_);    // Output
        casadi::SXVector T0i(_numJoints_);   // Output
       
        // Ti is transformation from link i-1 to link i
        Ti[0] = DHTemplate(_DHtable_.row(0), _q_(0), _jointsType_[0]);

        // Ti is transformation from link 0 to link i
        T0i[0] = Ti[0];

        for (int i = 1; i < _numJoints_; i++) {
            Ti[i] = DHTemplate(_DHtable_.row(i), _q_(i), _jointsType_[i]);
            T0i[i] = mtimes(T0i[i - 1],Ti[i]);
        }

        return std::make_tuple(T0i, Ti);
    }
 
    std::tuple<casadi::SXVector,casadi::SXVector> RobKinBasic::DHJacJoints(const casadi::SXVector& T0i_vec) {

        casadi::SX Ji_pos(3, _numJoints_);    // matrix of velocity jacobian
        casadi::SX Ji_or(3, _numJoints_);     // matrix of omega jacobian
        casadi::SXVector Ji_v(_numJoints_);   // vector of matrix Ji_v
        casadi::SXVector Ji_w(_numJoints_);   // vector of matrix Ji_w
        casadi::Slice r_tra_idx(0, 3);      // select translation vector of T()
        casadi::Slice r_rot_idx(0, 3);      // select k versor of T()
        casadi::Slice allRows;              // Select all rows

        for (int i = 0; i < _numJoints_; i++) {

            casadi::SX k0(3,1);             // versor of joint i
            casadi::SX O_0i(3,1);           // distance of joint i from joint 0
            casadi::SX T_0i(4,4);           // matrix tranformation of joint i from joint 0

            k0(2,0) = 1;
            T_0i = T0i_vec[i];
            O_0i = T_0i(r_tra_idx, 3);

            // First column of jacobian
            if (_jointsType_[0] == 'P') {
                Ji_pos(allRows,0) = k0;
            } else if (_jointsType_[0] == 'R') {
                Ji_pos(allRows,0) = mtimes(hat(k0),O_0i);
                Ji_or(allRows,0) = k0;
            } else {
                throw std::runtime_error("DHJac: Error joint type");
            }

            // Rest of columns of jacobian
            for (int j = 1; j <= i; j++) {

                // Init variables of column j-th of jacobian each cycle
                casadi::SX kj_1(3,1);             // versor of joint j-1
                casadi::SX O_j_1i(3,1);           // distance of joint i from joint j-1
                casadi::SX T_0j_1(4,4);           // matrix tranformation of joint i from joint j-1
        
                T_0j_1 = T0i_vec[j-1];
                kj_1 = T_0j_1(r_rot_idx, 2);
                O_j_1i = O_0i - T_0j_1(r_tra_idx, 3);

                if (_jointsType_[j] == 'P') {
                    Ji_pos(allRows, j) = kj_1;
                } else if (_jointsType_[j] == 'R') {
                    Ji_pos(allRows, j) = mtimes(hat(kj_1),O_j_1i);
                    Ji_or(allRows, j) = kj_1;
                } else {
                    throw std::runtime_error("DHJac: Error joint type");
                }
            }
            
            // Add end-effector transformation
            if(i==(_numJoints_-1)){
                casadi::SX R0i = T_0i(r_rot_idx,r_rot_idx);
                Ji_pos = Ji_pos - casadi::SX::mtimes({R0i,hat(_Ln2EE_.get_translation()),R0i.T(),Ji_or});
            } 
            
            // Add offset from world-frame transformation
            Ji_pos = mtimes(_lab2L0_.get_rotation(),Ji_pos);
            Ji_or = mtimes(_lab2L0_.get_rotation(),Ji_or);
            
            Ji_v[i] = Ji_pos;
            Ji_w[i] = Ji_or;
        }

        return std::make_tuple(Ji_v, Ji_w);
    }
    
    casadi::SX RobKinBasic::hat(const casadi::SX& v) {
        
        casadi::SX skew(3, 3);
        
        skew(0, 1) = -v(2);
        skew(0, 2) = v(1);
        skew(1, 0) = v(2);
        skew(1, 2) = -v(0);
        skew(2, 0) = -v(1);
        skew(2, 1) = v(0);
        
        return skew;
    }

    casadi::SX RobKinBasic::DHJacEE() {

        const int nj = _numJoints_;
        
        casadi::SXVector T0i(nj);
        std::tuple<casadi::SXVector, casadi::SXVector> T_tuple;

        casadi::SXVector Jvi(nj);
        casadi::SXVector Jwi(nj);
        std::tuple<casadi::SXVector, casadi::SXVector> J_tuple;

        casadi::SX Jn_pos(3,nj);
        casadi::SX Jn_or(3,nj);
        casadi::SX Jn(6,nj);

        T_tuple = DHFwKinJoints();
        T0i = std::get<0>(T_tuple);

        J_tuple = DHJacJoints(T0i);
        Jvi = std::get<0>(J_tuple);
        Jwi = std::get<1>(J_tuple);

        Jn_pos = Jvi[nj-1];
        Jn_or = Jwi[nj-1];
        Jn = casadi::SX::vertcat({Jn_pos,Jn_or});

        return Jn;
    }
    
    casadi::SX RobKinBasic::DHFwKinEE() {

        const int nj = _numJoints_;

        casadi::SXVector T0i(nj);
        std::tuple<casadi::SXVector, casadi::SXVector> T_tuple;
        casadi::SX T0EE = casadi::SX::eye(4);
        
        T_tuple = DHFwKinJoints();
        T0i = std::get<0>(T_tuple);
        T0EE = casadi::SX::mtimes({_lab2L0_.get_transform(),T0i[nj-1],_Ln2EE_.get_transform()});
        
        return T0EE;
    }
    
    void RobKinBasic::DHKin() {
        
        casadi::SX T0EE = DHFwKinEE(); 

        casadi::Function kin_fun(KIN_STRING,{_q_},{densify(T0EE)});
        kinematic_fun = kin_fun;
    }
    
    void RobKinBasic::DHJac() {
        
        casadi::SX Jn = DHJacEE(); 

        casadi::Function jac_fun(JAC_STRING,{_q_},{densify(Jn)});

        jacobian_fun = jac_fun;
    }
    
    void RobKinBasic::init_casadi_functions() {
        
        DHKin();
        DHJac();
    }
  
    void RobKinBasic::setArguments(const Eigen::VectorXd& q_){
        if(q_.size() == _numJoints_){
            q = q_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        compute();
    }

    void RobKinBasic::compute(){
        for(int i=0;i<_numJoints_;i++){
            args[0](i,0) = q(i);
        }
        kinematic_fun.call({args[0]},kinematic_res);
        jacobian_fun.call({args[0]},jacobian_res);       
    }
        
    Eigen::Matrix4d RobKinBasic::getKinematic(){

        Eigen::Matrix4d Kinfull;
        std::vector<casadi::SXElem> kin_elements = kinematic_res[0].get_elements();
        std::transform(kin_elements.begin(), kin_elements.end(), Kinfull.data(), mapFunction);
        
        return Kinfull;
    }
    
    Eigen::MatrixXd RobKinBasic::getJacobian(){

        Eigen::MatrixXd Jacfull(6,_numJoints_);
        std::vector<casadi::SXElem> jac_elements = jacobian_res[0].get_elements();
        std::transform(jac_elements.begin(), jac_elements.end(), Jacfull.data(), mapFunction);

        return Jacfull;
    }
    
    void RobKinBasic::generate_code(std::string& savePath){
        
        // Options for c-code auto generation
        casadi::Dict opts = casadi::Dict();
        opts["cpp"] = true;
        opts["with_header"] = true;
        
        // generate functions in c code
        casadi::CodeGenerator myCodeGen = casadi::CodeGenerator(GENERATED_FILE, opts);

        myCodeGen.add(kinematic_fun);
        myCodeGen.add(jacobian_fun);

        myCodeGen.generate(savePath);
    }

    std::vector<std::string> RobKinBasic::getFunctionsName() {
        std::vector<std::string> name_funs;
        name_funs.resize(NUMBER_FUNCTIONS);

        name_funs[0] = KIN_STRING;
        name_funs[1] = JAC_STRING;

        return name_funs;
    }

    std::vector<casadi::Function> RobKinBasic::getCasadiFunctions() {
        std::vector<casadi::Function> casadi_funs;
        casadi_funs.resize(NUMBER_FUNCTIONS);

        casadi_funs[0] = kinematic_fun;
        casadi_funs[1] = jacobian_fun;

        return casadi_funs;
    }

}