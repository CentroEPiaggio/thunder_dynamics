#include "../library/RobReg_Classic.h"

/* Function name used to generate code */
#define REG_STRING "regr_classic_fun"

/* File name of generated code */
#define GENERATED_FILE "regressor_classic_fun.cpp"

/* Define number of function generable */
#define NUMBER_FUNCTIONS 3

namespace thunder_ns{
    
    RobReg_Classic::RobReg_Classic(){};

    RobReg_Classic::RobReg_Classic(
        const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, 
        FrameOffset& base_frame,FrameOffset& ee_frame)
        : RobKinBasic(numJoints,jointsType,DHtable,base_frame,ee_frame){}
    
    RobReg_Classic::RobReg_Classic(
        const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, FrameOffset& base_frame)
        : RobKinBasic(numJoints,jointsType,DHtable,base_frame){}
    
    void RobReg_Classic::init(
        const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, 
        FrameOffset& base_frame,FrameOffset& ee_frame) {

        _numJoints_ = numJoints;
        _jointsType_ = jointsType;
        _DHtable_ = DHtable;
        _lab2L0_ = base_frame;
        _Ln2EE_ = ee_frame;
    
        initVarsFuns();
        compute();
    }

    void RobReg_Classic::init(
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
    
    void RobReg_Classic::initVarsFuns(){
        
        _q_ = casadi::SX::sym("_q_", _numJoints_,1);
        _dq_ = casadi::SX::sym("_dq_", _numJoints_,1);
        _dqr_ = casadi::SX::sym("_ddqr_", _numJoints_,1);
        _ddqr_ = casadi::SX::sym("_ddqr_", _numJoints_,1);
        
        q = Eigen::VectorXd::Zero(_numJoints_);
        dq = Eigen::VectorXd::Zero(_numJoints_);
        dqr = Eigen::VectorXd::Zero(_numJoints_);
        ddqr = Eigen::VectorXd::Zero(_numJoints_);
        
        args.resize(4);
        
        for(int i=0;i<4;i++){
            args[i].resize(_numJoints_,1);
        }
        
        kinematic_res.resize(1);
        jacobian_res.resize(1);
        regressor_res.resize(1);

        init_casadi_functions();
    }

    casadi::SX RobReg_Classic::dq_select(const casadi::SX& dq_) {
        
        int n = dq_.size1();
        
        casadi::Slice allRows;
        casadi::SX mat_dq = casadi::SX::zeros(n, n * n);
        for (int i = 0; i < n; i++) {
            casadi::Slice sel_col(i*n,(i+1)*n);   // Select columns
            mat_dq(allRows, sel_col) = casadi::SX::eye(n)*dq_(i);
        }
        
        return mat_dq;
    }
    
    casadi::SXVector RobReg_Classic::createQ() {

        casadi::SXVector Q_(3);

        for(int i=0;i<3;i++){
            Q_[i] = casadi::SX::zeros(3,3);
        }
        
        Q_[0](1,2) = -1;
        Q_[0](2,1) = 1;

        Q_[1](0,2) = 1;
        Q_[1](2,0) = -1;

        Q_[2](0,1) = -1;
        Q_[2](1,0) = 1;

        return Q_;
    }
    
    casadi::SXVector RobReg_Classic::createE() {
        
        casadi::SXVector E_(6);

        for(int i=0;i<6;i++){
            E_[i] = casadi::SX::zeros(3,3);
        }
        E_[0](0,0) = 1;

        E_[1](0,1) = 1;
        E_[1](1,0) = 1;

        E_[2](0,2) = 1;
        E_[2](2,0) = 1;

        E_[3](1,1) = 1;

        E_[4](1,2) = 1;
        E_[4](2,1) = 1;

        E_[5](2,2) = 1;

        return E_;
    }
    
    casadi::SXVector RobReg_Classic::stdCmatrix(const casadi::SX& B, const casadi::SX& q_, const casadi::SX& dq_, const casadi::SX& dq_sel_) {
        
        int n = q_.size1();

        casadi::SX C123(n, n);
        casadi::SX C132(n, n);
        casadi::SX C231(n, n);

        for (int h = 0; h < n; h++) {
            for (int j = 0; j < n; j++) {
                for (int k = 0; k < n; k++) {
                    casadi::SX dbhj_dqk = jacobian(B(h, j), q_(k));
                    casadi::SX dbhk_dqj = jacobian(B(h, k), q_(j));
                    //casadi::SX dbjk_dqh = jacobian(B(j, k), q_(h));
                    C123(h, j) = C123(h, j) + 0.5 * (dbhj_dqk) * dq_(k);
                    C132(h, j) = C132(h, j) + 0.5 * (dbhk_dqj) * dq_(k);
                    //C231(h, j) = C231(h, j) + 0.5 * (dbjk_dqh) * dq_(k);
                }
            }
        }

        casadi::SXVector C(3);
        C[0] = C123;
        C[1] = C132;
        C[2] = C132.T();

        return C;
    }
    
    casadi::SX RobReg_Classic::SXregressor(
        
        const casadi::SX& q_, const casadi::SX& dq_, const casadi::SX& dqr_, const casadi::SX& ddqr_, 
        const std::string jointsType_, const Eigen::MatrixXd& DHtable_, FrameOffset& base_frame,FrameOffset& ee_frame){
        
        casadi::SXVector E_ = createE();
        casadi::SXVector Q_ = createQ();
        casadi::SX dq_sel_ = dq_select(dq_);

        const int nj = q_.size1();
        
        casadi::SXVector Ti(nj);
        casadi::SXVector T0i(nj);
        std::tuple<casadi::SXVector, casadi::SXVector> T_tuple;

        casadi::SXVector Jvi(nj);
        casadi::SXVector Jwi(nj);
        std::tuple<casadi::SXVector, casadi::SXVector> J_tuple;

        casadi::SX g = base_frame.get_gravity();

        casadi::SX Yr(nj,10*nj);        // regressor matrix
                
        casadi::Slice allRows;
        casadi::Slice allCols(0,nj);          
        casadi::Slice selR(0,3);

        T_tuple = DHFwKinJoints();
        T0i = std::get<0>(T_tuple);
        //Ti  = std::get<1>(T_tuple);

        J_tuple = DHJacJoints(T0i);
        Jvi = std::get<0>(J_tuple);
        Jwi = std::get<1>(J_tuple);
        
        for (int i=0; i<nj; i++) {
            
            casadi::SX R0i = T0i[i](selR,selR);
            // if(i==(nj-1)){	// end-effector
			// 	R0i = mtimes(R0i,ee_frame.get_rotation());
			// } else {
            // 	R0i = casadi::SX::mtimes({base_frame.get_rotation(),R0i,base_frame.get_rotation().T()});
			// }
            R0i = casadi::SX::mtimes({base_frame.get_rotation(),R0i,base_frame.get_rotation().T()});

            // ------------------------- Y0r_i -------------------------- //
            
            casadi::SX B0_i = mtimes(Jvi[i].T(),Jvi[i]);
            casadi::SXVector C = RobReg_Classic::stdCmatrix(B0_i, q_, dq_, dq_sel_);

            casadi::SX dX0r_i = mtimes(B0_i,ddqr_);
            casadi::SX W0r_i = -mtimes((C[0]+C[1]-C[2]),dqr_)/2;
            casadi::SX Z0r_i = -mtimes(Jvi[i].T(),g);
            
            casadi::SX Y0r_i = dX0r_i - W0r_i + Z0r_i;
            
            // ------------------------- Y1r_i -------------------------- //
            
            casadi::SX dX1r_i(nj,3);
            casadi::SX W1r_i(nj,3);

            for (int l=0; l<3; l++) {

                casadi::SX Ql = Q_[l];
                casadi::SX B1l_i = casadi::SX::mtimes({Jwi[i].T(),R0i,Ql,R0i.T(),Jvi[i]}) - 
                                   casadi::SX::mtimes({Jvi[i].T(),R0i,Ql,R0i.T(),Jwi[i]});
                casadi::SXVector C = RobReg_Classic::stdCmatrix(B1l_i, q_, dq_, dq_sel_);

                dX1r_i(allRows,l) = mtimes(B1l_i,ddqr_);
                W1r_i(allRows,l) = -mtimes(C[0]+C[1]-C[2],dqr_)/2;
            }
            casadi::SX Z1r_i= -(jacobian(mtimes(R0i.T(),g),q_)).T();

            casadi::SX Y1r_i = dX1r_i - W1r_i + Z1r_i;

            // ------------------------- Y2r_i -------------------------- //

            casadi::SX dX2r_i(nj,6);
            casadi::SX W2r_i(nj,6);
            
            for (int l=0; l<6; l++) {

                casadi::SX El = E_[l];
                casadi::SX B2l_i = casadi::SX::mtimes({Jwi[i].T(),R0i,El,R0i.T(),Jwi[i]});
                casadi::SXVector C = RobReg_Classic::stdCmatrix(B2l_i, q_, dq_, dq_sel_);

                dX2r_i(allRows,l) = mtimes(B2l_i,ddqr_);
                W2r_i(allRows,l) = -mtimes(C[0]+C[1]-C[2],dqr_)/2;
            }

            casadi::SX Y2r_i = dX2r_i - W2r_i;

            // ------------------------- Yr_i -------------------------- //

            casadi::SX Yr_i = horzcat(Y0r_i,Y1r_i,Y2r_i);
            
            casadi::Slice selCols(i*10,(i+1)*10);          // Select current columns of matrix regressor

            Yr(allRows,selCols) = Yr_i;
        }

        return Yr;
    }
    
    void RobReg_Classic::init_casadi_functions(){

        DHKin();
        DHJac();
        Regressor();
    }
    
    void RobReg_Classic::Regressor(){
        
        SX_Yr = SXregressor(_q_, _dq_, _dqr_, _ddqr_, _jointsType_, _DHtable_, _lab2L0_, _Ln2EE_);

        casadi::Function regr_fun(REG_STRING, {_q_,_dq_,_dqr_,_ddqr_}, {densify(SX_Yr)});
        
        regressor_fun = regr_fun;
    }

    void RobReg_Classic::setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_,const Eigen::VectorXd& dqr_,const Eigen::VectorXd& ddqr_){
    
        if(q_.size() == _numJoints_ && dq_.size()==_numJoints_ && dqr_.size()==_numJoints_ && ddqr_.size()==_numJoints_){
            q = q_;
            dq = dq_;
            dqr = dqr_;
            ddqr = ddqr_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        compute();
    }

    void RobReg_Classic::compute(){
        
        for(int i=0;i<_numJoints_;i++){
            args[0](i,0) = q(i);
            args[1](i,0) = dq(i);
            args[2](i,0) = dqr(i);
            args[3](i,0) = ddqr(i);
        }
        kinematic_fun.call({args[0]},kinematic_res);
        jacobian_fun.call({args[0]},jacobian_res);
        regressor_fun.call(args,regressor_res);
    }
    
    Eigen::MatrixXd RobReg_Classic::getRegressor(){
        
        const int nrow = SX_Yr.size1();
        const int ncol = SX_Yr.size2();
        Eigen::MatrixXd Yfull(nrow,ncol);
        std::vector<casadi::SXElem> reg_elements = regressor_res[0].get_elements();
        std::transform(reg_elements.begin(), reg_elements.end(), Yfull.data(), mapFunction);
        
        return Yfull;
    }

    void RobReg_Classic::generate_code(std::string& savePath){
        
        // Options for c-code auto generation
        casadi::Dict opts = casadi::Dict();
        opts["cpp"] = true;
        opts["with_header"] = true;
        
        // generate functions in c code
        casadi::CodeGenerator myCodeGen = casadi::CodeGenerator(GENERATED_FILE, opts);

        myCodeGen.add(kinematic_fun);
        myCodeGen.add(jacobian_fun);
        myCodeGen.add(regressor_fun);

        myCodeGen.generate(savePath);
    }

    std::vector<std::string> RobReg_Classic::getFunctionsName() {

        std::vector<std::string> orig_name_funs;
        std::vector<std::string> all_name_funs;
        int dim_orig_funs;

        orig_name_funs = RobKinBasic::getFunctionsName();

        dim_orig_funs = orig_name_funs.size();
        all_name_funs.resize(NUMBER_FUNCTIONS);

        for (int i=0; i<dim_orig_funs; i++){
            all_name_funs[i] = orig_name_funs[i];
        }        
        
        all_name_funs[dim_orig_funs] = REG_STRING;
        
        return all_name_funs;
    }

    std::vector<casadi::Function> RobReg_Classic::getCasadiFunctions() {
        std::vector<casadi::Function> casadi_funs;
        casadi_funs.resize(NUMBER_FUNCTIONS);

        casadi_funs[0] = kinematic_fun;
        casadi_funs[1] = jacobian_fun;
        casadi_funs[2] = regressor_fun;

        return casadi_funs;
    }

}