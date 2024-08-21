#include "../library/robot.h"
#include "../library/kinematics.h"

/* Function name used to generate code */
#define KIN_STRING "kin_fun"
#define JAC_STRING "jac_fun"
#define T_STRING "T_fun"
#define DOT_JAC_STRING "dotJac_fun"
#define PINV_JAC_STRING "pinvJac_fun"
#define PINV_JAC_POS_STRING "pinvJacPos_fun"
#define DOT_PINV_JAC_STRING "dotPinvJac_fun"
#define DOT_PINV_JAC_POS_STRING "dotPinvJacPos_fun"
#define MASS_STRING "mass_fun"
#define CORIOLIS_STRING "coriolis_fun"
#define GRAVITY_STRING "gravity_fun"

/* File name of generated code */
#define GENERATED_FILE "kin_basic_fun.cpp"
#define GENERATED_FILE "kinematics_fun.cpp"
#define GENERATED_FILE "dynamics_fun.cpp"

/* Define number of function generable */
// #define NUMBER_FUNCTIONS 10
// #define MU 0.02

namespace thunder_ns{

	// constexpr double MU = 0.02; //pseudo-inverse damping coeff
	constexpr unsigned int N_PAR_LINK = 10; // number of link+joint parameters
	// constexpr unsigned int NUMBER_FUNCTIONS = 10; // number of generable functions

	Robot::Robot(const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, FrameOffset& base_frame, FrameOffset& ee_frame){
		_numJoints_ = numJoints;
		_jointsType_ = jointsType;
		_DHtable_ = DHtable;
		_world2L0_ = base_frame;
		_Ln2EE_ = ee_frame;
		valid = 1;
		// _mu_ = MU;

		initVarsFuns();
		compute();
	}

	void Robot::initVarsFuns(){
		
		if (_DHtable_.rows() != _numJoints_ || _DHtable_.cols() != 4){
			throw std::runtime_error("DHTemplate: Error size of DH table");
		}
		if ((int)_jointsType_.size()!=_numJoints_){
			throw std::runtime_error("DHFwkinJoints: Error size of joints string");
		}

		_q_ = casadi::SX::sym("_q_", _numJoints_,1);
		_dq_ = casadi::SX::sym("_dq_", _numJoints_,1);
		_dqr_ = casadi::SX::sym("_dqr_", _numJoints_,1);
		_ddqr_ = casadi::SX::sym("_ddqr_", _numJoints_,1);
		// _par_KIN_ = casadi::SX::sym("_par_DYN_", N_PAR_KIN,1);
		_par_DYN_ = casadi::SX::sym("_par_DYN_", N_PAR_LINK*_numJoints_,1);
		_par_REG_ = casadi::SX::sym("_par_REG_", N_PAR_LINK*_numJoints_,1);
		
		// model update
		model.insert({"q", _q_});
		model.insert({"dq", _dq_});
		model.insert({"dqr", _dqr_});
		model.insert({"ddqr", _ddqr_});
		// model.insert({"par_KIN", _par_KIN_});
		model.insert({"par_DYN", _par_DYN_});
		model.insert({"par_REG", _par_REG_});

		// functions update
		// casadi_fun.insert

		// numerical values
		q = Eigen::VectorXd::Zero(_numJoints_,1);
		dq = Eigen::VectorXd::Zero(_numJoints_,1);
		dqr = Eigen::VectorXd::Zero(_numJoints_,1);
		ddqr = Eigen::VectorXd::Zero(_numJoints_,1);
		par_DYN = Eigen::VectorXd::Zero(N_PAR_LINK*_numJoints_);
		par_REG = Eigen::VectorXd::Zero(N_PAR_LINK*_numJoints_);
		get.insert({"q", q});
		get.insert({"dq", dq});
		get.insert({"dqr", dqr});
		get.insert({"ddqr", ddqr});
		// get.insert({"par_KIN", par_KIN});
		get.insert({"par_DYN", par_DYN});
		get.insert({"par_REG", par_REG});
		
		// make args a map?
		// 0: q, 1: dq, 2: dqr, 3: ddqr, 4: par_DYN
		args.resize(5);
		for(int i=0; i<args.size(); i++){
			args[i].resize(_numJoints_,1);
		}
		args[4].resize(N_PAR_LINK*_numJoints_,1);
	
		// kinematic_res.resize(1);
		// jacobian_res.resize(1);
		// dotJacobian_res.resize(1);
		// pinvJacobian_res.resize(1);
		// pinvJacobianPos_res.resize(1);
		// dotPinvJacobian_res.resize(1);
		// dotPinvJacobianPos_res.resize(1);
		// mass_res.resize(1);
		// coriolis_res.resize(1);
		// gravity_res.resize(1);

		init_fun_names();

		init_casadi_functions();
	}

	void Robot::init_fun_names(){

	}

	void Robot::init_casadi_functions(){
		// functions have to be added by various modules
		DHKin();
		DHJac();
		DHDotJac();
		DHPinvJac();
		DHPinvJacPos();
		DHDotPinvJac();
		DHDotPinvJacPos();
		DHKin();
		DHJac();
		mass_coriolis_gravity();
	}

	void Robot::compute(){

		for(int i=0; i<_numJoints_; i++){
			args[0](i,0) = q(i);
			args[1](i,0) = dq(i);
			args[2](i,0) = dqr(i);
			args[3](i,0) = ddqr(i);
		}
		for(int i=0; i<N_PAR_LINK*_numJoints_; i++){
			args[4](i,0) = par_DYN(i);
		}

		kinematic_fun.call({args[0]},kinematic_res);
		jacobian_fun.call({args[0]},jacobian_res);
		dotJacobian_fun.call({args[0],args[1]},dotJacobian_res);
		pinvJacobian_fun.call({args[0]},pinvJacobian_res);
		pinvJacobianPos_fun.call({args[0]},pinvJacobianPos_res);
		dotPinvJacobian_fun.call({args[0],args[1]},dotPinvJacobian_res);
		dotPinvJacobianPos_fun.call({args[0],args[1]},dotPinvJacobianPos_res); 
		mass_fun.call({args[0],args[4]},mass_res);
		coriolis_fun.call({args[0],args[1],args[4]},coriolis_res);
		gravity_fun.call({args[0],args[4]},gravity_res);   
	}

    void Robot::generate_code(std::string& savePath){
		
		// Options for c-code auto generation
		casadi::Dict opts = casadi::Dict();
		opts["cpp"] = true;
		opts["with_header"] = true;
		
		// generate functions in c code
		casadi::CodeGenerator myCodeGen = casadi::CodeGenerator(GENERATED_FILE, opts);

		// myCodeGen.add(kinematic_fun);
		// myCodeGen.add(jacobian_fun);
		// myCodeGen.add(dotJacobian_fun);
		// myCodeGen.add(pinvJacobian_fun);
		// myCodeGen.add(pinvJacobianPos_fun);
		// myCodeGen.add(dotPinvJacobian_fun);
		// myCodeGen.add(dotPinvJacobianPos_fun);
		// myCodeGen.add(mass_fun);
		// myCodeGen.add(coriolis_fun);
		// myCodeGen.add(gravity_fun);

		// for (int i=0; i<NUMBER_FUNCTIONS; i++){
		// 	myCodeGen.add(casadi_functions[i]);
		// }
		for (const auto& f : casadi_fun) {
			myCodeGen.add(f.second);
		}

		myCodeGen.generate(savePath);
	}

	std::vector<std::string> Robot::getFunctionNames() {

		std::vector<std::string> name_funs;
		int sz = functions.size();
		name_funs.resize(sz);

		// made with for and list of string for functions
		// name_funs[0] = KIN_STRING;
		// name_funs[1] = JAC_STRING;
		// name_funs[2] = DOT_JAC_STRING; 
		// name_funs[3] = PINV_JAC_STRING;
		// name_funs[4] = PINV_JAC_POS_STRING;
		// name_funs[5] = DOT_PINV_JAC_STRING;
		// name_funs[6] = DOT_PINV_JAC_POS_STRING;
		// name_funs[7] = MASS_STRING;
		// name_funs[8] = CORIOLIS_STRING;
		// name_funs[9] = GRAVITY_STRING;
		for (int i=0; i<sz; i++){
			name_funs[0] = functions[i].name;
		}

		return name_funs;
	}

	std::vector<casadi::Function> Robot::getCasadiFunctions() {
		
		std::vector<casadi::Function> casadi_functions;
		int sz = functions.size();
		casadi_functions.resize(sz);

		// casadi_funs[0] = kinematic_fun;
		// casadi_funs[1] = jacobian_fun;
		// casadi_funs[2] = dotJacobian_fun; 
		// casadi_funs[3] = pinvJacobian_fun;
		// casadi_funs[4] = pinvJacobianPos_fun;
		// casadi_funs[5] = dotPinvJacobian_fun;
		// casadi_funs[6] = dotPinvJacobianPos_fun;

		for (int i=0; i<sz; i++){
			casadi_functions[i] = functions[i].fun;
		}

		return casadi_functions;
	}

	int Robot::get_numJoints(){
		return _numJoints_;
	}

	std::string Robot::get_jointsType(){
		return _jointsType_;
	}

	Eigen::MatrixXd Robot::get_DHTable(){
		return _DHtable_;
	}

	FrameOffset Robot::get_world2L0(){
		return _world2L0_;
	}

	FrameOffset Robot::get_Ln2EE(){
		return _Ln2EE_;
	}

	void Robot::update(){
		// update the map get[] with the casadi functions
	}

	int Robot::add_function(std::string name, casadi::SX expr, casadi::SXVector args, std::string descr = ""){
		// maybe directly model[name] = ...?
		if (model.count(name)){
    		// key already exists
			return 0;
		} else {
			model[name] = expr;
			fun_args[name] = args;
			fun_descr[name] = descr;
			casadi::Function fun(name, args, {densify(expr)});
			casadi_fun[name] = fun;
		}
	}

}