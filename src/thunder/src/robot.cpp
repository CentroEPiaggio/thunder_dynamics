#include "../library/robot.h"
#include "../library/kinematics.h"
#include "../library/utils.h"

/* File name of generated code */
// #define GENERATED_FILE "kin_basic_fun.cpp"
// #define GENERATED_FILE "kinematics_fun.cpp"
// #define GENERATED_FILE "dynamics_fun.cpp"
#define GENERATED_FILE "robot_gen.cpp"

/* Define number of function generable */
// #define NUMBER_FUNCTIONS 10
// #define MU 0.02
// #define N_PAR_LINK = 10

using namespace std;

namespace thunder_ns{

	// constexpr double MU = 0.02; //pseudo-inverse damping coeff
	// constexpr unsigned int N_PAR_LINK = 10; // number of link+joint parameters
	// constexpr unsigned int NUMBER_FUNCTIONS = 10; // number of generable functions

	Robot::Robot(const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, FrameOffset& base_frame, FrameOffset& ee_frame){
		_numJoints_ = numJoints;
		N_PAR_LINK = 10;
		_jointsType_ = jointsType;
		_DHtable_ = DHtable;
		_world2L0_ = base_frame;
		// N_PAR_LINK = 10;
		// gravity = base_frame.get_gravity();
		_Ln2EE_ = ee_frame;
		valid = 1;
		// _mu_ = MU;

		initVarsFuns();
		// compute();
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
		// q = Eigen::VectorXd::Zero(_numJoints_,1);
		// dq = Eigen::VectorXd::Zero(_numJoints_,1);
		// dqr = Eigen::VectorXd::Zero(_numJoints_,1);
		// ddqr = Eigen::VectorXd::Zero(_numJoints_,1);
		// par_DYN = Eigen::VectorXd::Zero(N_PAR_LINK*_numJoints_);
		// par_REG = Eigen::VectorXd::Zero(N_PAR_LINK*_numJoints_);
		args.insert({"q", casadi::SX::zeros(_numJoints_,1)});
		args.insert({"dq", casadi::SX::zeros(_numJoints_,1)});
		args.insert({"dqr", casadi::SX::zeros(_numJoints_,1)});
		args.insert({"ddqr", casadi::SX::zeros(_numJoints_,1)});
		// args.insert({"par_KIN", par_KIN});
		args.insert({"par_DYN", casadi::SX::zeros(N_PAR_LINK*_numJoints_)});
		args.insert({"par_REG", casadi::SX::zeros(N_PAR_LINK*_numJoints_)});
		
		// make args a map?
		// 0: q, 1: dq, 2: dqr, 3: ddqr, 4: par_DYN
		// args.resize(5);
		// for(int i=0; i<args.size(); i++){
		// 	args[i].resize(_numJoints_,1);
		// }
		// args[4].resize(N_PAR_LINK*_numJoints_,1);
	
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

		// init_fun_names();

		// init_casadi_functions();
	}

	// void Robot::init_fun_names(){

	// }

	// void Robot::init_casadi_functions(){
	// 	// functions have to be added by various modules
	// 	DHKin();
	// 	DHJac();
	// 	DHDotJac();
	// 	DHPinvJac();
	// 	DHPinvJacPos();
	// 	DHDotPinvJac();
	// 	DHDotPinvJacPos();
	// 	DHKin();
	// 	DHJac();
	// 	mass_coriolis_gravity();
	// }

	Eigen::MatrixXd Robot::get(std::string name){
		std::vector<casadi::SX> result;
		// Eigen::MatrixXd result_num;
		if (fun_args.count(name)){
    		// key exists
			// cout<<"ok1, name: "<<name<<endl;
			Eigen::MatrixXd result_num(model[name].rows(), model[name].columns());
			// cout<<"ok1.5, output_size: "<<result_num.size()<<endl;
			auto f_args = fun_args[name];
			int sz = f_args.size();
			// cout<<"ok2, f_args:"<<f_args<<", size: "<<sz<<endl;
			casadi::SXVector inputs(sz);
			int i=0;
			for (const auto& arg : f_args) {
				inputs[i] = args[arg];
				i++;
			}
			// cout<<"ok3, args: "<<inputs<<endl;
			casadi::Function fun = casadi_fun[name];
			// cout<<"ok3.5, fun: "<<fun<<endl;
			fun.call(inputs, result);
			// cout<<"ok4, result: "<<result<<endl;
			std::vector<casadi::SXElem> res_elements = result[0].get_elements();
			// cout<<"ok5, elements: "<<res_elements<<endl;
			std::transform(res_elements.begin(), res_elements.end(), result_num.data(), mapFunction);
			// cout<<"ok6, result: "<<result_num<<endl;
			return result_num;
		} else {
			cout<<name + " not recognised"<<endl;
			// result_num.resize(1,1);
			// result_num << 0;
			return Eigen::Matrix<double, 1, 1>::Zero();
		}
		
		// return result_num;
	}

	int Robot::set_arg(std::string name, Eigen::VectorXd value){
		int sz = value.size();
		casadi::SX& arg = args[name];
		arg = casadi::SX::zeros(sz);
		for (int i=0; i<sz; i++){
			arg(i) = value(i);
		}
		// if name==par_DYN or par_REG change also the other one!!

		return 1;
	}

	int Robot::set_q(Eigen::VectorXd value){
		casadi::SX& q = args["q"];
		if (value.size() == _numJoints_){
			for (int i=0; i<_numJoints_; i++){
				q(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	int Robot::set_dq(Eigen::VectorXd value){
		casadi::SX& dq = args["dq"];
		if (value.size() == _numJoints_){
			for (int i=0; i<_numJoints_; i++){
				dq(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	int Robot::set_dqr(Eigen::VectorXd value){
		casadi::SX& dqr = args["dqr"];
		if (value.size() == _numJoints_){
			for (int i=0; i<_numJoints_; i++){
				dqr(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	int Robot::set_ddqr(Eigen::VectorXd value){
		casadi::SX& ddqr = args["ddqr"];
		if (value.size() == _numJoints_){
			for (int i=0; i<_numJoints_; i++){
				ddqr(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	int Robot::set_par_DYN(Eigen::VectorXd value){
		casadi::SX& par_DYN = args["par_DYN"];
		if (value.size() == N_PAR_LINK*_numJoints_){
			for (int i=0; i<N_PAR_LINK*_numJoints_; i++){
				par_DYN(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
		// change par_REG
	}

	int Robot::set_par_REG(Eigen::VectorXd value){
		casadi::SX& par_REG = args["par_REG"];
		if (value.size() == N_PAR_LINK*_numJoints_){
			for (int i=0; i<N_PAR_LINK*_numJoints_; i++){
				par_REG(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
		// change par_DYN
	}

	Eigen::VectorXd Robot::get_par_DYN(){
		const casadi::SX& par_DYN_casadi = args["par_DYN"];
		int sz = N_PAR_LINK*_numJoints_;
		Eigen::VectorXd par_DYN(sz);
		std::vector<casadi::SXElem> res_elements = par_DYN_casadi.get_elements();
		std::transform(res_elements.begin(), res_elements.end(), par_DYN.data(), mapFunction);
		return par_DYN;
	}

	Eigen::VectorXd Robot::get_par_REG(){
		const casadi::SX& par_REG_casadi = args["par_REG"];
		int sz = N_PAR_LINK*_numJoints_;
		Eigen::VectorXd par_REG(sz);
		std::vector<casadi::SXElem> res_elements = par_REG_casadi.get_elements();
		std::transform(res_elements.begin(), res_elements.end(), par_REG.data(), mapFunction);
		return par_REG;
	}

	// void Robot::compute(){
	// 	for(int i=0; i<_numJoints_; i++){
	// 		args[0](i,0) = q(i);
	// 		args[1](i,0) = dq(i);
	// 		args[2](i,0) = dqr(i);
	// 		args[3](i,0) = ddqr(i);
	// 	}
	// 	for(int i=0; i<N_PAR_LINK*_numJoints_; i++){
	// 		args[4](i,0) = par_DYN(i);
	// 	}

	// 	kinematic_fun.call({args[0]},kinematic_res);
	// 	jacobian_fun.call({args[0]},jacobian_res);
	// 	dotJacobian_fun.call({args[0],args[1]},dotJacobian_res);
	// 	pinvJacobian_fun.call({args[0]},pinvJacobian_res);
	// 	pinvJacobianPos_fun.call({args[0]},pinvJacobianPos_res);
	// 	dotPinvJacobian_fun.call({args[0],args[1]},dotPinvJacobian_res);
	// 	dotPinvJacobianPos_fun.call({args[0],args[1]},dotPinvJacobianPos_res); 
	// 	mass_fun.call({args[0],args[4]},mass_res);
	// 	coriolis_fun.call({args[0],args[1],args[4]},coriolis_res);
	// 	gravity_fun.call({args[0],args[4]},gravity_res);   
	// }

    // void Robot::generate_code(std::string& savePath){
		
	// 	// Options for c-code auto generation
	// 	casadi::Dict opts = casadi::Dict();
	// 	opts["cpp"] = true;
	// 	opts["with_header"] = true;
		
	// 	// generate functions in c code
	// 	casadi::CodeGenerator myCodeGen = casadi::CodeGenerator(GENERATED_FILE, opts);
	// 	for (const auto& f : casadi_fun) {
	// 		myCodeGen.add(f.second);
	// 	}

	// 	myCodeGen.generate(savePath);
	// }

	std::vector<fun_obj> Robot::get_functions(bool onlyNames) {
		std::vector<fun_obj> functions;
		int sz = casadi_fun.size();
		functions.resize(sz);
		int i=0;
		for (auto &f : casadi_fun){
			std::string name = f.first;
			functions[i].name = f.first;
			functions[i].description = fun_descr[name];
			functions[i].args = fun_args[name];
			functions[i].out_size.resize(2);
			functions[i].out_size[0] = model[name].size1();
			functions[i].out_size[1] = model[name].size2();
			if (!onlyNames){
				functions[i].expr = model[name];
				functions[i].fun = f.second;
			}
			i++;
		}
		return functions;
	}

	// std::vector<casadi::Function> Robot::getCasadiFunctions() {
		
	// 	std::vector<casadi::Function> casadi_functions;
	// 	int sz = functions.size();
	// 	casadi_functions.resize(sz);

	// 	// casadi_funs[0] = kinematic_fun;
	// 	// casadi_funs[1] = jacobian_fun;
	// 	// casadi_funs[2] = dotJacobian_fun; 
	// 	// casadi_funs[3] = pinvJacobian_fun;
	// 	// casadi_funs[4] = pinvJacobianPos_fun;
	// 	// casadi_funs[5] = dotPinvJacobian_fun;
	// 	// casadi_funs[6] = dotPinvJacobianPos_fun;

	// 	for (int i=0; i<sz; i++){
	// 		casadi_functions[i] = functions[i].fun;
	// 	}

	// 	return casadi_functions;
	// }

	int Robot::get_numJoints(){
		return _numJoints_;
	}

	int Robot::get_numParLink(){
		return N_PAR_LINK;
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

	// int Robot::get_N_PAR_LINK(){
	// 	return N_PAR_LINK;
	// }

	// void Robot::update(){
	// 	// update the map get[] with the casadi functions
	// }

	int Robot::add_function(std::string name, casadi::SX expr, std::vector<std::string> f_args, std::string descr){
		// maybe directly model[name] = ...?
		if (model.count(name)){
			// key already exists
			return 0;
		} else {
			model[name] = expr;
			fun_args[name] = f_args;
			fun_descr[name] = descr;

			casadi::SXVector inputs(f_args.size());
			// for (const auto& arg : f_args) {
			// 	inputs.push_back(model[arg]);
			// }
			int i=0;
			for (const auto& arg : f_args) {
				inputs[i] = model[arg];
				i++;
			}

			casadi::Function fun(name+"_fun", inputs, {densify(expr)});
			// cout<<"fun: "<<fun<<endl;
			casadi_fun[name] = fun;
		}

		return 1;
	}

	void Robot::generate_library(const std::string& savePath, const std::string& name_file){
		// Options for c-code auto generation
		casadi::Dict opts = casadi::Dict();
		opts["cpp"] = true;
		opts["with_header"] = true;
		
		// generate functions in c code
		casadi::CodeGenerator myCodeGen = casadi::CodeGenerator(name_file, opts);
		// cout<<"casadi_fun: "<<casadi_fun<<endl;

		for (const auto& f : casadi_fun) {
			myCodeGen.add(f.second);
			// cout<<"f_name: "<<f.first<<endl;
			// cout<<"fun: "<<f.second<<endl<<endl;
		}
		myCodeGen.generate(savePath);
	}

}