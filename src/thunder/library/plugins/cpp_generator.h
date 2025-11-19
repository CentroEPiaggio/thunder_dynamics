#ifndef CPP_GEN_H
#define CPP_GEN_H

#include "../plugin_interfaces.h"
#include "../robot.h"
#include <yaml-cpp/yaml.h>

using std::cout;
using std::endl;
namespace thunder_ns {


    class CPPGenerator : public BaseGenerator {
        private:
        YAML::Node config_;
        std::string robot_name;

        int copy_to(std::string robot_name, std::string path_from, std::string path_conf, std::string path_par, std::string path_h, std::string path_cpp){
            // --- copy generated files --- //
            try{
                std::filesystem::path sourcePath;
                std::filesystem::path sourceDestPath;

                // copy .h generated files
                sourcePath = path_from + robot_name + "_gen" + ".h";
                sourceDestPath = path_h + robot_name + "_gen" + ".h";
                std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

                // copy .cpp generated files
                sourcePath = path_from + robot_name + "_gen" + ".cpp";
                sourceDestPath = path_cpp + robot_name + "_gen" + ".cpp";
                std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);
                
                // copy conf and parameters files
                sourcePath = path_from + robot_name + "_par_REG.yaml";
                sourceDestPath = path_par + robot_name + "_par_REG.yaml";
                std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);
                sourcePath = path_from + robot_name + "_conf.yaml";
                sourceDestPath = path_conf + robot_name + "_conf.yaml";
                std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

                // copy thunder_robot
                sourcePath = path_from + "thunder_" + robot_name + ".h";
                sourceDestPath = path_h + "thunder_" + robot_name + ".h";
                std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);
                sourcePath = path_from + "thunder_" + robot_name + ".cpp";
                sourceDestPath = path_cpp + "thunder_" + robot_name + ".cpp";
                std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);
            } catch (const std::runtime_error &err) {
                std::cout << err.what() << std::endl;
                return 0;
            }

            return 1;
        }
        
        public:
        
        CPPGenerator() : BaseGenerator("CPP Generator", "Generates a plaiin Eigen C++ library for Robot") {}
        
        
        int configure(const YAML::Node& config) override{
            std::cout<<"Configuring cpp plugin"<<std::endl;
            config_ = config;
            return 0;
        }
        
        
        void generate(const std::shared_ptr<Robot> robot) override{
            int nj = robot->get_numJoints();
            // --- Generate merge code --- //

            std::string robot_name = "robot";
            std::string path_robot = "../robots/";
            std::string config_file = path_robot + robot_name + "/robot.yaml";
            std::string robot_name_gen = robot_name + "_gen";

            std::string relativePath = robot_name + "_generatedFiles/";

            std::filesystem::path currentPath = std::filesystem::current_path();
            std::string absolutePath = currentPath / relativePath;

            // Create directory
            try {
                std::filesystem::create_directory(absolutePath);
            } catch(std::exception & e){
                std::cout<<"Problem creating directory generatedFiles/"<<std::endl;
                return;
            }

            // Generate library
            // if( gen_command.get<bool>("casadi"))
            // 	std::cout<<"Saving Casadi functions!"<<std::endl;
            robot->generate_library(absolutePath, robot_name_gen, config_["gen_casadi"].as<bool>());

            // --- Write thunder_robot into generatedFiles --- //
            std::filesystem::path sourcePath;
            std::filesystem::path sourceDestPath;
            std::string thunder_robot_cpp_path;
            std::string thunder_robot_h_path;
            std::string python_cmake_file;

            // Get home/.local/share directory
            std::string home = std::getenv("HOME");
            // std::string template_path = home + "/.local/share/thunder_dynamics/thunder_robot_template/";
            std::string template_path = "/usr/local/share/thunder_dynamics/thunder_robot_template/";

            if (std::filesystem::is_directory(template_path)){
                thunder_robot_cpp_path = template_path + "thunder_robot.cpp";
                thunder_robot_h_path = template_path + "thunder_robot.h";
                python_cmake_file = template_path + "CMakeLists.txt";
            }else{
                std::cerr<<"Template path not found: "<<template_path<<std::endl;
            }

            sourceDestPath = absolutePath + "thunder_" + robot_name + ".h";
            sourcePath = thunder_robot_h_path;
            std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

            sourceDestPath = absolutePath + "thunder_" + robot_name + ".cpp";
            sourcePath = thunder_robot_cpp_path;
            std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

            if (config_["python"].as<bool>()){
                // --- Generate python binding --- //
                std::filesystem::copy_file(python_cmake_file, absolutePath +  "CMakeLists.txt", std::filesystem::copy_options::overwrite_existing);
                int changed = update_cmake("robot", robot_name, absolutePath +  "CMakeLists.txt");
                if (!changed) {
                    cout<<"problem on changing robot name in the CMakeLists.txt:"<<endl;
                    return;
                }
                cout<<"Python binding generated!"<<endl;
            }

            // --- change the necessary into thunder_robot --- //
            int changed = change_to_robot("robot", robot_name, *robot, absolutePath+"thunder_"+robot_name+".h", absolutePath+"thunder_"+robot_name+".cpp", config_["python"].as<bool>());
            if (!changed) {
                cout<<"problem on changing robot name:"<<endl;
                return;
            }

            // --- generate parameters files --- //
            std::string par_file = absolutePath + robot_name + "_conf.yaml";
            std::string par_REG_file = absolutePath + robot_name + "_par_REG.yaml";
            robot->save_conf(par_file);
            robot->save_par_REG(par_REG_file);
            // if (!genInertial_files(robot_name, nj, config_file, par_file, par_REG_file)){
            // 	return 0;
            // }

            std::cout<<"Library generated!"<<std::endl;

            // thunder_robot path
            std::string COPY_PREFIX;
            if (currentPath.filename() == "build") { // last directory name
                COPY_PREFIX = currentPath/"../../../";
            } else {
                COPY_PREFIX = "/home/thunder_dev/thunder_dynamics/";
            }
            std::string PATH_COPY_H = COPY_PREFIX + "src/thunder_robot/library/";
            std::string PATH_COPY_CPP = COPY_PREFIX + "src/thunder_robot/src/";
            std::string PATH_COPY_YAML = COPY_PREFIX + "src/thunder_robot/robots/";
            std::string PATH_COPY_CHRONO_H = COPY_PREFIX + "src/thunder_robot_chrono/library/";
            std::string PATH_COPY_CHRONO_CPP = COPY_PREFIX + "src/thunder_robot_chrono/src/";
            std::string PATH_COPY_CHRONO_YAML = COPY_PREFIX + "src/thunder_robot_chrono/robots/";
            

        }

    };

} // namespace thunder_ns

#endif // CPP_GEN_H
