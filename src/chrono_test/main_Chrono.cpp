#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>
#include <chrono>
#include <random>

#include "../thunder/library/RobReg.h"
#include "../thunder/library/RobReg_Classic.h"
//#include "./library/ThunderPanda.h"
#include "./library/ThunderPandaClassic.h"

#define ITERATIONS 10000
#define SELECT 1
#define NJ 3

using namespace regrob;
using std::cout;
using std::endl;

//std::string name0 = "./csv_files/init_classic.csv";
std::string name1 = "./csv_files/compute_classic_3R_.csv";
std::string name2 = "./csv_files/compute_new_3R.csv";
std::string name3 = "./csv_files/compute_thunder_3R.csv";
std::string name4 = "./csv_files/compute_thunder_classic_3R.csv";

int main(){

    /* Init Timer */
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

    /* Init Arguments Regressor */
    Eigen::Matrix<double, NJ, 1> q;
    Eigen::Matrix<double, NJ, 1> Dq;
    Eigen::Matrix<double, NJ, 1> Dqr;
    Eigen::Matrix<double, NJ, 1> DDqr;

    /* Init Variables Regressor */
    //const int NJ = 7;
    std::string jType, jType_FULL;
    FrameOffset base_to_L0({0,0,0},{0,0,0},{0,0,-9.81});
    FrameOffset Ln_to_EE;
    Eigen::Matrix<double,NJ,4> DH_table;
    Eigen::Matrix<double,7,4> DH_table_FULL;
    Eigen::Matrix<double,NJ, NJ*10> Yr_classic, Yr_new, Yr_thunder;
    RobReg_Classic regrobot_classic;
    RobReg regrobot_new;
    //thunderPanda regrobot_thunder;
    thunderPandaClassic regrobot_thunder;

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> distribution(-2*M_PI, 2*M_PI);

    /* CSV Files */
    std::ofstream csv_classic, csv_new, csv_thunder, csv_init_c;

    DH_table_FULL << 0,		-M_PI_2,	0.3330, 0,
                    0,      M_PI_2,  	0,      0,
                    0.0825, M_PI_2,  	0.3160, 0,
                   -0.0825, -M_PI_2,	0,      0,
                    0,      M_PI_2,  	0.384,  0,
                    0.088,  M_PI_2,  	0,      0,
                    0,      0,         	0.107,  0;
    
    DH_table = DH_table_FULL.block(0,0,NJ,4);
    jType_FULL = "RRRRRRR"; 
    jType = jType_FULL.substr(0,NJ); 

    Ln_to_EE.set_translation({0.0,0.0,0.0});
    Ln_to_EE.set_ypr({0.0,0.0,0.0});

    /* csv_init_c.open(name0);
    for (int jjj = 0; jjj < ITERATIONS; jjj++) {
        start = std::chrono::high_resolution_clock::now();
        regrobot_classic.init(NJ,jType,DH_table,base_to_L0,Ln_to_EE);
        end = std::chrono::high_resolution_clock::now();
        
        duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        csv_init_c << duration << "\n";
    }
    csv_init_c.close(); */
    
    if (SELECT == 0){
        regrobot_classic.init(NJ,jType,DH_table,base_to_L0,Ln_to_EE);
        csv_classic.open(name1);
        for (int jjj = 0; jjj < ITERATIONS; jjj++) {
            
            for(int k = 0; k<NJ; k++){
                q[k] = distribution(rng);
                Dq[k] = distribution(rng);
                Dqr[k] = distribution(rng);
                DDqr[k] = distribution(rng);
            }
            
        // ----------------------------------------------------------------- //
            start = std::chrono::high_resolution_clock::now();
            
            regrobot_classic.setArguments(q, Dq, Dqr, DDqr);
            Yr_classic = regrobot_classic.getRegressor();
            
            end = std::chrono::high_resolution_clock::now();
            // ----------------------------------------------------------------- //
            
            duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
            csv_classic << duration << "\n";
            if (jjj%100==0){std::cout<<jjj<<std::endl;}
        }
        csv_classic.close();
        
    }
    
    if (SELECT == 1){
        regrobot_new.init(NJ,jType,DH_table,base_to_L0,Ln_to_EE);
    
        csv_new.open(name2);
        for (int jjj = 0; jjj < ITERATIONS; jjj++) {
            
            for(int k = 0; k<NJ; k++){
                q[k] = distribution(rng);
                Dq[k] = distribution(rng);
                Dqr[k] = distribution(rng);
                DDqr[k] = distribution(rng);
            }

            // ----------------------------------------------------------------- //
            start = std::chrono::high_resolution_clock::now();
            
            regrobot_new.setArguments(q, Dq, Dqr, DDqr);
            Yr_new = regrobot_new.getRegressor();
            
            end = std::chrono::high_resolution_clock::now();
            // ----------------------------------------------------------------- //
        
            duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
            csv_new << duration << "\n";
            if (jjj%100==0){std::cout<<jjj<<std::endl;}
        }
        csv_new.close();
    }

    if (SELECT == 2){
        regrobot_thunder.init(NJ);
        
        csv_thunder.open(name4);
        for (int jjj = 0; jjj < ITERATIONS; jjj++) {
            
            for(int k = 0; k<NJ; k++){
                q[k] = distribution(rng);
                Dq[k] = distribution(rng);
                Dqr[k] = distribution(rng);
                DDqr[k] = distribution(rng);
            }

            // ----------------------------------------------------------------- //
            start = std::chrono::high_resolution_clock::now();
            
            regrobot_thunder.setArguments(q, Dq, Dqr, DDqr);
            Yr_thunder = regrobot_thunder.getReg_gen();
            
            end = std::chrono::high_resolution_clock::now();
            // ----------------------------------------------------------------- //

            duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
            csv_thunder << duration << "\n";
            if (jjj%100==0){std::cout<<jjj<<std::endl;}
        }
        csv_thunder.close();
    }

    return 0;
}