#ifndef FRAMEOFFSET_H
#define FRAMEOFFSET_H

#include <iostream>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <stdexcept>

/* FrameOffset CLASS */

namespace regrob{

    /* Homogenous transformation of offset frame */
    class FrameOffset {

        private:

            /* [yaw, pitch, roll] */
            std::vector<double> ypr;
            /* [transl_x,transl_y,transl_z] */
            std::vector<double> translation;
            /* [grav_x,grav_y,grav_z] */
            std::vector<double> gravity;
            /* Operator hat */
            casadi::SX hat(const std::vector<double>&);

        public:

            /* Empty constructor */
            FrameOffset();
            /* Set [yaw,pitch,roll], [transl_x,transl_y,transl_z], [grav_x,grav_y,grav_z]
            All transformation is referred in world FrameOffset */
            FrameOffset(const std::vector<double>& ypr,const std::vector<double>& transl,const std::vector<double>& gravity);
            /* Set [yaw,pitch,roll], [transl_x,transl_y,transl_z]
            All transformation is referred in world FrameOffset */
            FrameOffset(const std::vector<double>& ypr,const std::vector<double>& trasl);
            /* Destructor */
            ~FrameOffset(){};
            
            /* Get translation vector of FrameOffset */
            void set_translation(const std::vector<double>& trasl);
            /* Set yaw, pitch, roll vector of FrameOffset */
            void set_ypr(const std::vector<double>& ypr);
            /* Set gravity of FrameOffset */
            void set_gravity(const std::vector<double>& gravity);

            /* Get yaw, pitch, roll vector of FrameOffset */
            std::vector<double> get_ypr();
            /* Get rotation matrix of FrameOffset */
            casadi::SX get_rotation();
            /* Get translation vector of FrameOffset */
            casadi::SX get_translation();
            /* Get homogeneous transformation matrix of FrameOffset */
            casadi::SX get_transform();
            /* Get gravity of FrameOffset */
            casadi::SX get_gravity();

    };
}
#endif