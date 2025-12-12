#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

#include "../include/utils.h"
#include "../include/robot.h"

using namespace std;

namespace thunder_ns{

	// do not call replace often
	void replace_all(std::string& source, const std::string& from_str, const std::string& to_str){
		std::string newString;
		newString.reserve(source.length());  // avoids a few memory allocations

		std::string::size_type lastPos = 0;
		std::string::size_type findPos;

		while(std::string::npos != (findPos = source.find(from_str, lastPos))){
			newString.append(source, lastPos, findPos - lastPos);
			newString.append(to_str);
			lastPos = findPos + from_str.length();
		}

		// Care for the rest after last occurrence
		// newString += source.substr(lastPos);
		newString.append(source.substr(lastPos));

		source.swap(newString);
		// return newString;
	}

	casadi::SX hat(const casadi::SX& v) {
		
		casadi::SX skew = casadi::SX::zeros(3,3);
		
		skew(0, 1) = -v(2);
		skew(0, 2) = v(1);
		skew(1, 0) = v(2);
		skew(1, 2) = -v(0);
		skew(2, 0) = -v(1);
		skew(2, 1) = v(0);
		
		return skew;
	}

	// - set to zero small values (Zero If Small) - //
	casadi::SX ZIS(const casadi::SX& x, double tol) {
		// Check if x is numeric and its absolute value is smaller than tolerance
		if (x.is_constant() && std::abs(static_cast<double>(x)) < tol)
			return casadi::SX(0);
		else
			return x;
	}

	// - Rotations - //
	casadi::SX R_x(const casadi::SX& angle) {
		casadi::SX R = casadi::SX::zeros(3, 3); // Create a 3x3 zero matrix

		// Define the rotation matrix components for rotation around the x-axis
		R(0, 0) = 1;        R(0, 1) = 0;                	R(0, 2) = 0;
		R(1, 0) = 0;        R(1, 1) = ZIS(cos(angle));      R(1, 2) = ZIS(-sin(angle));
		R(2, 0) = 0;        R(2, 1) = ZIS(sin(angle));      R(2, 2) = ZIS(cos(angle));

		return R;
	}
	casadi::SX R_y(const casadi::SX& angle) {
		casadi::SX R = casadi::SX::zeros(3, 3); // Create a 3x3 zero matrix

		// Define the rotation matrix components for rotation around the y-axis
		R(0, 0) = ZIS(cos(angle));      R(0, 1) = 0;		R(0, 2) = ZIS(sin(angle));
		R(1, 0) = 0;                	R(1, 1) = 1;    	R(1, 2) = 0;
		R(2, 0) = ZIS(-sin(angle));     R(2, 1) = 0;    	R(2, 2) = ZIS(cos(angle));

		return R;
	}

	// Function to create a rotation matrix for a given angle about the z-axis
	casadi::SX R_z(const casadi::SX& angle) {
		// Ensure the input is of type casadi::SX
		casadi::SX R = casadi::SX::zeros(3, 3); // Create a 3x3 zero matrix

		// Define the rotation matrix components for rotation around the z-axis
		R(0, 0) = ZIS(cos(angle));      R(0, 1) = ZIS(-sin(angle));     R(0, 2) = 0;
		R(1, 0) = ZIS(sin(angle));      R(1, 1) = ZIS(cos(angle));      R(1, 2) = 0;
		R(2, 0) = 0;                	R(2, 1) = 0;                	R(2, 2) = 1;

		return R;
	}

	// Get the transformation matrix from frame parameters (xyz rpy)
	casadi::SX get_transform_rpy(casadi::SX frame_rpy){
		// traslation -> xyz, rotation -> yaw-pitch-roll
		casadi::SX T(4,4); // output
		casadi::SX R(3,3);
		casadi::SX p(3,1);
		casadi::Slice idx(0, 3);      // [0,1,2] indexes

		p = frame_rpy(idx);
		casadi::SX phi = frame_rpy(3);
		casadi::SX theta = frame_rpy(4);
		casadi::SX psi = frame_rpy(5);
		R = casadi::SX::mtimes(casadi::SX::mtimes(R_x(phi), R_y(theta)), R_z(psi));

		T(idx,idx) = R;
		T(idx,3) = p;
		T(3,3) = 1;

		return T;
	}

	// Get the transformation matrix from frame parameters (xyz ypr)
	casadi::SX get_transform_ypr(casadi::SX frame_ypr){
		// traslation -> xyz, rotation -> yaw-pitch-roll
		casadi::SX T(4,4); // output
		casadi::SX R(3,3);
		casadi::SX p(3,1);
		casadi::Slice idx(0, 3);      // [0,1,2] indexes

		p = frame_ypr(idx);
		casadi::SX psi = frame_ypr(3);
		casadi::SX theta = frame_ypr(4);
		casadi::SX phi = frame_ypr(5);
		R = casadi::SX::mtimes(casadi::SX::mtimes(R_z(psi), R_y(theta)), R_x(phi));

		T(idx,idx) = R;
		T(idx,3) = p;
		T(3,3) = 1;

		return T;
	}

}
