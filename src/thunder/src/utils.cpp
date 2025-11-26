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

	std::string get_ret_type(const fun_obj fun){
		std::vector<int> out_size = fun.out_size;
		string ret_type = "Eigen::Matrix<double,"+to_string(out_size[0])+","+to_string(out_size[1])+">";
		return ret_type;
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

	Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
		
		Eigen::Matrix3d skew(3, 3);
		skew.setZero();
		
		skew(0, 1) = -v(2);
		skew(0, 2) = v(1);
		skew(1, 0) = v(2);
		skew(1, 2) = -v(0);
		skew(2, 0) = -v(1);
		skew(2, 1) = v(0);
		
		return skew;
	}

}
