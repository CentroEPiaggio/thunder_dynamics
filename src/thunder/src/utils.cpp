#include "utils.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

using namespace std;

namespace thunder_ns{

	int change_to_robot(const string from_robot, const string to_robot, int n_joints, const string file_path_h, const string file_path_cpp){

		// --- file .h --- //
		ifstream file_h(file_path_h); // open in reading mode
		if (!file_h.is_open()) {
			cerr << "error in file_h opening:" << file_path_h << endl;
			return 0;
		} else {
			stringstream buffer_h;
			buffer_h << file_h.rdbuf(); // read file_h on buffer_h
			string file_content_h = buffer_h.str(); // file_h as string

			file_h.close(); // close the file_h

			// - change num_joints - //
			string header = "THUNDERROBOT";
			size_t index_header = file_content_h.find(header);
			file_content_h.replace(index_header, header.length(), "THUNDER_" + to_robot);
			index_header = file_content_h.find(header);
			file_content_h.replace(index_header, header.length(), "THUNDER_" + to_robot);

			// - substitute 'from_robot' wiht 'to_robot' - //
			replace_all(file_content_h, from_robot, to_robot);

			// - overwrite file_h - //
			ofstream out_h(file_path_h);
			out_h << file_content_h;
			out_h.close();
		}

		// --- file .cpp --- //
		ifstream file_cpp(file_path_cpp); // open in reading mode
		if (!file_cpp.is_open()) {
			cerr << "error in file_cpp opening:" << file_path_h << endl;
			return 0;
		} else {
			stringstream buffer_cpp;
			buffer_cpp << file_cpp.rdbuf(); // read file_cpp on buffer_cpp
			string file_content_cpp = buffer_cpp.str(); // file_cpp as string

			file_cpp.close(); // close the file_cpp

			// - change num_joints - //
			size_t index_joints = file_content_cpp.find("N_JOINTS");
			size_t index_semicolon = file_content_cpp.find(';', index_joints);
			file_content_cpp.replace(index_joints, index_semicolon - index_joints, "N_JOINTS = " + to_string(n_joints));

			// - substitute 'from_robot' wiht 'to_robot' - //
			replace_all(file_content_cpp, from_robot, to_robot);

			// - overwrite file_cpp - //
			ofstream out_cpp(file_path_cpp);
			out_cpp << file_content_cpp;
			out_cpp.close();
		}

		return 1;
	}

	// string file_content_cpp = (string str, string from_str, string to_str){
	// 	// const int ROBOT_LEN = from_str.length();
	// 	// size_t index = str.find(from_str);
	// 	// cout<<"robot_len: "<<ROBOT_LEN<<endl;
	// 	// cout<<"initial index: "<<index<<endl;
		
	// 	// while ((index+ROBOT_LEN+1 <str.length()) && (index != -1)){
	// 	// 	str.replace(index, ROBOT_LEN, to_str);
	// 	// 	index += ROBOT_LEN;
	// 	// 	index = str.find(from_str, index);
	// 	// 	cout<<"index: "<<index<<endl;
	// 	// }
	// 	size_t start_pos;
	// 	while((start_pos = str.find(from_str, start_pos)) != string::npos) {
	// 		str.replace(start_pos, from_str.length(), to_str);
	// 		start_pos += to_str.length(); // Handles case where 'to_str' is a substring of 'from_str'
	// 	}
	// 	return str;
	// }

	// void file_content_cpp = (string &str, const string& from_str, const string& to_str){
	// 	size_t start_pos = 0;
	// 	start_pos = str.find(from_str, start_pos);
	// 	// cout<<"content_h: "<< str <<endl;
	// 	while((start_pos = str.find(from_str, start_pos)) != -1) {
	// 		cout<<"start_pos: "<<start_pos<<endl;
	// 		cout<<"from_str: "<<from_str<<", length: "<<from_str.length()<<endl;
	// 		cout<<"string: "<<str.substr(start_pos, from_str.length())<<endl;
	// 		str.replace(start_pos, from_str.length(), to_str);

	// 		start_pos += to_str.length();
	// 	}
	// 	// cout<<"content_h: "<< str <<endl;
	// }

	// string file_content_cpp = (string str, const string from_str, const string to_str){
	// 	size_t start_pos = 0;
	// 	while((start_pos = str.find(from_str, start_pos)) != string::npos) {
	// 		str.replace(start_pos, from_str.length(), to_str);
	// 		start_pos += to_str.length();
	// 	}
	// 	return str;
	// }

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

}
