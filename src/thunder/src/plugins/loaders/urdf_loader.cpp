#include "plugins/loaders/urdf_loader.h"
#include "utils.h"
#include <algorithm>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <functional>
#include <cctype>

namespace {

static std::string trim(const std::string& s) {
	const auto is_space = [](unsigned char c) { return std::isspace(c); };
	const auto l = std::find_if_not(s.begin(), s.end(), is_space);
	const auto r = std::find_if_not(s.rbegin(), s.rend(), is_space).base();
	return (l < r) ? std::string(l, r) : std::string();
}

static std::vector<int> parseInts(const std::string& s) {
	std::vector<int> out;
	std::istringstream iss(s);
	std::string token;
	while (iss >> token) {
		// Support both numeric (0/1) and boolean words (true/false).
		switch (token.size()) {
			case 1:
				if (token == "0") { out.push_back(0); continue; }
				if (token == "1") { out.push_back(1); continue; }
				break;
		}
		std::string token_lc = token;
		std::transform(token_lc.begin(), token_lc.end(), token_lc.begin(), [](unsigned char c){ return std::tolower(c); });
		if (token_lc == "true" || token_lc == "yes" || token_lc == "y") {
			out.push_back(1);
			continue;
		} else if (token_lc == "false" || token_lc == "no" || token_lc == "n") {
			out.push_back(0);
			continue;
		}
		try {
			int v = std::stoi(token);
			out.push_back(v);
		} catch (...) {
			// ignore non-integer tokens
		}
	}
	return out;
}

static short yamlScalarToBinary(const YAML::Node& node, short default_value) {
	if (!node || !node.IsScalar()) return default_value;
	// Try boolean-like scalars first (true/false/0/1)
	std::string s = node.as<std::string>();
	std::string sl = s;
	std::transform(sl.begin(), sl.end(), sl.begin(), [](unsigned char c){ return std::tolower(c); });
	if (sl == "0" || sl == "false" || sl == "no" || sl == "n") return 0;
	if (sl == "1" || sl == "true" || sl == "yes" || sl == "y") return 1;
	// Fallback to numeric conversion
	try {
		int v = std::stoi(s);
		return v != 0 ? 1 : 0;
	} catch (...) {
	}
	return default_value;
}

static std::vector<short> toBinary(const std::vector<int>& vals, int expected, short default_value) {
	std::vector<short> out(expected, default_value);
	for (int i = 0; i < (int)std::min((int)vals.size(), expected); ++i) {
		out[i] = vals[i] != 0 ? 1 : 0;
	}
	return out;
}

static bool extractXmlTag(const std::string& text, const std::string& tag, std::string& attributes, std::string& inner, bool& self_closing) {
	const std::string open = "<" + tag;
	size_t p = text.find(open);
	if (p == std::string::npos) return false;
	size_t gt = text.find('>', p);
	if (gt == std::string::npos) return false;
	attributes = text.substr(p + open.size(), gt - (p + open.size()));
	// detect "<tag ... />" even if there is whitespace before the '/'
	self_closing = false;
	if (gt > 0) {
		size_t scan = gt - 1;
		while (scan > 0 && std::isspace(static_cast<unsigned char>(text[scan]))) {
			scan--;
		}
		if (text[scan] == '/') {
			self_closing = true;
		}
	}
	if (self_closing) {
		inner.clear();
		return true;
	}
	const std::string close = "</" + tag + ">";
	size_t close_pos = text.find(close, gt + 1);
	if (close_pos == std::string::npos) return false;
	inner = text.substr(gt + 1, close_pos - (gt + 1));
	return true;
}

static bool extractXmlElement(const std::string& text, const std::string& element, std::string& out) {
	const std::string open = "<" + element + ">";
	const std::string close = "</" + element + ">";
	size_t p = text.find(open);
	if (p == std::string::npos) return false;
	size_t start = p + open.size();
	size_t end = text.find(close, start);
	if (end == std::string::npos) return false;
	out = text.substr(start, end - start);
	return true;
}

static std::string extractXmlAttribute(const std::string& attributes, const std::string& name) {
	const std::string key = name + "=\"";
	size_t p = attributes.find(key);
	if (p == std::string::npos) return "";
	size_t start = p + key.size();
	size_t end = attributes.find('"', start);
	if (end == std::string::npos) return "";
	return attributes.substr(start, end - start);
}

static std::vector<short> parseKinematicSymbolicFromXml(const std::string& xml, short global_default) {
	std::string attributes, inner;
	bool self_closing;
	if (!extractXmlTag(xml, "symbolic_kinematics", attributes, inner, self_closing)) return {};

	std::vector<short> out(6, global_default);

	// <symbolic_kinematics xyz="..." rpy="..." />
	auto xyz_attr = extractXmlAttribute(attributes, "xyz");
	auto rpy_attr = extractXmlAttribute(attributes, "rpy");
	if (!xyz_attr.empty() || !rpy_attr.empty()) {
		if (!xyz_attr.empty()) {
			auto v = parseInts(xyz_attr);
			auto b = toBinary(v, 3, global_default);
			for (int i = 0; i < 3; ++i) out[i] = b[i];
		}
		if (!rpy_attr.empty()) {
			auto v = parseInts(rpy_attr);
			auto b = toBinary(v, 3, global_default);
			for (int i = 0; i < 3; ++i) out[3 + i] = b[i];
		}
		return out;
	}

	// <symbolic_kinematics><xyz>...</xyz><rpy>...</rpy></symbolic_kinematics>
	std::string xyz_inner, rpy_inner;
	if (extractXmlElement(inner, "xyz", xyz_inner) || extractXmlElement(inner, "XYZ", xyz_inner)) {
		auto v = parseInts(xyz_inner);
		auto b = toBinary(v, 3, global_default);
		for (int i = 0; i < 3; ++i) out[i] = b[i];
	}
	if (extractXmlElement(inner, "rpy", rpy_inner) || extractXmlElement(inner, "RPY", rpy_inner)) {
		auto v = parseInts(rpy_inner);
		auto b = toBinary(v, 3, global_default);
		for (int i = 0; i < 3; ++i) out[3 + i] = b[i];
	}
	if (!xyz_inner.empty() || !rpy_inner.empty()) {
		return out;
	}

	// <symbolic_kinematics>0 0 1 0 0 0</symbolic_kinematics>
	auto v = parseInts(inner);
	if (!v.empty()) {
		auto b = toBinary(v, 6, global_default);
		return b;
	}

	return {};
}

static std::vector<short> parseDynamicSymbolicFromXml(const std::string& xml, short global_default) {
	std::string attributes, inner;
	bool self_closing;
	if (!extractXmlTag(xml, "symbolic_dynamics", attributes, inner, self_closing)) return {};

	std::vector<short> out(10, global_default);

	// <symbolic_dynamics mass="..." com="..." inertia="..." />
	auto mass_attr = extractXmlAttribute(attributes, "mass");
	auto com_attr = extractXmlAttribute(attributes, "com");
	if (com_attr.empty()) com_attr = extractXmlAttribute(attributes, "CoM");
	auto inertia_attr = extractXmlAttribute(attributes, "inertia");
	if (inertia_attr.empty()) inertia_attr = extractXmlAttribute(attributes, "I");

	if (!mass_attr.empty() || !com_attr.empty() || !inertia_attr.empty()) {
		if (!mass_attr.empty()) {
			auto v = parseInts(mass_attr);
			out[0] = (!v.empty() && v[0] != 0) ? 1 : 0;
		}
		if (!com_attr.empty()) {
			auto v = parseInts(com_attr);
			auto b = toBinary(v, 3, global_default);
			for (int i = 0; i < 3; ++i) out[1 + i] = b[i];
		}
		if (!inertia_attr.empty()) {
			auto v = parseInts(inertia_attr);
			auto b = toBinary(v, 6, global_default);
			for (int i = 0; i < 6; ++i) out[4 + i] = b[i];
		}
		return out;
	}

	// <symbolic_dynamics><mass>...</mass><com>...</com><inertia>...</inertia></symbolic_dynamics>
	std::string mass_inner, com_inner, inertia_inner;
	if (extractXmlElement(inner, "mass", mass_inner)) {
		auto v = parseInts(mass_inner);
		out[0] = (!v.empty() && v[0] != 0) ? 1 : 0;
	}
	if (extractXmlElement(inner, "com", com_inner) || extractXmlElement(inner, "CoM", com_inner)) {
		auto v = parseInts(com_inner);
		auto b = toBinary(v, 3, global_default);
		for (int i = 0; i < 3; ++i) out[1 + i] = b[i];
	}
	if (extractXmlElement(inner, "inertia", inertia_inner) || extractXmlElement(inner, "I", inertia_inner)) {
		auto v = parseInts(inertia_inner);
		auto b = toBinary(v, 6, global_default);
		for (int i = 0; i < 6; ++i) out[4 + i] = b[i];
	}
	
	// If inner text only contains 10 numbers
	auto v = parseInts(inner);
	if (v.size() >= 10) {
		auto b = toBinary(v, 10, global_default);
		return b;
	}

	// If we have some values, return what we have (others remain default)
	bool any = false;
	for (auto x : out) if (x != global_default) { any = true; break; }
	return any ? out : std::vector<short>();
}

static std::unordered_map<std::string, std::vector<short>> parseSymbolicFromUrdf(const std::string& urdf_text,
	const std::string& tag,
	int expected_size,
	short global_default,
	const std::function<std::vector<short>(const std::string&, short)>& parser) {
	std::unordered_map<std::string, std::vector<short>> result;
	size_t pos = 0;
	while (true) {
		size_t link_pos = urdf_text.find("<link", pos);
		if (link_pos == std::string::npos) break;
		size_t name_pos = urdf_text.find("name=\"", link_pos);
		if (name_pos == std::string::npos) break;
		size_t name_start = name_pos + 6;
		size_t name_end = urdf_text.find('"', name_start);
		if (name_end == std::string::npos) break;
		std::string link_name = urdf_text.substr(name_start, name_end - name_start);

		size_t link_tag_end = urdf_text.find('>', name_end);
		if (link_tag_end == std::string::npos) break;
		size_t close_pos = urdf_text.find("</link>", link_tag_end);
		if (close_pos == std::string::npos) break;
		std::string link_body = urdf_text.substr(link_tag_end + 1, close_pos - (link_tag_end + 1));

		auto vec = parser(link_body, global_default);
		if (!vec.empty()) {
			if ((int)vec.size() == expected_size) {
				result[link_name] = vec;
			} else {
				// If the parser returned a smaller vector, pad with global_default
				std::vector<short> padded(expected_size, global_default);
				for (int i = 0; i < (int)vec.size() && i < expected_size; ++i) {
					padded[i] = vec[i];
				}
				result[link_name] = padded;
			}
		}
		pos = close_pos + 7;
	}
	return result;
}

static std::unordered_map<std::string, std::vector<short>> parseSymbolicKinematicsFromUrdfJoints(const std::string& urdf_text, short global_default) {
	std::unordered_map<std::string, std::vector<short>> result;
	size_t pos = 0;
	while (true) {
		size_t joint_pos = urdf_text.find("<joint", pos);
		if (joint_pos == std::string::npos) break;
		size_t joint_end = urdf_text.find('>', joint_pos);
		if (joint_end == std::string::npos) break;

		// Determine joint block boundaries
		size_t close_pos = urdf_text.find("</joint>", joint_end);
		size_t block_end = (close_pos == std::string::npos) ? joint_end + 1 : close_pos + 8;
		std::string joint_block = urdf_text.substr(joint_pos, block_end - joint_pos);

		// Extract child link name
		std::string child_link;
		size_t child_pos = joint_block.find("<child");
		if (child_pos != std::string::npos) {
			size_t name_pos = joint_block.find("link=\"", child_pos);
			if (name_pos != std::string::npos) {
				name_pos += 6;
				size_t name_end = joint_block.find('"', name_pos);
				if (name_end != std::string::npos) {
					child_link = joint_block.substr(name_pos, name_end - name_pos);
				}
			}
		}

		if (!child_link.empty()) {
			auto vec = parseKinematicSymbolicFromXml(joint_block, global_default);
			if (!vec.empty()) {
				result[child_link] = vec;
			}
		}

		pos = block_end;
	}
	return result;
}

static std::unordered_map<std::string, std::vector<short>> parseKinematicSymbolicFromYaml(const YAML::Node& node, short global_default) {
	std::unordered_map<std::string, std::vector<short>> result;
	if (!node || !node.IsMap()) return result;
	for (auto it = node.begin(); it != node.end(); ++it) {
		const std::string key = it->first.as<std::string>();
		if (key == "default") continue;
		auto val = it->second;
		std::vector<short> mask(6, global_default);
		if (val.IsSequence()) {
			// Accept sequences of 0/1 or true/false
			std::vector<int> v;
			for (auto item : val) {
				v.push_back(yamlScalarToBinary(item, global_default));
			}
			mask = toBinary(v, 6, global_default);
		} else if (val.IsScalar()) {
			short b = yamlScalarToBinary(val, global_default);
			mask = toBinary(std::vector<int>{b, b, b, b, b, b}, 6, global_default);
		} else if (val.IsMap()) {
			if (val["xyz"]) {
				if (val["xyz"].IsSequence()) {
					std::vector<int> v;
					for (auto item : val["xyz"]) {
						v.push_back(yamlScalarToBinary(item, global_default));
					}
					auto b = toBinary(v, 3, global_default);
					for (int i = 0; i < 3; ++i) mask[i] = b[i];
				} else if (val["xyz"].IsScalar()) {
					short b = yamlScalarToBinary(val["xyz"], global_default);
					auto bv = toBinary(std::vector<int>{b, b, b}, 3, global_default);
					for (int i = 0; i < 3; ++i) mask[i] = bv[i];
				}
			}
			if (val["rpy"]) {
				if (val["rpy"].IsSequence()) {
					std::vector<int> v;
					for (auto item : val["rpy"]) {
						v.push_back(yamlScalarToBinary(item, global_default));
					}
					auto b = toBinary(v, 3, global_default);
					for (int i = 0; i < 3; ++i) mask[3 + i] = b[i];
				} else if (val["rpy"].IsScalar()) {
					short b = yamlScalarToBinary(val["rpy"], global_default);
					auto bv = toBinary(std::vector<int>{b, b, b}, 3, global_default);
					for (int i = 0; i < 3; ++i) mask[3 + i] = bv[i];
				}
			}
		}
		result[key] = mask;
	}
	return result;
}

static std::unordered_map<std::string, std::vector<short>> parseDynamicSymbolicFromYaml(const YAML::Node& node, short global_default) {
	std::unordered_map<std::string, std::vector<short>> result;
	if (!node || !node.IsMap()) return result;
	for (auto it = node.begin(); it != node.end(); ++it) {
		const std::string key = it->first.as<std::string>();
		if (key == "default") continue;
		auto val = it->second;
		std::vector<short> mask(10, global_default);
		if (val.IsSequence()) {
			// Accept sequences of 0/1 or true/false
			std::vector<int> v;
			for (auto item : val) {
				v.push_back(yamlScalarToBinary(item, global_default));
			}
			mask = toBinary(v, 10, global_default);
		} else if (val.IsScalar()) {
			short b = yamlScalarToBinary(val, global_default);
			mask = toBinary(std::vector<int>{b, b, b, b, b, b, b, b, b, b}, 10, global_default);
		} else if (val.IsMap()) {
			if (val["mass"]) {
				if (val["mass"].IsScalar()) {
					short b = yamlScalarToBinary(val["mass"], global_default);
					mask[0] = (b != 0) ? 1 : 0;
				} else if (val["mass"].IsSequence()) {
					auto v = val["mass"];
					if (v.IsSequence()) {
						std::vector<int> vv;
						for (auto item : v) vv.push_back(yamlScalarToBinary(item, global_default));
						mask[0] = (!vv.empty() && vv[0] != 0) ? 1 : 0;
					}
				}
			}
			if (val["CoM"] || val["com"]) {
				auto n = val["CoM"] ? val["CoM"] : val["com"];
				if (n.IsSequence()) {
					std::vector<int> v;
					for (auto item : n) {
						v.push_back(yamlScalarToBinary(item, global_default));
					}
					auto b = toBinary(v, 3, global_default);
					for (int i = 0; i < 3; ++i) mask[1 + i] = b[i];
				} else if (n.IsScalar()) {
					short b = yamlScalarToBinary(n, global_default);
					auto bv = toBinary(std::vector<int>{b, b, b}, 3, global_default);
					for (int i = 0; i < 3; ++i) mask[1 + i] = bv[i];
				}
			}
			if (val["I"] || val["inertia"]) {
				auto n = val["I"] ? val["I"] : val["inertia"];
				if (n.IsSequence()) {
					std::vector<int> v;
					for (auto item : n) {
						v.push_back(yamlScalarToBinary(item, global_default));
					}
					auto b = toBinary(v, 6, global_default);
					for (int i = 0; i < 6; ++i) mask[4 + i] = b[i];
				} else if (n.IsScalar()) {
					short b = yamlScalarToBinary(n, global_default);
					auto bv = toBinary(std::vector<int>{b, b, b, b, b, b}, 6, global_default);
					for (int i = 0; i < 6; ++i) mask[4 + i] = bv[i];
				}
			}
		}
		result[key] = mask;
	}
	return result;
}

static std::string readFileToString(const std::string& path) {
	std::ifstream ifs(path);
	if (!ifs) return std::string();
	std::ostringstream ss;
	ss << ifs.rdbuf();
	return ss.str();
}

} // namespace

namespace thunder_ns {

	void UrdfLoader::accumulateChain(std::shared_ptr<urdf::Link> link, const std::string& base, std::vector<std::shared_ptr<urdf::Link>>& chain) {
		while (link && link->name != base) {
			chain.push_back(link);
			link = link->getParent();
		}
		if (link && link->name == base) {
			chain.push_back(link);
		}
	}

	casadi::SX UrdfLoader::to_casadi_sx(const urdf::Transform& T) {
		casadi::SX R = casadi::SX::zeros(4, 4);
		Eigen::Matrix4d M = T.matrix();
		for (int r = 0; r < 4; ++r) {
			for (int c = 0; c < 4; ++c) {
				R(r, c) = M(r, c);
			}
		}
		return R;
	}

	// Build a 6D frame parameterization as [x, y, z, r, p, y].
	// Supported formats are xyzrpy, xyz+rpy, xyz+ypr, and legacy tr+ypr.
	void UrdfLoader::parse_frame_parameterization(
		std::shared_ptr<Robot> robot,
		const YAML::Node& frame_node,
		const std::string& frame_prefix,
		const std::vector<double>& default_xyzrpy,
		const std::vector<short>& default_symb,
		casadi::SX& frame_expr,
		std::vector<std::string>& frame_args,
		const std::string& description_prefix) {

		if (default_xyzrpy.size() != 6 || default_symb.size() != 6) {
			throw std::runtime_error("Invalid defaults for frame parameterization helper.");
		}

		std::vector<double> xyz_default(default_xyzrpy.begin(), default_xyzrpy.begin() + 3);
		std::vector<double> rpy_default(default_xyzrpy.begin() + 3, default_xyzrpy.end());

		std::vector<short> frame_isSymb = default_symb;
		if (frame_node && frame_node["symb"]) {
			frame_isSymb = frame_node["symb"].as<std::vector<short>>();
			if (frame_isSymb.size() != 6) {
				throw std::runtime_error("'" + frame_prefix + ".symb' must have 6 elements.");
			}
		}

		if (frame_node && frame_node["xyzrpy"]) {
			auto xyzrpy_num = frame_node["xyzrpy"].as<std::vector<double>>();
			if (xyzrpy_num.size() != 6) {
				throw std::runtime_error("'" + frame_prefix + ".xyzrpy' must have 6 elements.");
			}
			const std::string xyzrpy_name = frame_prefix + "_xyzrpy";
			casadi::SX xyzrpy_symb = casadi::SX::sym(xyzrpy_name, 6);
			robot->add_parameter(xyzrpy_name, xyzrpy_symb, xyzrpy_num, frame_isSymb, description_prefix + " in xyzrpy", true);
			frame_expr = robot->get_model(xyzrpy_name);
			frame_args = {xyzrpy_name};
			return;
		}

		bool has_xyz = frame_node && (frame_node["xyz"] || frame_node["tr"]);
		auto xyz_num = xyz_default;
		if (has_xyz) {
			if (frame_node["xyz"]) xyz_num = frame_node["xyz"].as<std::vector<double>>();
			else xyz_num = frame_node["tr"].as<std::vector<double>>();
			if (xyz_num.size() != 3) {
				throw std::runtime_error("'" + frame_prefix + ".xyz' (or '.tr') must have 3 elements.");
			}
		}

		std::vector<short> xyz_isSymb(frame_isSymb.begin(), frame_isSymb.begin() + 3);
		std::vector<short> or_isSymb(frame_isSymb.begin() + 3, frame_isSymb.end());

		const std::string xyz_name = frame_prefix + "_xyz";
		casadi::SX xyz_symb = casadi::SX::sym(xyz_name, 3);
		robot->add_parameter(xyz_name, xyz_symb, xyz_num, xyz_isSymb, description_prefix + " translation", true);

		if (frame_node && frame_node["rpy"]) {
			auto rpy_num = frame_node["rpy"].as<std::vector<double>>();
			if (rpy_num.size() != 3) {
				throw std::runtime_error("'" + frame_prefix + ".rpy' must have 3 elements.");
			}
			const std::string rpy_name = frame_prefix + "_rpy";
			casadi::SX rpy_symb = casadi::SX::sym(rpy_name, 3);
			robot->add_parameter(rpy_name, rpy_symb, rpy_num, or_isSymb, description_prefix + " orientation in rpy", true);

			casadi::SXVector frame_parts(2);
			frame_parts[0] = robot->get_model(xyz_name);
			frame_parts[1] = robot->get_model(rpy_name);
			frame_expr = casadi::SX::vertcat(frame_parts);
			frame_args = {xyz_name, rpy_name};
			return;
		}

		if (frame_node && frame_node["ypr"]) {
			auto ypr_num = frame_node["ypr"].as<std::vector<double>>();
			if (ypr_num.size() != 3) {
				throw std::runtime_error("'" + frame_prefix + ".ypr' must have 3 elements.");
			}
			const std::string ypr_name = frame_prefix + "_ypr";
			casadi::SX ypr_symb = casadi::SX::sym(ypr_name, 3);
			robot->add_parameter(ypr_name, ypr_symb, ypr_num, or_isSymb, description_prefix + " orientation in ypr", true);

			casadi::SX ypr_frame = casadi::SX::zeros(6, 1);
			ypr_frame(casadi::Slice(3, 6)) = robot->get_model(ypr_name);
			casadi::SX rpy_from_ypr = get_euler_rpy(get_transform_ypr(ypr_frame));

			casadi::SXVector frame_parts(2);
			frame_parts[0] = robot->get_model(xyz_name);
			frame_parts[1] = rpy_from_ypr;
			frame_expr = casadi::SX::vertcat(frame_parts);
			frame_args = {xyz_name, ypr_name};
			return;
		}

		const std::string rpy_name = frame_prefix + "_rpy";
		casadi::SX rpy_symb = casadi::SX::sym(rpy_name, 3);
		robot->add_parameter(rpy_name, rpy_symb, rpy_default, or_isSymb, description_prefix + " orientation in rpy", true);

		casadi::SXVector frame_parts(2);
		frame_parts[0] = robot->get_model(xyz_name);
		frame_parts[1] = robot->get_model(rpy_name);
		frame_expr = casadi::SX::vertcat(frame_parts);
		frame_args = {xyz_name, rpy_name};
	}

	casadi::DM UrdfLoader::extractKinematicsFromJoint(std::shared_ptr<urdf::Joint> joint) {
		// extract position from joints?
		casadi::DM T_pj = to_casadi_sx(joint->parent_to_joint_transform);
		
		// extract rpy
		casadi::DM rpy = get_euler_rpy(T_pj);

		// extract translation
		casadi::DM xyz = T_pj(casadi::Slice(0,3), 3);

		return casadi::DM::vertcat({xyz, rpy});
	}

	casadi::DM UrdfLoader::extractInertiaFromLink(std::shared_ptr<urdf::Link> link) {
		if (!link->inertial) return casadi::SX::zeros(10,1); // Zero inertia

		casadi::DM m = link->inertial->mass;
		casadi::DM T_li = to_casadi_sx(link->inertial->origin);
		casadi::Slice first3(0, 3);
		casadi::DM CoM = T_li(first3, 3);

		casadi::DM Icom = casadi::SX::zeros(6, 1);
		Icom(0) = link->inertial->ixx;
		Icom(1) = link->inertial->ixy;
		Icom(2) = link->inertial->ixz;
		Icom(3) = link->inertial->iyy;
		Icom(4) = link->inertial->iyz;
		Icom(5) = link->inertial->izz;

		return casadi::DM::vertcat({m, CoM, Icom});
	}

	void UrdfLoader::reset_thunder_chain() {
		numJoints = 0;
		ndof = 0;
		jointsName.resize(0);
		jointsParent.resize(0);
		jointsType.resize(0, "FIXED");
		jointsDimension.resize(0, 0);
		jointsAvailable.resize(0, false);
		jointsDerivatives.resize(0, false);
		jointsAxis.resize(0, {0,0,1});
		par_KIN_num.resize(0, 0);
		par_DYN_num.resize(0, 0);
	}

	bool UrdfLoader::chain_has_link(string link_name) {
		bool has_chain = false;
		for (int i=0; i<chain.size(); i++) {
			if (link_name == chain[i]->name) has_chain = true;
		}
		return has_chain;
	}

	void UrdfLoader::add_chain_from(int parent, std::shared_ptr<urdf::Link> link) {
		// - Fill thunder structures - //
		numJoints++;
		int link_id = jointsName.size();
		jointsName.push_back(link->name);
		jointsParent.push_back(parent);
		parent = link_id++;					// new link index
		auto par_DYN_link = extractInertiaFromLink(link);
		for(int j=0; j<DYN_DIM; j++) par_DYN_num.push_back(static_cast<double>(par_DYN_link(j,0)));

		if ((link->child_joints.empty()) || (ee_link == link)) {		// link is an end-effector
			debug_log("add_chain_from: treating link '" + link->name + "' as terminal/fixed node", VERB_DEBUG);
			jointsType.push_back("FIXED");
			jointsAxis.push_back({0,0,0});
			jointsDimension.push_back(0);
			jointsAvailable.push_back(true);
			jointsDerivatives.push_back(true);
			for(int j=0; j<KIN_DIM; j++) par_KIN_num.push_back(0);
		} else {								// have to explore deeply
			// add kinematic properties for joints
			int i = 0;
			for (auto joint : link->child_joints) {
				if (chain_has_link(joint->child_link_name)) {
					if (i > 0) {
						// Initialize ghost node for branches
						// cout << "_" + std::to_string(i);
						link_id++;
						numJoints++;
						jointsName.push_back(link->name + "_" + std::to_string(i));
						jointsParent.push_back(parent); 							// Attached to same parent as main node
						for(int j=0; j<DYN_DIM; j++) par_DYN_num.push_back(0);		// fictitious link
					}
					// joints origin
					jointsAvailable.push_back(false);
					jointsDerivatives.push_back(false);
					// extract kinematics from joint position
					auto par_KIN_link = extractKinematicsFromJoint(joint);
					for(int j=0; j<KIN_DIM; j++) par_KIN_num.push_back(static_cast<double>(par_KIN_link(j,0)));
					add_joint(joint);
					// cout << endl;
					// add childrens
					if (chain_has_link(joint->child_link_name)) {
						add_chain_from(parent, urdf_model->getLink(joint->child_link_name));
					}
					i++;
				}
			}
		}
	}

	void UrdfLoader::add_joint(std::shared_ptr<urdf::Joint> joint) {
		switch (joint->type) {
			case urdf::JointType::FIXED:
				jointsType.push_back("FIXED");
				jointsDimension.push_back(0);
				jointsAxis.push_back({0,0,0});
				break;
			case urdf::JointType::REVOLUTE:
				jointsType.push_back("R");
				jointsDimension.push_back(1);
				jointsAxis.push_back({joint->axis.x(), joint->axis.y(), joint->axis.z()});
				break;
			case urdf::JointType::CONTINUOUS:
				jointsType.push_back("R");
				jointsDimension.push_back(1);
				jointsAxis.push_back({joint->axis.x(), joint->axis.y(), joint->axis.z()});
				break;
			case urdf::JointType::PRISMATIC:
				jointsType.push_back("P");
				jointsDimension.push_back(1);
				jointsAxis.push_back({joint->axis.x(), joint->axis.y(), joint->axis.z()});
				break;
			case urdf::JointType::FLOATING:
				jointsType.push_back("F");
				// panic
				break;
			case urdf::JointType::PLANAR:
				jointsType.push_back("XY");
				jointsDimension.push_back(2);
				jointsAxis.push_back({0,0,0});
				// panic
				break;
			default:
				debug_log("Detected non-standard joint type for joint '" + joint->name + "'", VERB_INFO);
				jointsType.push_back("UNKNOWN");
				jointsDimension.push_back(0);
				jointsAxis.push_back({0,0,0});
				break;
		}
		ndof += jointsDimension[jointsDimension.size()-1];
	}

	std::shared_ptr<Robot> UrdfLoader::load(std::shared_ptr<Robot> robot) {
		debug_log("URDF Loading started", VERB_INFO);

		try {
			// --- Get configuration --- //
			if (!config_["urdf_path"]) {
				throw std::runtime_error("Missing 'urdf_path' in configuration.");
			}
			auto urdf_path_str = config_["urdf_path"].as<std::string>();
			std::filesystem::path urdf_path(urdf_path_str);
			

			// if the path is relative, we try to resolve it using the config file directory
			if (!urdf_path.is_absolute()) {
				if (config_["config_path"]) {
					// strip the filename, as this ends with the .yaml file
					auto config_dir = std::filesystem::path(config_["config_path"].as<std::string>()).parent_path();
					// resolve wrt config dir	
					urdf_path = config_dir / urdf_path;
					debug_log("Relative URDF path resolved to: " + urdf_path.string(), VERB_DEBUG);
				} else {
					debug_log("URDF path is relative: " + urdf_path_str + " but no config path found. Using current working directory.", VERB_INFO);
					urdf_path = std::filesystem::absolute(urdf_path);
				}
			}

			if (!std::filesystem::exists(urdf_path)) {
				throw std::runtime_error("URDF file not found: " + urdf_path.string());
			}
			
			auto urdf_path_final = urdf_path.string();
			
			auto configured_base_link_name = config_["base_link"] ? config_["base_link"].as<std::string>() : "base_link";

			debug_log("Loading URDF from: " + urdf_path_final, VERB_INFO);
			debug_log("Configured base link: " + configured_base_link_name, VERB_INFO);

			// --- Load URDF --- //
			try {
				urdf_model = urdf::UrdfModel::fromUrdfFile(urdf_path_final.c_str());
			} catch (const std::exception& e) {
				std::cerr << "Failed to load URDF: " << e.what() << std::endl;
				return robot;
			}

			if (!urdf_model) {
				std::cerr << "Failed to load URDF (null model)" << std::endl;
				return robot;
			}

			if (config_["base_link"]){
				try{
					root_link = urdf_model->getLink(config_["base_link"].as<std::string>());
					if (!root_link) {
						std::cerr << "Base link '" << configured_base_link_name << "' not found in URDF." << std::endl;
						return robot;
					}
				} catch (const std::exception& e) {
					std::cerr << "Base link '" << configured_base_link_name << "' not found in URDF: " << e.what() << std::endl;
					return robot;
				}
			} else {
				root_link = urdf_model->getRoot();
			}

			if (!root_link) {
				std::cerr << "Failed to resolve root/base link from URDF." << std::endl;
				return robot;
			}

			const std::string resolved_base_link_name = root_link->name;
			debug_log("Resolved base link: " + resolved_base_link_name, VERB_INFO);

			// --- Build chain from EE back to base --- //
			
			if (config_["ee_link"]) {
				try{
					ee_link = urdf_model->getLink(config_["ee_link"].as<std::string>());
					if (!ee_link) {
						std::cerr << "End-effector link not found in URDF." << std::endl;
						return robot;
					}
					accumulateChain(ee_link, resolved_base_link_name, chain);
					std::reverse(chain.begin(), chain.end());
				} catch (const std::exception& e) {	
					std::cerr << "Error while building kinematic chain: " << e.what() << std::endl;
					return robot;
				}
			} else {
				debug_log("No end-effector link specified, using all links in the model", VERB_INFO);
				urdf_model->getLinks(chain);
			}
			
			debug_log("Chain: ", VERB_DEBUG);
			for (auto& link : chain) {
				debug_log(" - " + link->name, VERB_DEBUG);
			}

			if (chain.empty()) {
				std::cerr << "Chain is empty." << std::endl;
				return robot;
			}
			if (std::none_of(chain.begin(), chain.end(),
					[&](const auto& link) { return link->name == resolved_base_link_name; })) {
				std::cerr << "Base link '" << resolved_base_link_name << "' not found in chain." << std::endl;
				return robot;
			}

			// --- Identify joints and links --- //
			// std::vector<std::shared_ptr<urdf::Joint>> active_joints;
			// std::vector<casadi::SX> static_transforms; // Transform from parent joint to current joint
			// std::vector<InertialData> active_bodies;
			
			casadi::SX current_cumulative_transform = casadi::SX::eye(4);
			
			debug_log("Chain size: " + std::to_string(chain.size()), VERB_DEBUG);

			// - create thunder structures - //
			reset_thunder_chain();
			// add_chain_from(-1, urdf_model->getLink(base_link_name));
			add_chain_from(-1, root_link);

			// numJoints = jointsName.size();
			
			robot->add_property<int>("numJoints", numJoints, "int", "Number of joints", true);
			robot->add_property<int>("ndof", ndof, "int", "Number of degrees of freedom", true);
			robot->add_property<vector<string>>("jointsName", jointsName, "vector<string>", "Name of joints", true);
			robot->add_property<vector<string>>("jointsType", jointsType, "vector<string>", "Type of joints", true);
			robot->add_property<vector<bool>>("jointsAvailable", jointsAvailable, "vector<bool>", "Joints that are available in the generated library", true);
			robot->add_property<vector<bool>>("jointsDerivatives", jointsDerivatives, "vector<bool>", "Create jacobian derivatives for these joints", true);
			robot->add_property<vector<vector<double>>>("jointsAxis", jointsAxis, "vector<vector<double>>", "Axes of joints", true);
			robot->add_property<vector<int>>("jointsDimension", jointsDimension, "vector<int>", "Degrees of freedom of each joint", true);
			robot->add_property<vector<int>>("jointsParent", jointsParent, "vector<int>", "Parent Id of joints", true);

			// --- Variables --- //
			robot->add_variable("q", casadi::SX::sym("q", ndof, 1), std::vector<double>(ndof, 0), {1}, "Configuration", true);
			robot->add_variable("dq", casadi::SX::sym("dq", ndof, 1), std::vector<double>(ndof, 0), {1}, "Velocity", true);
			robot->add_variable("ddq", casadi::SX::sym("ddq", ndof, 1), std::vector<double>(ndof, 0), {1}, "Acceleration", true);
			robot->add_variable("d3q", casadi::SX::sym("d3q", ndof, 1), std::vector<double>(ndof, 0), {1}, "Jerk", true);
			robot->add_variable("d4q", casadi::SX::sym("d4q", ndof, 1), std::vector<double>(ndof, 0), {1}, "Snap", true);

			// --- Kinematic parameters (par_KIN_num) --- //
			// Whether kinematic parameters should be symbolic (1) or numeric (0).
			// Can be overridden per-link (symbolic_kinematics) or per-element (par_KIN_symb).
		// Default to numeric (false) unless overridden.
		short kin_symb_global = 0;
		if (config_["symbolic_kinematics"]) {
			if (config_["symbolic_kinematics"].IsScalar()) {
				kin_symb_global = yamlScalarToBinary(config_["symbolic_kinematics"], kin_symb_global);
			} else if (config_["symbolic_kinematics"]["default"]) {
				kin_symb_global = yamlScalarToBinary(config_["symbolic_kinematics"]["default"], kin_symb_global);
			}
		}

		auto yaml_kin_map = parseKinematicSymbolicFromYaml(config_["symbolic_kinematics"], kin_symb_global);
			std::string urdf_text = readFileToString(urdf_path_final);
			auto urdf_kin_map = parseSymbolicKinematicsFromUrdfJoints(urdf_text, kin_symb_global);

			// Parse optional base/ee frame offsets up-front so they can be merged into par_KIN.
			const bool has_base_offset = static_cast<bool>(config_["Base_to_L0"]);
			const bool has_ee_offset = static_cast<bool>(config_["Ln_to_EE"]);

			casadi::SX world2L0_expr;
			std::vector<std::string> world2L0_args;
			parse_frame_parameterization(
				robot,
				config_["Base_to_L0"],
				"world2L0",
				std::vector<double>(6, 0.0),
				std::vector<short>(6, 0),
				world2L0_expr,
				world2L0_args,
				"World to base frame");

			casadi::SX ln2ee_expr;
			std::vector<std::string> ln2ee_args;
			parse_frame_parameterization(
				robot,
				config_["Ln_to_EE"],
				"Ln2EE",
				std::vector<double>(6, 0.0),
				std::vector<short>(6, 0),
				ln2ee_expr,
				ln2ee_args,
				"Last link to end-effector frame");

			std::vector<bool> node_has_children(numJoints, false);
			for (int i = 0; i < numJoints; ++i) {
				int p = jointsParent[i];
				if (p >= 0 && p < numJoints) {
					node_has_children[p] = true;
				}
			}

			auto append_unique_args = [](std::vector<std::string>& dst, const std::vector<std::string>& src) {
				for (const auto& a : src) {
					if (std::find(dst.begin(), dst.end(), a) == dst.end()) {
						dst.push_back(a);
					}
				}
			};

			auto compose_frames_rpy = [](const casadi::SX& left_frame, const casadi::SX& right_frame) {
				casadi::SX T = casadi::SX::mtimes({get_transform_rpy(left_frame), get_transform_rpy(right_frame)});
				casadi::SX out = casadi::SX::zeros(6, 1);
				out(casadi::Slice(0, 3)) = T(casadi::Slice(0, 3), 3);
				out(casadi::Slice(3, 6)) = get_euler_rpy(T);
				return out;
			};

			std::vector<short> par_KIN_isSymb(6 * numJoints, kin_symb_global);
			for (int i = 0; i < numJoints; ++i) {
				const auto& link_name = jointsName[i];
				auto it_yaml = yaml_kin_map.find(link_name);
				auto it_urdf = urdf_kin_map.find(link_name);
				if (it_yaml != yaml_kin_map.end()) {
					for (int j = 0; j < 6; ++j) {
						par_KIN_isSymb[6 * i + j] = (j < (int)it_yaml->second.size()) ? it_yaml->second[j] : kin_symb_global;
					}
				} else if (it_urdf != urdf_kin_map.end()) {
					for (int j = 0; j < 6; ++j) {
						par_KIN_isSymb[6 * i + j] = (j < (int)it_urdf->second.size()) ? it_urdf->second[j] : kin_symb_global;
					}
				}
			}

			if (config_["par_KIN_symb"]) {
				par_KIN_isSymb = config_["par_KIN_symb"].as<std::vector<short>>();
			}

			casadi::SX par_KIN_expr = casadi::SX::zeros(6 * numJoints, 1);
			std::vector<std::string> par_KIN_args;
			auto kinematics_cfg = config_["kinematics"];
			for (int i = 0; i < numJoints; ++i) {
				std::vector<double> default_xyzrpy(6, 0.0);
				std::vector<short> default_symb(6, kin_symb_global);
				for (int j = 0; j < 6; ++j) {
					default_xyzrpy[j] = par_KIN_num[6 * i + j];
					default_symb[j] = par_KIN_isSymb[6 * i + j];
				}

				YAML::Node joint_cfg;
				if (kinematics_cfg && kinematics_cfg[jointsName[i]]) {
					joint_cfg = kinematics_cfg[jointsName[i]];
				}

				casadi::SX joint_expr;
				std::vector<std::string> joint_args;
				parse_frame_parameterization(
					robot,
					joint_cfg,
					"KIN_" + jointsName[i],
					default_xyzrpy,
					default_symb,
					joint_expr,
					joint_args,
					"Kinematic frame");

				append_unique_args(par_KIN_args, joint_args);

				// Root nodes are pre-multiplied by Base_to_L0 if configured.
				if (has_base_offset && jointsParent[i] == -1) {
					joint_expr = compose_frames_rpy(world2L0_expr, joint_expr);
					append_unique_args(par_KIN_args, world2L0_args);
				}

				// Terminal nodes (no children in the constructed tree) are post-multiplied by Ln_to_EE if configured.
				if (has_ee_offset && !node_has_children[i]) {
					joint_expr = compose_frames_rpy(joint_expr, ln2ee_expr);
					append_unique_args(par_KIN_args, ln2ee_args);
				}

				par_KIN_expr(casadi::Slice(6 * i, 6 * (i + 1))) = joint_expr;
			}

			if (!robot->add_function("par_KIN", par_KIN_expr, par_KIN_args, "Kinematic parameters")) {
				std::cerr << "Error adding kinematic parameters function!" << std::endl;
				return robot;
			}

			// --- World to L0 (par_world2L0) --- //
			if (!robot->add_function("par_world2L0", world2L0_expr, world2L0_args, "World to base frame.")) {
				std::cerr << "Error adding base frame function!" << std::endl;
				return robot;
			}

			// --- Ln to EE (par_Ln2EE) --- //
			if (!robot->add_function("par_Ln2EE", ln2ee_expr, ln2ee_args, "Last link to end-effector frame.")) {
				std::cerr << "Error adding end-effector frame function!" << std::endl;
				return robot;
			}

			// --- Dynamic parameters (par_DYN_num) --- //
			int STD_PAR_LINK = 10;
			if (robot->properties.count("STD_PAR_LINK")) {
				STD_PAR_LINK = robot->get<int>("STD_PAR_LINK");
			} else {
				robot->add_property<int>("STD_PAR_LINK", STD_PAR_LINK, "int", "Standard number of dynamic parameters per link", true);
			}

			// Whether dynamic parameters should be symbolic (1) or numeric (0).
			// Can be overridden per-link (symbolic_dynamics) or per-element (par_DYN_symb).
			// Default to numeric (false) unless overridden.
			short dyn_symb_global = 0;
			if (config_["symbolic_dynamics"]) {
				if (config_["symbolic_dynamics"].IsScalar()) {
					dyn_symb_global = yamlScalarToBinary(config_["symbolic_dynamics"], dyn_symb_global);
				} else if (config_["symbolic_dynamics"]["default"]) {
					dyn_symb_global = yamlScalarToBinary(config_["symbolic_dynamics"]["default"], dyn_symb_global);
				}
			}
			auto yaml_dyn_map = parseDynamicSymbolicFromYaml(config_["symbolic_dynamics"], dyn_symb_global);
			auto urdf_dyn_map = parseSymbolicFromUrdf(urdf_text, "symbolic_dynamics", STD_PAR_LINK, dyn_symb_global, parseDynamicSymbolicFromXml);

			std::vector<short> par_DYN_isSymb(STD_PAR_LINK * numJoints, dyn_symb_global);
			for (int i = 0; i < numJoints; ++i) {
				const auto& link_name = jointsName[i];
				auto it_yaml = yaml_dyn_map.find(link_name);
				auto it_urdf = urdf_dyn_map.find(link_name);
				if (it_yaml != yaml_dyn_map.end()) {
					for (int j = 0; j < STD_PAR_LINK; ++j) {
						par_DYN_isSymb[STD_PAR_LINK * i + j] = (j < (int)it_yaml->second.size()) ? it_yaml->second[j] : dyn_symb_global;
					}
				} else if (it_urdf != urdf_dyn_map.end()) {
					for (int j = 0; j < STD_PAR_LINK; ++j) {
						par_DYN_isSymb[STD_PAR_LINK * i + j] = (j < (int)it_urdf->second.size()) ? it_urdf->second[j] : dyn_symb_global;
					}
				}
			}

			if (config_["par_DYN_symb"]) {
				par_DYN_isSymb = config_["par_DYN_symb"].as<std::vector<short>>();
			}

			// for (int i = 0; i < numJoints; ++i) {
			// 	const auto& b = active_bodies[i];
			// 	par_DYN_num[STD_PAR_LINK * i + 0] = static_cast<double>(b.mass);
			// 	par_DYN_num[STD_PAR_LINK * i + 1] = static_cast<double>(b.mass_moment(0));
			// 	par_DYN_num[STD_PAR_LINK * i + 2] = static_cast<double>(b.mass_moment(1));
			// 	par_DYN_num[STD_PAR_LINK * i + 3] = static_cast<double>(b.mass_moment(2));
			// 	par_DYN_num[STD_PAR_LINK * i + 4] = static_cast<double>(b.inertia_at_origin(0, 0)); // Ixx
			// 	par_DYN_num[STD_PAR_LINK * i + 5] = static_cast<double>(b.inertia_at_origin(0, 1)); // Ixy
			// 	par_DYN_num[STD_PAR_LINK * i + 6] = static_cast<double>(b.inertia_at_origin(0, 2)); // Ixz
			// 	par_DYN_num[STD_PAR_LINK * i + 7] = static_cast<double>(b.inertia_at_origin(1, 1)); // Iyy
			// 	par_DYN_num[STD_PAR_LINK * i + 8] = static_cast<double>(b.inertia_at_origin(1, 2)); // Iyz
			// 	par_DYN_num[STD_PAR_LINK * i + 9] = static_cast<double>(b.inertia_at_origin(2, 2)); // Izz
			// }

			robot->add_parameter("par_DYN", casadi::SX::sym("par_DYN", STD_PAR_LINK * numJoints, 1), par_DYN_num, par_DYN_isSymb, "Dynamic parameters", true);
			robot->add_parameter("par_REG", casadi::SX::sym("par_REG", STD_PAR_LINK * numJoints, 1), std::vector<double>(STD_PAR_LINK * numJoints, 0), {1}, "Dynamic parameters for regressor", true);

			// --- Gravity --- //
			std::vector<double> gravity_num = {0.0, 0.0, 0.0}; // default no gravity
			if (config_["gravity"]) {
				gravity_num = config_["gravity"]["value"].as<std::vector<double>>();
			}
			std::vector<short> gravity_isSymb(3, 0);
			if (config_["gravity"] && config_["gravity"]["symb"]) {
				gravity_isSymb = config_["gravity"]["symb"].as<std::vector<short>>();
			}
			robot->add_parameter("par_gravity", casadi::SX::sym("gravity", 3), gravity_num, gravity_isSymb, "Gravity on world frame", true);

		} catch (const std::exception& e) {
			std::cerr << "Error in UrdfLoader::load: " << e.what() << std::endl;
			return robot;
		}

		debug_log("URDF Loading finished", VERB_INFO);
		return robot;
	}

} // namespace thunder_ns
