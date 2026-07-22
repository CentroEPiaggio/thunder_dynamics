// thunder_core: nanobind Python module exposing Robot and core types
//
// This module allows Python plugins to interact with the Thunder pipeline
// by reading/writing Robot properties, parameters, and functions using
// CasADi symbolic types seamlessly.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/map.h>

// Type casters (must be included before any binding code that uses these types)
#include "casadi_casters.h"
#include "yaml_casters.h"

#include "robot.h"
#include "utils.h"
#include "plugin_manager.h"

namespace nb = nanobind;
using namespace thunder_ns;

NB_MODULE(_bindings, m) {
    m.doc() = "Thunder Dynamics core bindings — Robot, parameters, functions, and utilities";

    // =======================================================================
    // Property
    // =======================================================================
    nb::class_<Property>(m, "Property")
        .def(nb::init<>())
        .def_rw("name", &Property::name)
        .def_rw("description", &Property::description)
        .def_rw("type_str", &Property::type_str)
        // std::any cannot be directly exposed; provide a dispatcher
        .def("get_value", [](const Property& self) -> nb::object {
            const std::string& t = self.type_str;
            try {
                if (t == "int") return nb::cast(std::any_cast<int>(self.value));
                if (t == "short") return nb::cast(std::any_cast<short>(self.value));
                if (t == "long") return nb::cast(std::any_cast<long>(self.value));
                if (t == "float") return nb::cast(std::any_cast<float>(self.value));
                if (t == "double") return nb::cast(std::any_cast<double>(self.value));
                if (t == "bool") return nb::cast(std::any_cast<bool>(self.value));
                if (t == "string" || t == "std::string") return nb::cast(std::any_cast<std::string>(self.value));
                if (t == "vector<int>") return nb::cast(std::any_cast<std::vector<int>>(self.value));
                if (t == "vector<double>") return nb::cast(std::any_cast<std::vector<double>>(self.value));
                if (t == "vector<string>" || t == "vector<std::string>") return nb::cast(std::any_cast<std::vector<std::string>>(self.value));
                if (t == "vector<short>") return nb::cast(std::any_cast<std::vector<short>>(self.value));
                if (t == "vector<vector<int>>") return nb::cast(std::any_cast<std::vector<std::vector<int>>>(self.value));
                if (t == "vector<vector<double>>") return nb::cast(std::any_cast<std::vector<std::vector<double>>>(self.value));
                if (t == "vector<vector<string>>") return nb::cast(std::any_cast<std::vector<std::vector<std::string>>>(self.value));
            } catch (const std::bad_any_cast&) {
                return nb::none();
            }
            // Fallback: return string representation
            return nb::cast(const_cast<Property&>(self).get_value_str());
        }, "Get property value as a Python object (dispatches on type_str)")
        .def("get_value_str", static_cast<std::string (Property::*)()>(&Property::get_value_str))
    ;

    // =======================================================================
    // Parameter
    // =======================================================================
    nb::class_<Parameter>(m, "Parameter")
        .def(nb::init<>())
        .def_rw("name", &Parameter::name)
        .def_rw("description", &Parameter::description)
        .def_rw("is_symbolic", &Parameter::is_symbolic)
        .def_rw("symb", &Parameter::symb)
        .def_rw("num", &Parameter::num)
        .def("size", &Parameter::size)
        .def("symb_size", &Parameter::symb_size)
        .def("get_value_resized", &Parameter::get_value_resized)
        .def("get_symb_resized", &Parameter::get_symb_resized)
        .def("get_model", &Parameter::get_model)
        .def("get_value_str", &Parameter::get_value_str)
    ;

    // =======================================================================
    // FunArg
    // =======================================================================
    nb::class_<FunArg>(m, "FunArg")
        .def(nb::init<std::string, casadi::SX>(), nb::arg("name"), nb::arg("value"))
        .def_rw("name", &FunArg::name)
        .def_rw("value", &FunArg::value)
        .def("size", &FunArg::size)
    ;

    // =======================================================================
    // Function
    // =======================================================================
    nb::class_<Function>(m, "Function")
        .def(nb::init<>())
        .def_rw("name", &Function::name)
        .def_rw("description", &Function::description)
        .def_rw("args", &Function::args)
        .def_rw("explicit_args", &Function::explicit_args)
        .def_rw("expr", &Function::expr)
        .def_rw("fun", &Function::fun)
        .def("get_out_size", &Function::get_out_size)
        .def("get_args_str", &Function::get_args_str)
        .def("get_ret_type_str", &Function::get_ret_type_str)
    ;

    // =======================================================================
    // Robot
    // =======================================================================
    nb::class_<Robot>(m, "Robot", nb::dynamic_attr())
        .def(nb::init<std::string>(), nb::arg("name") = "robot")
        .def(nb::init<>())
        .def_rw("robotName", &Robot::robotName)
        .def_rw("properties", &Robot::properties)
        .def_rw("parameters", &Robot::parameters)
        .def_rw("functions", &Robot::functions)
        .def_rw("config_yaml", &Robot::config_yaml)

        // --- Typed getters for properties (template get<T>) ---
        .def("get_int", [](Robot& self, const std::string& key) { return self.get<int>(key); },
            nb::arg("key"), "Get property value as int")
        .def("get_double", [](Robot& self, const std::string& key) { return self.get<double>(key); },
            nb::arg("key"), "Get property value as double")
        .def("get_string", [](Robot& self, const std::string& key) { return self.get<std::string>(key); },
            nb::arg("key"), "Get property value as string")
        .def("get_bool", [](Robot& self, const std::string& key) { return self.get<bool>(key); },
            nb::arg("key"), "Get property value as bool")
        .def("get_vector_int", [](Robot& self, const std::string& key) { return self.get<std::vector<int>>(key); },
            nb::arg("key"), "Get property value as vector<int>")
        .def("get_vector_double", [](Robot& self, const std::string& key) { return self.get<std::vector<double>>(key); },
            nb::arg("key"), "Get property value as vector<double>")
        .def("get_vector_string", [](Robot& self, const std::string& key) { return self.get<std::vector<std::string>>(key); },
            nb::arg("key"), "Get property value as vector<string>")
        .def("get_vector_short", [](Robot& self, const std::string& key) { return self.get<std::vector<short>>(key); },
            nb::arg("key"), "Get property value as vector<short>")

        // --- Symbolic model access ---
        .def("get_model", &Robot::get_model,
            nb::arg("key"), nb::arg("explicit_args") = std::vector<casadi::SX>{},
            "Get symbolic expression (casadi.SX) for a parameter or function")

        // --- Numeric evaluation ---
        .def("get_value", static_cast<casadi::DM (Robot::*)(std::string, std::vector<casadi::DM>)>(&Robot::get),
            nb::arg("key"), nb::arg("explicit_args") = std::vector<casadi::DM>{},
            "Evaluate a parameter or function numerically (returns casadi.DM)")

        // --- Set parameter value ---
        .def("set", &Robot::set, nb::arg("name"), nb::arg("value"),
            "Set a parameter's numeric value")

        // --- Typed add_property ---
        .def("add_property_int", [](Robot& self, const std::string& name, int val, const std::string& descr, bool overwrite) {
            return self.add_property<int>(name, val, "int", descr, overwrite);
        }, nb::arg("name"), nb::arg("value"), nb::arg("descr") = "", nb::arg("overwrite") = true)
        .def("add_property_double", [](Robot& self, const std::string& name, double val, const std::string& descr, bool overwrite) {
            return self.add_property<double>(name, val, "double", descr, overwrite);
        }, nb::arg("name"), nb::arg("value"), nb::arg("descr") = "", nb::arg("overwrite") = true)
        .def("add_property_string", [](Robot& self, const std::string& name, const std::string& val, const std::string& descr, bool overwrite) {
            return self.add_property<std::string>(name, val, "string", descr, overwrite);
        }, nb::arg("name"), nb::arg("value"), nb::arg("descr") = "", nb::arg("overwrite") = true)
        .def("add_property_bool", [](Robot& self, const std::string& name, bool val, const std::string& descr, bool overwrite) {
            return self.add_property<bool>(name, val, "bool", descr, overwrite);
        }, nb::arg("name"), nb::arg("value"), nb::arg("descr") = "", nb::arg("overwrite") = true)
        .def("add_property_vector_int", [](Robot& self, const std::string& name, const std::vector<int>& val, const std::string& descr, bool overwrite) {
            return self.add_property<std::vector<int>>(name, val, "vector<int>", descr, overwrite);
        }, nb::arg("name"), nb::arg("value"), nb::arg("descr") = "", nb::arg("overwrite") = true)
        .def("add_property_vector_double", [](Robot& self, const std::string& name, const std::vector<double>& val, const std::string& descr, bool overwrite) {
            return self.add_property<std::vector<double>>(name, val, "vector<double>", descr, overwrite);
        }, nb::arg("name"), nb::arg("value"), nb::arg("descr") = "", nb::arg("overwrite") = true)
        .def("add_property_vector_string", [](Robot& self, const std::string& name, const std::vector<std::string>& val, const std::string& descr, bool overwrite) {
            return self.add_property<std::vector<std::string>>(name, val, "vector<string>", descr, overwrite);
        }, nb::arg("name"), nb::arg("value"), nb::arg("descr") = "", nb::arg("overwrite") = true)

        // --- Variable & Parameter ---
        .def("add_variable", &Robot::add_variable,
            nb::arg("name"), nb::arg("symb"), nb::arg("num"), nb::arg("is_symbolic") = std::vector<short>{1},
            nb::arg("descr") = "", nb::arg("overwrite") = true)
        .def("add_parameter", &Robot::add_parameter,
            nb::arg("name"), nb::arg("symb"), nb::arg("num"), nb::arg("is_symbolic") = std::vector<short>{0},
            nb::arg("descr") = "", nb::arg("overwrite") = true)

        // --- Function ---
        .def("add_function", &Robot::add_function,
            nb::arg("name"), nb::arg("expr"), nb::arg("args"), nb::arg("descr") = "",
            nb::arg("explicit_args") = std::vector<FunArg>{}, nb::arg("overwrite") = true,
            "Add a symbolic function to the robot model")

        // --- I/O ---
        .def("load_par", &Robot::load_par,
            nb::arg("par_file"), nb::arg("par_list") = std::vector<std::string>{})
        .def("save_par", &Robot::save_par,
            nb::arg("par_file"), nb::arg("par_list") = std::vector<std::string>{})
        .def("save_conf", &Robot::save_conf, nb::arg("conf_file"))

        // --- Bulk accessors ---
        .def("get_properties", &Robot::get_properties,
            nb::arg("prop_list") = std::vector<std::string>{})
        .def("get_parameters", &Robot::get_parameters,
            nb::arg("par_list") = std::vector<std::string>{})
        .def("get_functions", &Robot::get_functions,
            nb::arg("fun_list") = std::vector<std::string>{})
    ;

    // =======================================================================
    // PluginManager
    // =======================================================================
    nb::class_<PluginManager>(m, "PluginManager")
        .def(nb::init<>())
        .def("set_verbose", &PluginManager::set_verbose, nb::arg("verbose"))
        .def("print_available_plugins", &PluginManager::print_available_plugins,
            nb::arg("verbose") = false)
        .def("configure_pipeline",
            static_cast<void (PluginManager::*)(const std::string&, int)>(&PluginManager::configure_pipeline),
            nb::arg("config_path"), nb::arg("no_generation") = 0,
            "Configure pipeline from a YAML file path")
        .def("execute", &PluginManager::execute,
            nb::arg("robot_name") = "robot",
            "Execute the pipeline and return the Robot")
    ;

    // =======================================================================
    // Utility functions
    // =======================================================================
    m.def("hat", &thunder_ns::hat, nb::arg("v"), "Skew-symmetric matrix from 3-vector");
    m.def("vect", &thunder_ns::vect, nb::arg("S"), "Extract vector from skew-symmetric matrix");
    m.def("R_x", &thunder_ns::R_x, nb::arg("angle"), "Rotation matrix about X axis");
    m.def("R_y", &thunder_ns::R_y, nb::arg("angle"), "Rotation matrix about Y axis");
    m.def("R_z", &thunder_ns::R_z, nb::arg("angle"), "Rotation matrix about Z axis");
    m.def("R_aa", &thunder_ns::R_aa, nb::arg("axis"), nb::arg("angle"), "Rotation matrix from axis-angle");
    m.def("get_transform_rpy", &thunder_ns::get_transform_rpy, nb::arg("frame_rpy"),
        "Homogeneous transform from [x,y,z,r,p,y] vector");
    m.def("get_euler_rpy", &thunder_ns::get_euler_rpy, nb::arg("T"),
        "Extract RPY Euler angles from homogeneous transform");

    // =======================================================================
    // Internal: _wrap_robot — used by C++ proxy plugins to pass Robot to Python
    // =======================================================================
    m.def("_wrap_robot", [](nb::capsule capsule) -> std::shared_ptr<Robot> {
        auto* sp = static_cast<std::shared_ptr<Robot>*>(
            PyCapsule_GetPointer(capsule.ptr(), "thunder_robot_ptr"));
        if (!sp) {
            throw std::runtime_error("Invalid robot capsule");
        }
        return *sp;
    }, nb::arg("capsule"),
       "Internal: reconstruct a Robot shared_ptr from a PyCapsule (used by C++ proxy plugins)");
}
