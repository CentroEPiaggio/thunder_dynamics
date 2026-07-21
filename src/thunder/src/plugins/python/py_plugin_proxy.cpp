#ifdef THUNDER_PYTHON_PLUGINS

#include "plugins/python/py_plugin_proxy.h"

#include <Python.h>
#include <filesystem>
#include <sstream>
#include <stdexcept>

namespace thunder_ns {

// =========================================================================
// Helpers
// =========================================================================

std::tuple<std::string, std::string> parse_py_plugin_name(const std::string& name) {
    // Input: "PY.module.path.ClassName"
    // Strip "PY." prefix
    std::string qualified = name.substr(3);

    // Split at last '.' to separate module from class
    auto last_dot = qualified.rfind('.');
    if (last_dot == std::string::npos) {
        throw std::runtime_error(
            "Invalid Python plugin name '" + name + "': expected 'PY.module.ClassName'");
    }

    return {qualified.substr(0, last_dot), qualified.substr(last_dot + 1)};
}

/// Fetch and format the current Python exception as a string.
/// Clears the Python error indicator.
static std::string fetch_python_error() {
    if (!PyErr_Occurred()) return "(no Python error set)";

    PyObject *ptype, *pvalue, *ptraceback;
    PyErr_Fetch(&ptype, &pvalue, &ptraceback);
    PyErr_NormalizeException(&ptype, &pvalue, &ptraceback);

    std::string msg = "Python error";
    if (pvalue) {
        PyObject* str_obj = PyObject_Str(pvalue);
        if (str_obj) {
            const char* s = PyUnicode_AsUTF8(str_obj);
            if (s) msg = s;
            Py_DECREF(str_obj);
        }
    }

    // Try to get traceback
    if (ptraceback) {
        PyObject* tb_module = PyImport_ImportModule("traceback");
        if (tb_module) {
            PyObject* format_func = PyObject_GetAttrString(tb_module, "format_exception");
            if (format_func) {
                PyObject* result = PyObject_CallFunctionObjArgs(
                    format_func, ptype, pvalue, ptraceback, NULL);
                if (result) {
                    PyObject* joined = PyUnicode_Join(PyUnicode_FromString(""), result);
                    if (joined) {
                        const char* s = PyUnicode_AsUTF8(joined);
                        if (s) msg = s;
                        Py_DECREF(joined);
                    }
                    Py_DECREF(result);
                }
                Py_DECREF(format_func);
            }
            Py_DECREF(tb_module);
        }
    }

    Py_XDECREF(ptype);
    Py_XDECREF(pvalue);
    Py_XDECREF(ptraceback);

    return msg;
}

/// Import a Python module and instantiate a class from it.
/// Returns a new reference to the instance (caller must DECREF).
static PyObject* import_and_instantiate(const std::string& module_name, const std::string& class_name) {
    PythonInterpreter::instance().ensure_initialized();

    PyObject* py_module = PyImport_ImportModule(module_name.c_str());
    if (!py_module) {
        std::string err = fetch_python_error();
        throw std::runtime_error("Failed to import Python module '" + module_name + "': " + err);
    }

    PyObject* py_class = PyObject_GetAttrString(py_module, class_name.c_str());
    Py_DECREF(py_module);
    if (!py_class) {
        std::string err = fetch_python_error();
        throw std::runtime_error(
            "Class '" + class_name + "' not found in module '" + module_name + "': " + err);
    }

    // Instantiate: py_instance = py_class()
    PyObject* py_instance = PyObject_CallObject(py_class, NULL);
    Py_DECREF(py_class);
    if (!py_instance) {
        std::string err = fetch_python_error();
        throw std::runtime_error(
            "Failed to instantiate '" + module_name + "." + class_name + "': " + err);
    }

    return py_instance;
}

/// Call py_instance.configure(config_dict) where config is a YAML::Node
/// converted to a Python dict via the yaml_to_python helper.
static void call_configure(PyObject* py_instance, const YAML::Node& config) {
    // Import thunder_py yaml helper (compiled into thunder_core module)
    // Instead, we do the conversion manually using the Python yaml module or
    // by converting YAML -> string -> Python dict via yaml.safe_load
    // This avoids depending on the nanobind module at this point.

    // Convert YAML::Node to a YAML string, then parse in Python
    YAML::Emitter emitter;
    emitter << config;
    std::string yaml_str = emitter.c_str();

    // Use Python's yaml.safe_load to parse
    PyObject* yaml_module = PyImport_ImportModule("yaml");
    if (!yaml_module) {
        // Fallback: pass an empty dict if PyYAML is not available
        PyObject* empty_dict = PyDict_New();
        PyObject* result = PyObject_CallMethod(py_instance, "configure", "(O)", empty_dict);
        Py_XDECREF(result);
        Py_DECREF(empty_dict);
        return;
    }

    PyObject* py_yaml_str = PyUnicode_FromString(yaml_str.c_str());
    PyObject* config_dict = PyObject_CallMethod(yaml_module, "safe_load", "(O)", py_yaml_str);
    Py_DECREF(py_yaml_str);
    Py_DECREF(yaml_module);

    if (!config_dict || config_dict == Py_None) {
        Py_XDECREF(config_dict);
        config_dict = PyDict_New();
    }

    PyObject* result = PyObject_CallMethod(py_instance, "configure", "(O)", config_dict);
    Py_DECREF(config_dict);
    if (!result) {
        std::string err = fetch_python_error();
        throw std::runtime_error("Python plugin configure() failed: " + err);
    }
    Py_DECREF(result);
}

/// Call a method on the Python plugin instance, passing the Robot as a
/// thunder_core.Robot object via nanobind capsule.
/// This requires the thunder_core module to be importable.
static void call_with_robot(PyObject* py_instance, const char* method_name,
                            std::shared_ptr<Robot> robot) {
    // Import the thunder_core._bindings module to get the Robot type
    PyObject* tc_module = PyImport_ImportModule("thunder_core._bindings");
    if (!tc_module) {
        std::string err = fetch_python_error();
        throw std::runtime_error(
            "Failed to import thunder_core._bindings module (required for Python plugins): " + err);
    }

    // Use nanobind's internal casting to create a Python Robot wrapper.
    // We do this by calling thunder_core._wrap_robot_ptr(capsule) — but nanobind
    // doesn't expose that directly. Instead, we use the nanobind C API.
    //
    // Alternative approach: use a global function registered in thunder_core
    // that accepts a raw pointer via capsule and returns a wrapped Robot.

    // We registered a _wrap_robot helper in thunder_core._bindings for this purpose.
    PyObject* wrap_fn = PyObject_GetAttrString(tc_module, "_wrap_robot");
    Py_DECREF(tc_module);

    if (!wrap_fn) {
        std::string err = fetch_python_error();
        throw std::runtime_error("thunder_core._bindings._wrap_robot not found: " + err);
    }

    // Pass the shared_ptr as a PyCapsule
    // The capsule destructor will release the shared_ptr copy
    auto* sp_copy = new std::shared_ptr<Robot>(robot);
    PyObject* capsule = PyCapsule_New(sp_copy, "thunder_robot_ptr", [](PyObject* cap) {
        auto* sp = static_cast<std::shared_ptr<Robot>*>(PyCapsule_GetPointer(cap, "thunder_robot_ptr"));
        delete sp;
    });

    PyObject* py_robot = PyObject_CallFunctionObjArgs(wrap_fn, capsule, NULL);
    Py_DECREF(wrap_fn);
    Py_DECREF(capsule);

    if (!py_robot) {
        std::string err = fetch_python_error();
        throw std::runtime_error("Failed to wrap Robot for Python: " + err);
    }

    // Call the method: py_instance.method(py_robot)
    PyObject* result = PyObject_CallMethod(py_instance, method_name, "(O)", py_robot);
    Py_DECREF(py_robot);

    if (!result) {
        std::string err = fetch_python_error();
        throw std::runtime_error(
            std::string("Python plugin ") + method_name + "() failed: " + err);
    }
    Py_DECREF(result);
}

// =========================================================================
// PyLoaderProxy
// =========================================================================

PyLoaderProxy::PyLoaderProxy(const std::string& module_name, const std::string& class_name)
    : BaseLoader("PY." + module_name + "." + class_name,
                 "Python loader: " + module_name + "." + class_name),
      module_name_(module_name), class_name_(class_name) {}

void PyLoaderProxy::ensure_instance() {
    if (!py_instance_) {
        // Add the config file directory to sys.path
        if (config_["config_path"]) {
            std::string config_path = config_["config_path"].as<std::string>();
            std::filesystem::path dir = std::filesystem::path(config_path).parent_path();
            PythonInterpreter::instance().add_to_sys_path(dir.string());
        }
        py_instance_ = import_and_instantiate(module_name_, class_name_);
        call_configure(py_instance_, config_);
    }
}

std::shared_ptr<Robot> PyLoaderProxy::load(std::shared_ptr<Robot> robot) {
    PythonInterpreter::instance().ensure_initialized();
    PyGILState_STATE gstate = PyGILState_Ensure();
    try {
        ensure_instance();
        call_with_robot(py_instance_, "load", robot);
        PyGILState_Release(gstate);
        return robot;
    } catch (...) {
        PyGILState_Release(gstate);
        throw;
    }
}

// =========================================================================
// PyBuilderProxy
// =========================================================================

PyBuilderProxy::PyBuilderProxy(const std::string& module_name, const std::string& class_name)
    : BaseBuilder("PY." + module_name + "." + class_name,
                  "Python builder: " + module_name + "." + class_name),
      module_name_(module_name), class_name_(class_name) {}

void PyBuilderProxy::ensure_instance() {
    if (!py_instance_) {
        if (config_["config_path"]) {
            std::string config_path = config_["config_path"].as<std::string>();
            std::filesystem::path dir = std::filesystem::path(config_path).parent_path();
            PythonInterpreter::instance().add_to_sys_path(dir.string());
        }
        py_instance_ = import_and_instantiate(module_name_, class_name_);
        call_configure(py_instance_, config_);
    }
}

void PyBuilderProxy::build(std::shared_ptr<Robot> robot) {
    PythonInterpreter::instance().ensure_initialized();
    PyGILState_STATE gstate = PyGILState_Ensure();
    try {
        ensure_instance();
        call_with_robot(py_instance_, "build", robot);
        PyGILState_Release(gstate);
    } catch (...) {
        PyGILState_Release(gstate);
        throw;
    }
}

// =========================================================================
// PyGeneratorProxy
// =========================================================================

PyGeneratorProxy::PyGeneratorProxy(const std::string& module_name, const std::string& class_name)
    : BaseGenerator("PY." + module_name + "." + class_name,
                    "Python generator: " + module_name + "." + class_name),
      module_name_(module_name), class_name_(class_name) {}

void PyGeneratorProxy::ensure_instance() {
    if (!py_instance_) {
        if (config_["config_path"]) {
            std::string config_path = config_["config_path"].as<std::string>();
            std::filesystem::path dir = std::filesystem::path(config_path).parent_path();
            PythonInterpreter::instance().add_to_sys_path(dir.string());
        }
        py_instance_ = import_and_instantiate(module_name_, class_name_);
        call_configure(py_instance_, config_);
    }
}

void PyGeneratorProxy::generate(const std::shared_ptr<Robot> robot) {
    PythonInterpreter::instance().ensure_initialized();
    PyGILState_STATE gstate = PyGILState_Ensure();
    try {
        ensure_instance();
        call_with_robot(py_instance_, "generate", std::const_pointer_cast<Robot>(robot));
        PyGILState_Release(gstate);
    } catch (...) {
        PyGILState_Release(gstate);
        throw;
    }
}

} // namespace thunder_ns

#endif // THUNDER_PYTHON_PLUGINS
