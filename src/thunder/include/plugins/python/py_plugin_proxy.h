#ifndef THUNDER_PY_PLUGIN_PROXY_H
#define THUNDER_PY_PLUGIN_PROXY_H

#ifdef THUNDER_PYTHON_PLUGINS

#include <Python.h>
#include <string>
#include <memory>
#include <tuple>

#include "../../plugin_interfaces.h"
#include "py_interpreter.h"

namespace thunder_ns {

/// Parse a "py:module.ClassName" string into (module_name, class_name).
/// Handles dotted module paths: "py:my_pkg.sub.MyClass" -> ("my_pkg.sub", "MyClass")
std::tuple<std::string, std::string> parse_py_plugin_name(const std::string& name);

// =========================================================================
// PyLoaderProxy
// =========================================================================
class PyLoaderProxy : public BaseLoader {
public:
    PyLoaderProxy(const std::string& module_name, const std::string& class_name);

    std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot) override;

private:
    std::string module_name_;
    std::string class_name_;
    PyObject* py_instance_ = nullptr;

    void ensure_instance();
};

// =========================================================================
// PyBuilderProxy
// =========================================================================
class PyBuilderProxy : public BaseBuilder {
public:
    PyBuilderProxy(const std::string& module_name, const std::string& class_name);

    void build(std::shared_ptr<Robot> robot) override;

private:
    std::string module_name_;
    std::string class_name_;
    PyObject* py_instance_ = nullptr;

    void ensure_instance();
};

// =========================================================================
// PyGeneratorProxy
// =========================================================================
class PyGeneratorProxy : public BaseGenerator {
public:
    PyGeneratorProxy(const std::string& module_name, const std::string& class_name);

    void generate(const std::shared_ptr<Robot> robot) override;

private:
    std::string module_name_;
    std::string class_name_;
    PyObject* py_instance_ = nullptr;

    void ensure_instance();
};

} // namespace thunder_ns

#endif // THUNDER_PYTHON_PLUGINS
#endif // THUNDER_PY_PLUGIN_PROXY_H
