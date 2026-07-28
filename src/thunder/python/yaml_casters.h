#ifndef THUNDER_YAML_CASTERS_H
#define THUNDER_YAML_CASTERS_H

// Bidirectional conversion between YAML::Node and Python dict/list/scalar.
// Used to pass YAML config to Python plugins as native Python dicts.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <yaml-cpp/yaml.h>

namespace nb = nanobind;

namespace thunder_py {

// ---------------------------------------------------------------------------
// YAML::Node -> Python object (recursive)
// ---------------------------------------------------------------------------
inline nb::object yaml_to_python(const YAML::Node& node) {
    if (!node.IsDefined() || node.IsNull()) {
        return nb::none();
    }

    if (node.IsScalar()) {
        const std::string& s = node.Scalar();

        // Try bool
        if (s == "true" || s == "True" || s == "TRUE") return nb::bool_(true);
        if (s == "false" || s == "False" || s == "FALSE") return nb::bool_(false);

        // Try integer
        try {
            size_t pos;
            long long iv = std::stoll(s, &pos);
            if (pos == s.size()) return nb::int_(iv);
        } catch (...) {}

        // Try double
        try {
            size_t pos;
            double dv = std::stod(s, &pos);
            if (pos == s.size()) return nb::float_(dv);
        } catch (...) {}

        // Fallback to string
        return nb::cast(s);
    }

    if (node.IsSequence()) {
        nb::list lst;
        for (auto it = node.begin(); it != node.end(); ++it) {
            lst.append(yaml_to_python(*it));
        }
        return lst;
    }

    if (node.IsMap()) {
        nb::dict d;
        for (auto it = node.begin(); it != node.end(); ++it) {
            nb::object key = yaml_to_python(it->first);
            nb::object val = yaml_to_python(it->second);
            d[key] = val;
        }
        return d;
    }

    return nb::none();
}

// ---------------------------------------------------------------------------
// Python object -> YAML::Node (recursive)
// ---------------------------------------------------------------------------
inline YAML::Node python_to_yaml(nb::handle obj) {
    YAML::Node node;

    if (obj.is_none()) {
        return node; // Null node
    }

    // Check bool before int (bool is subclass of int in Python)
    if (nb::isinstance<nb::bool_>(obj)) {
        node = nb::cast<bool>(obj);
        return node;
    }

    if (nb::isinstance<nb::int_>(obj)) {
        node = nb::cast<long long>(obj);
        return node;
    }

    if (nb::isinstance<nb::float_>(obj)) {
        node = nb::cast<double>(obj);
        return node;
    }

    if (nb::isinstance<nb::str>(obj)) {
        node = nb::cast<std::string>(obj);
        return node;
    }

    if (nb::isinstance<nb::list>(obj) || nb::isinstance<nb::tuple>(obj)) {
        for (nb::handle item : obj) {
            node.push_back(python_to_yaml(item));
        }
        return node;
    }

    if (nb::isinstance<nb::dict>(obj)) {
        nb::dict d = nb::borrow<nb::dict>(obj);
        for (auto [key, val] : d) {
            std::string key_str = nb::cast<std::string>(nb::str(key));
            node[key_str] = python_to_yaml(val);
        }
        return node;
    }

    // Fallback: convert to string via Python str()
    node = nb::cast<std::string>(nb::str(obj));
    return node;
}

} // namespace thunder_py

// ===========================================================================
// nanobind type caster for YAML::Node
// ===========================================================================
namespace nanobind { namespace detail {

template <>
struct type_caster<YAML::Node> {
    NB_TYPE_CASTER(YAML::Node, const_name("dict"))

    bool from_python(handle src, uint8_t, cleanup_list*) noexcept {
        try {
            value = thunder_py::python_to_yaml(src);
            return true;
        } catch (...) {
            return false;
        }
    }

    static handle from_cpp(const YAML::Node& src, rv_policy, cleanup_list*) noexcept {
        try {
            nb::object obj = thunder_py::yaml_to_python(src);
            return obj.release();
        } catch (...) {
            return handle();
        }
    }
};

}} // namespace nanobind::detail

#endif // THUNDER_YAML_CASTERS_H
