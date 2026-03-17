#ifndef THUNDER_CASADI_CASTERS_H
#define THUNDER_CASADI_CASTERS_H

// CasADi SWIG <-> nanobind type bridge
// Uses the swigbind11 pointer-extraction technique adapted for:
//   1. nanobind (instead of pybind11)
//   2. value types (instead of shared_ptr)
//
// CasADi Python objects are SWIG-wrapped. This caster extracts the C++ pointer
// from the SwigPyObject and copy-constructs the value type, and vice versa.

#include <nanobind/nanobind.h>
#include <casadi/casadi.hpp>

// SWIG runtime header — provides SwigPyObject, SWIG_TypeQuery, SWIG_NewPointerObj
#include "swigpyrun.h"

namespace nb = nanobind;

namespace thunder_py {

// ---------------------------------------------------------------------------
// Helper: extract C++ pointer from a SWIG-wrapped Python object
// ---------------------------------------------------------------------------
template <typename T>
T* extract_swig_ptr(nb::handle obj) {
    // SWIG proxy objects have a .this attribute holding the SwigPyObject
    PyObject* this_attr = PyObject_GetAttrString(obj.ptr(), "this");
    if (!this_attr) {
        PyErr_Clear();
        throw nb::type_error("Expected a SWIG-wrapped object (no .this attribute)");
    }

    SwigPyObject* swig_obj = reinterpret_cast<SwigPyObject*>(this_attr);
    T* ptr = reinterpret_cast<T*>(swig_obj->ptr);
    Py_DECREF(this_attr);

    if (!ptr) {
        throw nb::type_error("SWIG object contains a null pointer");
    }
    return ptr;
}

// ---------------------------------------------------------------------------
// Helper: wrap a C++ object as a SWIG Python object
// ---------------------------------------------------------------------------
template <typename T>
nb::object create_swig_object(const T& value, const char* swig_type_name, const char* swig_module_name) {
    // Ensure the SWIG module is imported so type info is registered
    PyObject* mod = PyImport_ImportModule(swig_module_name);
    if (!mod) {
        throw nb::type_error("Failed to import SWIG module for type creation");
    }
    Py_DECREF(mod);

    swig_type_info* info = SWIG_TypeQuery(swig_type_name);
    if (!info) {
        std::string msg = std::string("SWIG type info not found for: ") + swig_type_name;
        throw nb::type_error(msg.c_str());
    }

    // Heap-allocate a copy; SWIG_POINTER_OWN transfers ownership to Python
    T* heap_copy = new T(value);
    PyObject* py_obj = SWIG_NewPointerObj(heap_copy, info, SWIG_POINTER_OWN);
    if (!py_obj) {
        delete heap_copy;
        throw nb::type_error("Failed to create SWIG Python object");
    }

    return nb::steal(py_obj);
}

} // namespace thunder_py


// ===========================================================================
// nanobind type casters for CasADi types
// ===========================================================================

namespace nanobind { namespace detail {

// ---------------------------------------------------------------------------
// casadi::SX
// ---------------------------------------------------------------------------
template <>
struct type_caster<casadi::SX> {
    NB_TYPE_CASTER(casadi::SX, const_name("casadi.SX"))

    bool from_python(handle src, uint8_t, cleanup_list*) noexcept {
        try {
            casadi::SX* ptr = thunder_py::extract_swig_ptr<casadi::SX>(src);
            value = *ptr;
            return true;
        } catch (...) {
            return false;
        }
    }

    static handle from_cpp(const casadi::SX& src, rv_policy, cleanup_list*) noexcept {
        try {
            nb::object obj = thunder_py::create_swig_object(src, "casadi::SX *", "casadi.casadi");
            return obj.release();
        } catch (...) {
            return handle();
        }
    }
};

// ---------------------------------------------------------------------------
// casadi::DM
// ---------------------------------------------------------------------------
template <>
struct type_caster<casadi::DM> {
    NB_TYPE_CASTER(casadi::DM, const_name("casadi.DM"))

    bool from_python(handle src, uint8_t, cleanup_list*) noexcept {
        try {
            casadi::DM* ptr = thunder_py::extract_swig_ptr<casadi::DM>(src);
            value = *ptr;
            return true;
        } catch (...) {
            return false;
        }
    }

    static handle from_cpp(const casadi::DM& src, rv_policy, cleanup_list*) noexcept {
        try {
            nb::object obj = thunder_py::create_swig_object(src, "casadi::DM *", "casadi.casadi");
            return obj.release();
        } catch (...) {
            return handle();
        }
    }
};

// ---------------------------------------------------------------------------
// casadi::Function
// ---------------------------------------------------------------------------
template <>
struct type_caster<casadi::Function> {
    NB_TYPE_CASTER(casadi::Function, const_name("casadi.Function"))

    bool from_python(handle src, uint8_t, cleanup_list*) noexcept {
        try {
            casadi::Function* ptr = thunder_py::extract_swig_ptr<casadi::Function>(src);
            value = *ptr;
            return true;
        } catch (...) {
            return false;
        }
    }

    static handle from_cpp(const casadi::Function& src, rv_policy, cleanup_list*) noexcept {
        try {
            nb::object obj = thunder_py::create_swig_object(src, "casadi::Function *", "casadi.casadi");
            return obj.release();
        } catch (...) {
            return handle();
        }
    }
};

}} // namespace nanobind::detail

#endif // THUNDER_CASADI_CASTERS_H
