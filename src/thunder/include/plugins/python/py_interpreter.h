#ifndef THUNDER_PY_INTERPRETER_H
#define THUNDER_PY_INTERPRETER_H

#ifdef THUNDER_PYTHON_PLUGINS

#include <Python.h>
#include <iostream>
#include <string>

namespace thunder_ns {

/// Singleton managing the embedded Python interpreter lifecycle.
/// Lazily initializes on first use and finalizes at program exit.
///
/// After initialization, the GIL is released so that PyGILState_Ensure/Release
/// can be used by proxy plugins to acquire it when needed.
class PythonInterpreter {
public:
    static PythonInterpreter& instance() {
        static PythonInterpreter inst;
        return inst;
    }

    /// Ensure the Python interpreter is initialized.
    /// After first call, the GIL is released (saved as tstate_) so that
    /// subsequent PyGILState_Ensure() calls work correctly.
    /// Safe to call multiple times.
    void ensure_initialized() {
        if (!initialized_by_us_ && !Py_IsInitialized()) {
            Py_Initialize();
            // Enable threading support and release the GIL so that
            // PyGILState_Ensure/Release work properly from any thread.
            tstate_ = PyEval_SaveThread();
            initialized_by_us_ = true;
        }
    }

    /// Add a directory to sys.path.
    /// Must be called after ensure_initialized().
    /// Acquires and releases the GIL internally.
    void add_to_sys_path(const std::string& path) {
        PyGILState_STATE gstate = PyGILState_Ensure();

        PyObject* sys_path = PySys_GetObject("path");
        if (sys_path) {
            PyObject* py_path = PyUnicode_FromString(path.c_str());
            if (!PySequence_Contains(sys_path, py_path)) {
                PyList_Insert(sys_path, 0, py_path);
            }
            Py_DECREF(py_path);
        }

        PyGILState_Release(gstate);
    }

    bool is_initialized() const { return Py_IsInitialized(); }

private:
    PythonInterpreter() = default;
    ~PythonInterpreter() {
        if (initialized_by_us_ && Py_IsInitialized()) {
            // Re-acquire the GIL before finalizing
            if (tstate_) {
                PyEval_RestoreThread(tstate_);
            }
            Py_FinalizeEx();
        }
    }

    PythonInterpreter(const PythonInterpreter&) = delete;
    PythonInterpreter& operator=(const PythonInterpreter&) = delete;

    bool initialized_by_us_ = false;
    PyThreadState* tstate_ = nullptr;
};

} // namespace thunder_ns

#endif // THUNDER_PYTHON_PLUGINS
#endif // THUNDER_PY_INTERPRETER_H
