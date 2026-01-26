#  thunder_<ROBOT> Generated Library

This is a robot library generated for <ROBOT> using thunder.
It contains the generated C++ code and Python bindings for the <ROBOT> robot model.

## C++ Compilation

You can compile this code as a standalone C++ library to include in your own projects.

### Dependencies

*   **Eigen3**

### Building and install the library
```bash
mkdir build && cd build
cmake ..
make -j && sudo make install
```

By default, cmake will install in `/usr/local`, you can customize the install directory using `-DCMAKE_INSTALL_PREFIX`

## Python Library

This project is configured as a Python package using `scikit-build-core` and `pybind11`.

### Dependencies
- Python 3.7+
- CMake >= 3.15
- C++ Compiler (GCC/Clang/MSVC)

### Recommended Installation (using `uv`)

If you use `uv`, installation is straightforward:

```bash
uv pip install -e .
```


### Simple Installation (using `pip`)

If you are using standard `pip`, you must ensure a build backend (like Ninja) is available first:

```bash
pip install ninja
pip install .
```


### Usage

Once installed, you can import using

```python
from thunder_<ROBOT>_py import thunder_<ROBOT>

my_robot = thunder_<ROBOT>()

```