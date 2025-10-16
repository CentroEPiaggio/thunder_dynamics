

## Implementation roadmap for PyTorch integration
This branch is heavily inspired by [Cusadi](https://github.com/se-hwan/cusadi).

The idea of this branch is to have a PyTorch backend for the `thunder` library, which allows us to use the same functions as in the C++ backend, but with batch-invariance, autograd and potentially GPU acceleration. 

For now, for simplicity, the process is not embedded yet in `thunder` code generation, and it requires a few manual steps to have a functional library.

## Current status
 - We have a script that scans for all the `.casadi` functions, runs the PyTorch code generation and outputs a single file with all the generated functions.
 - We are able to run these functions in batch mode and in GPU in a standalone test script.

## How to use what we have so far
### Add Cusadi dictionary
You will need [THIS FILE](https://github.com/se-hwan/cusadi/blob/dev/src/CusadiOperations.py) from Cusadi for the conversion. Rename it as `cusadiops.py`, place it in `src/thunder_torch/`, and uncomment the last `OP_PYTORCH_DICT`. You can comment out everything else, that's the only part needed.

### Post Installation steps
1. You need to re-compile casadi with python bindings, go into the `casadi` directory (if you are using the docker, this is `~/casadi`) and run:
  

   ```bash
    cd build
    cmake -DWITH_PYTHON=ON -DWITH_PYTHON3=ON ..
    make 
    sudo make install
   ```

2. Casadi installation directory by default is `/usr/local/python/casadi`. Make sure that this path is in your PYTHONPATH. You can do this by adding the following line to your `~/.bashrc` file:

   ```bash
   export PYTHONPATH=$PYTHONPATH:/usr/local/python
   ```
   Then, run `source ~/.bashrc` to apply the changes.
3. Install pytorch
    ```bash
    pip install torch
    ```

#### How to run code generation

1. Generate the library with the casadi functions:
   ```bash
   thunder gen -f
   ```
   This will generate the `.casadi` files in the `generatedFiles/casadi_functions` directory.

2. The generation script and the conversion dictionary are in the `src/thunder_torch` folder. Run the code generation script using the python interpreter that has the casadi bindings installed:
   
   ```bash
   python src/thunder_torch/thunder_torch_gen.py --robot_dir path/to/your/robot_generatedFiles --output_dir path/to/your/robot_generatedTorchLib
   ```
   
   This will generate a single **huge** file `gen_file.py` in the `path/to/your/robot_generatedTorchLib` directory. This file contains all the functions generated from the `.casadi` files. Similarly to the C version, you should not edit or open this file, as it reaches easily several hundred thousands of lines of code (~ 1M for a 7DOF robot).

---

### Implementation diary


- [x] Implement a script that scans for all the .casadi functions, run torch codegen and outputs a single file with all the functions.
  - [x]  We are using the dictionary `OP_PYTORCH_DICT` to map the CasADi operations to PyTorch operations. This correctly handles batch invariance.
  - [x]  We are currently able to run _dense_ operations in `test.py`.
- [ ] Implement a high-level wrapper class that implements state variables, the batch size, and parameter handling.
  - [x] Current implementation `thunder_robot_torch.py`  is in the template folder and handles state variables.
  - [ ] Handle parameter loading and unloading
  - [ ] Generate the high level wrapper of each .casadi function.
- [ ] Currently we need to generate the .casadi functions using `thunder gen -f` and then run the codegen script. This could be automated in a single `thunder gen --pytorch` command.
- [ ] Current implementation requires to build casadi python bindings by source add them to PYTHONPATH. This should be streamlined in the Dockerfile.

### Changes to the main thunder generation
- So far we have access only to the `.casadi` files during pytorch conversion. This means that we cannot use the Func class but we need to infer the function metadata from the file. Fortunately, the `.casadi` files contain input and output names, but in the current version they were not saved in the file. This has been fixed in this branch. We still don't know how to access the function description.

### What we can expect in the future.

- Isaac Sim Integration
- GPU-accelerated computation for main variables.
- `torch.NN module` integration: any function with NN module could gives us automatic differentiation and parameter handling. Any function, and possibly any combination of functions, could be exported as a torch NN layer, where the symbolic parameters (eg dynamic parameters) are treated as weight and can be optimised in a training loop.
