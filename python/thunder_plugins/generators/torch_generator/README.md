# Thunder PyTorch Code Generator Plugin


This plugin generates **batched PyTorch implementations** of CasADi symbolic functions from the Thunder dynamics model using CUSADI. It produces:

1. **Generated function code** (`gen_torch_functions.py`): Raw PyTorch operations for each CasADi function
2. **Wrapper class** (`torch_robot_wrapper.py`): High-level class inheriting from `ThunderRobotTorch` for easy interaction

## Features
 -  **Batched computation**: All generated functions support batch dimension for vectorized execution  
 - **GPU-ready**: Generated code works on CPU or GPU via PyTorch's device abstraction  
 - **Automatic wrapper generation**: Produces robot-specific wrapper class with typed methods  

## Installation

This plugin will automatically be installed with Thunder Dynamics by doing 
```bash
sudo make install
```
on the Thunder Dynamics source code.

You can use it directly in YAML:

```yaml
pipeline:
  loaders: ["kin_loader", "dyn_loader"]
  builders: ["kin_builder", "dyn_builder"]
  generators: ["robot_generator", "PY.thunder_plugins.generators.TorchGenerator"]

"PY.thunder_plugins.generators.TorchGenerator":
  output_dir: "./RRR_torch"
  output_dir: "./myrobot_thunder_torch"
  generate_wrapper: true
  verbose: true
```


After generation, you can either install using
```bash
pip install -e ./myrobot_thunder_torch
```
or simply import the generated wrapper directly from the output directory

```python
import myrobot_thunder_torch.ThunderRobotMyRobot_PyTorch as MyRobotTorch

robot = MyRobotTorch(batch_size=32, device='cuda')

robot.set_q(torch.randn(7))
robot.set_dq(torch.randn(7))

# Compute dynamics
M = robot.get_M()         # Mass matrix (B, 7, 7)
C = robot.get_C()         # Coriolis matrix (B, 7, 7)
G = robot.get_G()         # Gravity vector (B, 7, 1)
Yr = robot.get_Yr()       # Regressor (B, 7, 70)

# All operations are batched and GPU-accelerated
tau = torch.matmul(M, robot.ddq.unsqueeze(-1)).squeeze(-1) + \
      torch.matmul(C, robot.dq.unsqueeze(-1)).squeeze(-1) + G[:, :, 0]

```


### From Python

```python
from thunder_core.pipeline import Config

# Configure and run pipeline
cfg = Config("robot.yaml")
robot = cfg.execute(robot_name="my_robot", no_generation=False)

# Generated files will be in ./torch_generated/
```



## Configuration Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `output_dir` | str | `"./torch_generated"` | Directory to write generated code |
| `generate_wrapper` | bool | `true` | Generate wrapper class inheriting from ThunderRobotTorch |
| `verbose` | bool | `false` | Print detailed generation progress |


## Generated Files

### `gen_torch_functions.py`

Contains raw PyTorch implementations of each CasADi function. Do not edit or open, it may be very large. 

### `torch_robot_wrapper.py`

High-level interface.


