# Robot Configuration File (`.yaml`)

The robot configuration YAML file is the cornerstone of Thunder Dynamics. It defines the structure, kinematics, and dynamics of your serial manipulator, enabling the `thunder gen` command to generate tailored code.

## File Structure

A typical robot configuration file has the following main sections:

```yaml
# -------------------------- #
## ----- Robot Config ----- ##
# -------------------------- #
version: 0.2

# --- Constants --- #
PI_2: &PI_2           1.5707963267948966
PI_2_neg: &PI_2_neg  -1.5707963267948966

# --- Hyperparameters --- #
num_joints: 3

# joint types: 'R': rotational, 'P': prismatic
type_joints: ['R', 'R', 'R']

# DH convention: a -> alpha -> d -> theta
kinematics:
  symb: [0,0,0,0, 0,0,0,0, 0,0,0,0]
  DH: [0,  0,      1, 0,
       0,  *PI_2,  0, 0,
       1,  0,      0, 0]

# gravity vector
gravity:
  symb: [0,0,0]
  value: [0, 0, -9.81]

# world to link_0
Base_to_L0:
  symb: [0,0,0, 0,0,0]
  tr: [0, 0, 0]
  ypr: [0, 0, 0]

# Link_n to EE
Ln_to_EE:
  symb: [1,1,1, 0,0,0]
  tr: [1, 0, 0]
  ypr: [0, 0, *PI_2_neg]

# -------------------------------- #
## ----- Dynamic Parameters ----- ##
# -------------------------------- #

# inertial parameters expressed w.r.t. DH frames
dynamics:
  link1:
    inertial:
      symb: [1,1,1,1,1,1,1,1,1,1]
      mass: 1.0
      CoM_x: 1.0
      CoM_y: 1.0
      CoM_z: 1.0
      Ixx: 1.0
      Ixy: 1.0
      Ixz: 1.0
      Iyy: 1.0
      Iyz: 1.0
      Izz: 1.0
    friction:
      symb: [0,0]
      Dl: [0.1, 0.01]

  link2:
    inertial:
      symb: [1,1,1,1,1,1,1,1,1,1]
      mass: 1.0
      CoM_x: 1.0
      CoM_y: 1.0
      CoM_z: 1.0
      Ixx: 1.0
      Ixy: 1.0
      Ixz: 1.0
      Iyy: 1.0
      Iyz: 1.0
      Izz: 1.0
    friction:
      symb: [0,0]
      Dl: [0.1, 0.01]

  link3:
    inertial:
      symb: [1,1,1,1,1,1,1,1,1,1]
      mass: 1.0
      CoM_x: 1.0
      CoM_y: 1.0
      CoM_z: 1.0
      Ixx: 1.0
      Ixy: 1.0
      Ixz: 1.0
      Iyy: 1.0
      Iyz: 1.0
      Izz: 1.0
    friction:
      symb: [0,0]
      Dl: [0.1, 0.01]

```

## Key Sections Explained

*   **`version`**: Specifies the configuration file format version (for compatibility keep `0.2`).

*   **Constants**: YAML anchors can be defined for commonly used values like `PI_2` for π/2 and `PI_2_neg` for -π/2.

*   **`num_joints`**: The number of joints (degrees of freedom) of the manipulator. This must match the number of joints defined in `type_joints`, the size of the `kinematics.DH` array, and the number of links in the `dynamics` section.

*   **`type_joints`**: Array specifying the joint types for each joint:
    *   `'R'`: Rotational (revolute) joint
    *   `'P'`: Prismatic joint
    *   `'R_SEA'`: Series Elastic Actuator rotational joint
    *   `'P_SEA'`: Series Elastic Actuator prismatic joint

*   **`kinematics`**: Defines the Denavit-Hartenberg parameters for the robot.
    *   `symb`: Array of flags (0 or 1) for each DH parameter. If `1`, the parameter is symbolic; if `0`, it's numeric.
    *   `DH`: Flat array containing all DH parameters in the order `[a1, alpha1, d1, theta1, a2, alpha2, d2, theta2, ...]`
    *   **Convention**: The parameters define the transformation from frame `{i-1}` to frame `{i}`. The order `a, alpha, d, theta` corresponds to:
        1.  Rotate about `z_{i-1}` by `theta_i` (joint variable if the joint is revolute).
        2.  Translate along `z_{i-1}` by `d_i` (joint variable if the joint is prismatic).
        3.  Translate along `x_i` (new) by `a_i`.
        4.  Rotate about `x_i` by `alpha_i`.
    *  In the YAML, joint variables are often set to `0.0` as placeholders.

*   **`gravity`**: Defines the gravity vector in the base frame `{0}`.
    *   `symb`: Array of 3 flags for the gravity vector components.
    *   `value`: The gravity vector `[gx, gy, gz]` in m/s². Default is typically `[0, 0, -9.81]`.

*   **`Base_to_L0`**: Transformation from world frame to the first link frame.
    *   `symb`: Array of 6 flags for the transformation parameters.
    *   `tr`: Translation vector `[x, y, z]` in meters.
    *   `ypr`: Rotation as Yaw-Pitch-Roll angles `[yaw, pitch, roll]` in radians.

*   **`Ln_to_EE`**: Transformation from the last link frame to the end-effector frame.
    *   `symb`: Array of 6 flags for the transformation parameters.
    *   `tr`: Translation vector `[x, y, z]` in meters.
    *   `ypr`: Rotation as Yaw-Pitch-Roll angles `[yaw, pitch, roll]` in radians.

*   **`dynamics`**: Contains the dynamic properties for each link.
    *   **`linkX.inertial`**: Inertial parameters for link X, expressed in the respective DH frame.
        *   `symb`: Array of 10 flags (0 or 1) for each inertial parameter.
        *   `mass`: Link mass in kg.
        *   `CoM_x`, `CoM_y`, `CoM_z`: Center of mass coordinates in meters.
        *   `Ixx`, `Iyy`, `Izz`, `Ixy`, `Ixz`, `Iyz`: Inertia tensor components in kg⋅m².
    *   **`linkX.friction`**: Friction parameters for joint X.
        *   `symb`: Array of flags for friction parameters.
        *   `Dl`: Array containing friction coefficients (typically `[Coulomb, Viscous]`).

## Optional Sections

*   **`ELASTIC_MODEL`**: (Optional) Boolean flag to enable elastic modeling for SEA robots. Set to `true` for robots with elastic joints.

*   **`Dl_order`**: (Optional) Specifies the order of link friction model (0 to disable link friction).

*   **`elastic`**: (Optional) Parameters for Series Elastic Actuators, required when `ELASTIC_MODEL: true`.
    *   `K_order`: Order of stiffness model.
    *   `D_order`: Order of damping model.
    *   `Dm_order`: Order of motor damping model.
    *   **`joints.jointX`**: Parameters for each elastic joint.
        *   `K`: Stiffness coefficients array.
        *   `D`: Damping coefficients array.
        *   `Dm`: Motor damping coefficients array.
        *   `K_symb`, `D_symb`, `Dm_symb`: Symbolic flags for each parameter set.

## Symbolic Parameters (`symb:`)

The `symb:` flag is crucial for applications like adaptive control or online parameter tuning.

*   Setting `symb: [1, 1, ..., 1]` for a parameter group (e.g., `dynamics.link1.inertial.symb`) makes all those parameters symbolic.
*   The individual parameter fields (e.g., `mass`, `CoM_x`, `Ixx`, etc.) provide the *initial* numerical values for these symbolic parameters when the code is generated.
*   Parameters marked with `symb: 0` will have their values baked into the generated code and cannot be changed at runtime.

**Examples of symbolic parameter usage**:
- `kinematics.symb: [1,1,1,1, 0,0,0,0, 0,0,0,0]` - Makes the first joint's DH parameters symbolic
- `dynamics.link1.inertial.symb: [1,1,1,1,1,1,1,1,1,1]` - Makes all inertial parameters of link1 symbolic
- `gravity.symb: [0,0,0]` - Makes gravity parameters non-symbolic (numeric)

## Examples

Refer to the YAML files within the `robots/` directory for concrete examples:
- `robots/RRR/RRR.yaml` - Basic 3-DOF RRR robot configuration
- `robots/franka/franka.yaml` - 7-DOF Franka Emika Panda robot configuration  
- `robots/RRR_sea/seaRRR.yaml` - 3-DOF robot with Series Elastic Actuators
