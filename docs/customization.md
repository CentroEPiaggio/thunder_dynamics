# Customizing Thunder: Adding New Functions

Thunder Dynamics allows you to extend its capabilities by defining your own custom symbolic functions using CasADi and integrating them into the code generation process. These custom functions become available as methods in the generated `thunder_<robot_name>` class, just like the standard kinematics and dynamics functions.

## Use Cases

*   Implementing specific control laws.
*   Calculating task-space dynamics.
*   Computing energy consumption metrics.
*   Any other computation based on the robot's state (q, dq) and parameters.

## Steps to Add a Custom Function

Let's say you want to add a function `getMyCustomValue` that calculates `sin(q_1) * custom_gain`, where `custom_gain` is a parameter defined in the YAML.

1.  **Define the Parameter (Optional):**
    If your function needs parameters not already defined (like DH or inertial), add them to your robot's YAML configuration file under a suitable section (e.g., `custom_params`). Make it symbolic if you want to change it at runtime.

    ```yaml
    # In your <robot>.yaml
    custom_params:
      my_gain:
        symb: [1] # Symbolic
        value: [10.0] # Initial value
    ```

2.  **Declare the Function Signature:**
    Open `src/thunder/library/userDefined.h`. Add the declaration of your function. It must take a `Robot&` reference as its only argument and return a flag indicating success or failure (typically `int`).

    ```c++
    // In src/thunder/library/userDefined.h
    #ifndef USERDEFINED_H
    #define USERDEFINED_H

    #include "robot.h" // Include necessary headers

    namespace thunder {
        namespace userDefined {
            // Add your function declaration here:
            int add_getMyCustomValue(Robot& robot);

            // Add declarations for other custom functions...

        } // namespace userDefined
    } // namespace thunder
    #endif // USERDEFINED_H
    ```

3.  **Implement the Function Logic:**
    Open `src/thunder/src/userDefined.cpp`. Include the header and implement your function. Inside the function, you will:
    *   Access robot properties like joint variables (`q`, `dq`) and parameters using `Robot` class methods (e.g., `robot.get_q()`, `robot.get_yaml_param("custom_params.my_gain")`). Remember these return CasADi `SX` symbolic variables.
    *   Perform your calculations using CasADi symbolic operations.
    *   Use `robot.addFunction()` to register your symbolic expression with the `Robot` object.

    ```c++
    // In src/thunder/src/userDefined.cpp
    #include "userDefined.h"
    #include "kinematics.h" // Include if you need kinematic results
    #include "dynamics.h" // Include if you need dynamic results
    #include <casadi/casadi.hpp> // Include CasADi

    namespace thunder {
        namespace userDefined {

            int add_getMyCustomValue(Robot& robot) {
                // Get symbolic joint variables (q)
                casadi::SX q = robot.get_q();

                // Get the symbolic custom gain parameter from the YAML config
                // The path matches the YAML structure.
                casadi::SX custom_gain = robot.get_yaml_param("custom_params.my_gain");

                // Perform symbolic calculation using CasADi functions
                // Example: sin(q_1) * custom_gain[0]
                // Note: CasADi SX indexing is 0-based.
                casadi::SX custom_value = sin(q(0)) * custom_gain(0);

                // Add the function to the robot object
                // Arguments to addFunction:
                // 1. Function name (will become "get_<name>" in the generated class)
                // 2. Symbolic input variables (usually q, dq, parameters)
                // 3. Symbolic output expression
                // 4. Input variable names (for generated C code)
                // 5. Output variable name (for generated C code)
                robot.addFunction("MyCustomValue", // Name -> get_MyCustomValue
                                  {q, custom_gain}, // Inputs needed
                                  {custom_value},   // Output expression
                                  {"q", "custom_gain"}, // Input names in C code
                                  {"custom_value_out"}); // Output name in C code

                // This will generate a method in the robot class:
                // Eigen::VectorXd get_MyCustomValue(const Eigen::VectorXd& q,
                //                                    const Eigen::VectorXd& custom_gain);

                return 1; // Return success
            }

            // Implement other custom functions...

        } // namespace userDefined
    } // namespace thunder
    ```
    *   **Important:** The `Robot::addFunction` call registers the computation. The first argument (`"MyCustomValue"`) determines the name of the getter method in the generated class (`get_MyCustomValue`). Ensure the symbolic inputs listed (`{q, custom_gain}`) include everything needed to compute the output expression.


4.  **Rebuild the `thunder` Executable:**
    Since you modified the `thunder` source code, you need to recompile it.
    ```bash
    cd /path/to/thunder_dynamics/src/thunder/build
    make -j$(nproc)
    # Optional: sudo make install if you installed it system-wide
    ```

5.  **Regenerate the Robot Library:**
    Run `thunder gen` again for your robot.
    ```bash
    ./bin/thunder gen [options] <path/to/robot.yaml>
    ```

6.  **Use the New Function:**
    Your custom function is now available in the generated `thunder_<robot_name>` class.

    *   **C++:**
        ```c++
        #include "thunder_<robot_name>.h"
        // ... setup robot instance, setArguments, load_conf ...
        Eigen::VectorXd custom_result = robot.get_MyCustomValue();
        std::cout << "Custom Value: " << custom_result(0) << std::endl;
        ```
    *   **Python:**
        ```python
        # ... import module, setup robot instance, set_q, load_conf ...
        custom_result = robot.get_MyCustomValue()
        print(f"Custom Value: {custom_result[0]}")
        ```

By following these steps, you can tailor Thunder Dynamics to perform specialized calculations specific to your robotics application.
