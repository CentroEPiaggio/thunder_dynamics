"""
Thunder Dynamics PyTorch Code Generator Plugin

This plugin generates PyTorch-compatible code for the Robot model.

It provides:
1. Per-function PyTorch code generation with batched vectorization
2. A wrapper class (ThunderRobotTorch) mirroring the C++ thunder_robot template
3. Full parameter and state management 

Usage in YAML:
    pipeline:
      generators: ["robot_generator", "py:torch_generator.TorchGenerator"]
    
    "py:torch_generator.TorchGenerator":
      output_dir: "./torch_generated"
      generate_wrapper: true
      verbose: false

Dependencies:
- CasADi (for symbolic function introspection)
- PyTorch (for generated code and wrapper)
- pydantic (for config validation)

Authors: Simone Tolomei
"""

import os
import re
import textwrap
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import casadi
from pydantic import BaseModel, Field
from thunder_core.plugins import BaseGenerator


# ============================================================================
# PyTorch Operation Mapping
# ============================================================================
# Maps CasADi operation IDs to PyTorch code templates (from Cusadi)
from .cusadiops import OP_PYTORCH_DICT

# ============================================================================
# Code Generation Functions
# ============================================================================



def modified_generate_pytorch_code(f: casadi.Function):
    """
    Generates PyTorch code for a CasADi function by analyzing its computational graph
    and mapping each operation to its PyTorch equivalent.
    
    Args:
        f (casadi.Function): CasADi function object loaded from .casadi file
        
    Returns:
        str: Complete Python function definition with metadata
        
    Notes:
        - Generated functions follow naming convention: _{original_name}
        - Function signature: func(outputs, inputs, work) for in-place computation
        - Memory layout matches CasADi's graph
        
    Raises:
        NotImplementedError: If CasADi operation has no PyTorch equivalent
    """
    f_name = f.name()

    # ==================== CasADi Function Analysis ====================

    # Core function properties
    n_instr = f.n_instructions()   # Number of elementary operations in the graph
    n_in = f.n_in()                # Number of input arguments
    n_out = f.n_out()              # Number of output arguments  
    n_w = f.sz_w()                 # Size of work vector (intermediate storage)

    # Each instruction represents one elementary operation (add, mul, sin, etc.)
    input_idx = [f.instruction_input(i) for i in range(n_instr)]      # Input indices for each op
    output_idx = [f.instruction_output(i) for i in range(n_instr)]    # Output indices for each op
    operations = [f.instruction_id(i) for i in range(n_instr)]        # Operation type IDs
    const_instr = [f.instruction_constant(i) for i in range(n_instr)] # Constant values

    # CasADi names are retained only for debugging. Compiled Thunder functions
    # often call inputs i0, i1, ...; wrapper methods must use Function.args.
    input_names = [f.name_in(i) for i in range(n_in)]    # Symbolic names (e.g., 'q', 'dq', 'tau')
    output_names = [f.name_out(i) for i in range(n_out)] # Output names (e.g., 'ddq', 'tau_ext')

    # ==================== Code Generation ====================
    # We will append '_' to all variables and functions to avoid conflicts in the wrapper
    n_in_str = f"_{f_name}_N_IN = {n_in}\n"
    n_out_str = f"_{f_name}_N_OUT = {n_out}\n"
    n_instr_str = f"_{f_name}_N_INSTR = {n_instr}\n"
    
    # CasADi stores sparse matrices as flat non-zero arrays. The wrapper needs
    # these counts to allocate the flat buffers before reshaping the result.
    nnz_in_str = f"_{f_name}_NNZ_IN = {[f.nnz_in(i) for i in range(n_in)]}\n"
    nnz_out_str = f"_{f_name}_NNZ_OUT = {[f.nnz_out(i) for i in range(n_out)]}\n"
    
    input_names_str = f"_{f_name}_INPUT_NAMES = {input_names}\n"
    output_names_str = f"_{f_name}_OUTPUT_NAMES = {output_names}\n"

    n_w_str = f"_{f_name}_SZ_W = {n_w}\n"

    # Combine all metadata into a preamble
    preamble_variables = n_in_str + n_out_str + n_instr_str + n_w_str + nnz_in_str + nnz_out_str + input_names_str + output_names_str
    
    # Generate function.
    # TODO: in theory we could use torch.compile or torch.jit, but these functions may easily
    # get way too large (hundreds of thoudsands of operations) so sadly it's not a good idea.
    # TODO: might be wise to check if we can wrap these functios as torch.nn.Module
    function_signature = f"def _{f_name}(outputs, inputs, work):"
    str_operations = preamble_variables + function_signature

    # ==================== Operation Translation ====================
    # Convert each CasADi instruction to equivalent PyTorch operation
    # This basically comes from Cusadi.
    
    for k in range(n_instr):
        op = operations[k]             # Current operation type
        o_idx = output_idx[k]          # Where to store result
        i_idx = input_idx[k]           # Input operand indices

        if op == casadi.OP_CONST:
            const_val = const_instr[k]
            # Try to coerce scalar constants to float, otherwise raise informative error
            try:
                const_scalar = float(const_val)
                str_operations += OP_PYTORCH_DICT[casadi.OP_CONST] % (o_idx[0], const_scalar)
            except Exception:
                raise ValueError(
                    f"Non-scalar constant encountered in function '{f_name}' "
                    f"at instruction {k} (output idx {o_idx}). Constant type: {type(const_val)}; value: {const_val}"
                )

        elif op == casadi.OP_INPUT:
            str_operations += OP_PYTORCH_DICT[casadi.OP_INPUT] % (o_idx[0], i_idx[0], i_idx[1])

        elif op == casadi.OP_OUTPUT:
            str_operations += OP_PYTORCH_DICT[casadi.OP_OUTPUT] % (o_idx[0], o_idx[1], i_idx[0])

        elif op == casadi.OP_SQ:
            str_operations += OP_PYTORCH_DICT[casadi.OP_SQ] % (o_idx[0], i_idx[0], i_idx[0])

        elif OP_PYTORCH_DICT[op].count("%d") == 3:
            str_operations += OP_PYTORCH_DICT[op] % (o_idx[0], i_idx[0], i_idx[1])

        elif OP_PYTORCH_DICT[op].count("%d") == 2:
            str_operations += OP_PYTORCH_DICT[op] % (o_idx[0], i_idx[0])

        else:
            raise NotImplementedError(
                f"CasADi operation '{op}' is not supported. Add mapping to OP_PYTORCH_DICT in cusadiops.py"
            )

    return str_operations + "\n"




def generate_pytorch_code(f: casadi.Function) -> str:
    """
    Generate PyTorch code for a CasADi function by analyzing its computational graph.

    Args:
        f: CasADi function object

    Returns:
        String containing complete Python function definition with metadata

    Raises:
        NotImplementedError: If CasADi operation has no PyTorch equivalent
    """
    f_name = f.name()

    # Analyze function structure
    n_instr = f.n_instructions()
    n_in = f.n_in()
    n_out = f.n_out()
    n_w = f.sz_w()

    # Extract instruction details
    input_idx = [f.instruction_input(i) for i in range(n_instr)]
    output_idx = [f.instruction_output(i) for i in range(n_instr)]
    operations = [f.instruction_id(i) for i in range(n_instr)]
    const_instr = [f.instruction_constant(i) for i in range(n_instr)]

    # Extract names
    input_names = [f.name_in(i) for i in range(n_in)]
    output_names = [f.name_out(i) for i in range(n_out)]

    # Metadata strings (all variables prefixed with '_' to avoid conflicts)
    preamble = (
        f"_{f_name}_N_IN = {n_in}\n"
        f"_{f_name}_N_OUT = {n_out}\n"
        f"_{f_name}_N_INSTR = {n_instr}\n"
        f"_{f_name}_SZ_W = {n_w}\n"
        f"_{f_name}_NNZ_IN = {[f.nnz_in(i) for i in range(n_in)]}\n"
        f"_{f_name}_NNZ_OUT = {[f.nnz_out(i) for i in range(n_out)]}\n"
        f"_{f_name}_INPUT_NAMES = {input_names}\n"
        f"_{f_name}_OUTPUT_NAMES = {output_names}\n"
    )

    # Function signature
    func_def = f"def _{f_name}(outputs, inputs, work):"
    code = preamble + func_def

    # Generate operation instructions
    for k in range(n_instr):
        op = operations[k]
        o_idx = output_idx[k]
        i_idx = input_idx[k]

        if op not in OP_PYTORCH_DICT:
            raise NotImplementedError(
                f"CasADi operation '{op}' is not supported. "
                f"Add mapping to OP_PYTORCH_DICT"
            )

        template = OP_PYTORCH_DICT[op]
        num_placeholders = template.count("%d") + template.count("%f")

        # Handle special cases
        if op == casadi.OP_CONST:
            code += template % (o_idx[0], const_instr[k])
        elif op == casadi.OP_INPUT:
            code += template % (o_idx[0], i_idx[0], i_idx[1])
        elif op == casadi.OP_OUTPUT:
            code += template % (o_idx[0], o_idx[1], i_idx[0])
        elif op == casadi.OP_SQ:
            code += template % (o_idx[0], i_idx[0], i_idx[0])
        elif num_placeholders == 3:  # Binary operations
            code += template % (o_idx[0], i_idx[0], i_idx[1])
        elif num_placeholders == 2:  # Unary operations
            code += template % (o_idx[0], i_idx[0])

    return code + "\n"

def get_robot_inventory(robot: Any) -> Dict[str, Any]:
    """Collect symbols after CasADi compilation.

    Thunder substitutes non-symbolic Robot parameters into each expression as
    constants. Only ``symb_size()`` values survive as function inputs, so using
    ``Parameter.size()`` here would create unused tensors and wrong interfaces.
    """
    parameter_sizes: Dict[str, int] = {}
    parameter_defaults: Dict[str, List[float]] = {}
    
    #! Parameter object, binded from cpp
    for par in robot.get_parameters():
        try:
            size = int(par.symb_size())
        except Exception:
            size = 0

        # Non-symbolic values are embedded in the CasADi expression.
        # Excluding them mirrors the C++ wrapper.
        if size <= 0:
            continue

        parameter_sizes[par.name] = size
        try:
            values = [float(value) for value in par.get_value_resized().nonzeros()]
            if len(values) == size:
                parameter_defaults[par.name] = values
        except Exception:
            # If the parameter has no default value, we simply skip it 
            # ! the wrapper has default zero, which probably may be wrong
            pass

    try:
        n_joints = int(robot.get_int("ndof"))
    except Exception:
        try:
            # For the sake of retrocompatibility
            n_joints = int(robot.get_int("numJoints"))
        except Exception as e:
            raise ValueError(
                "TorchGenerator requires robot property 'ndof' or 'numJoints'"
            ) from e

    if n_joints <= 0:
        raise ValueError("TorchGenerator requires a positive joint-count property")

    return {
        "n_joints": n_joints,
        "parameter_sizes": parameter_sizes,
        "parameter_defaults": parameter_defaults,
    }


def generate_wrapper_class(
    class_name: str,
    module_basename: str,
    n_joints: int,
    parameter_sizes: Dict[str, int],
    parameter_defaults: Dict[str, List[float]],
    robot_name: str,
    functions_meta: List[Dict[str, Any]],
) -> str:
    """Generate the wrapper class code."""
    wrapper_header = textwrap.dedent(f"""
    # ! AUTOMATICALLY GENERATED WRAPPER
    # Wrapper class for PyTorch robotics dynamics
    import torch
    
    # Prefer package-relative imports; keep absolute fallback for standalone usage.
    try:
        from .thunder_robot_torch import ThunderRobotTorch
        from . import {module_basename} as gen
    except ImportError:
        from thunder_robot_torch import ThunderRobotTorch
        import {module_basename} as gen
    
    class {class_name}(ThunderRobotTorch):
        \"\"\"
        PyTorch wrapper for Thunder robot dynamics.
        
        Inherits from ThunderRobotTorch for batched state and parameter management.
        Provides auto-generated methods for each CasADi function.
        \"\"\"
        
        def __init__(self, batch_size: int = 1, device: str = "cpu", dtype: torch.dtype = torch.double):
            super().__init__(
                n_joints={n_joints},
                batch_size=batch_size,
                device=device,
                dtype=dtype,
                robotName={robot_name!r},
                parameter_sizes={parameter_sizes!r},
                parameter_defaults={parameter_defaults!r},
            )
    """)

    #! We use the global get_parmeter from the base torch class
    parameter_accessors = []
    for parameter_name, parameter_size in parameter_sizes.items():
        if not parameter_name.isidentifier():
            raise ValueError(
                f"Cannot generate accessors for invalid Python parameter name "
                f"{parameter_name!r}"
            )
        parameter_accessors.append(textwrap.dedent(f"""
        def set_{parameter_name}(self, value) -> None:
            \"\"\"Set the `{parameter_name}` parameter ({parameter_size} values).\"\"\"
            self.set_parameter({parameter_name!r}, value)

        def get_{parameter_name}(self) -> torch.Tensor:
            \"\"\"Get a copy of the `{parameter_name}` parameter.\"\"\"
            return self.get_parameter({parameter_name!r})
        """))

    # Generate function methods
    methods = []
    for meta in functions_meta:
        fname = meta["name"]
        iname = meta["internal_name"]
        out_shape = meta["output_shape"]
        parameter_input_names = meta["parameter_input_names"]
        explicit_args = meta["explicit_args"]
        explicit_signature = ", ".join(arg["name"] for arg in explicit_args)
        method_signature = f"self, {explicit_signature}" if explicit_signature else "self"
        explicit_inputs = ", ".join(
            f"self._prepare_explicit_input({arg['name']}, {arg['size']}, {arg['name']!r})"
            for arg in explicit_args
        )
        input_expression = (
            f"[getattr(self, name) for name in {parameter_input_names!r}]"
            + (f" + [{explicit_inputs}]" if explicit_inputs else "")
        )

        method = textwrap.dedent(f"""
        def {fname}({method_signature}):
            \"\"\"
            Compute {iname} using generated PyTorch code.
            
            Returns:
                torch.Tensor of shape (batch_size, {out_shape[0]}, {out_shape[1]})
            \"\"\"
            B = self.batch_size
            _NNZ_OUT = gen._{iname}_NNZ_OUT
            _SZ_W = gen._{iname}_SZ_W
            
            # Function.args preserves the parameter order used to define this function.
            # We do not use gen._*_INPUT_NAMES because CasADi may
            # replace robot names with anonymous labels such as i0 and i1.
            inputs = {input_expression}
            
            # CasADi instructions write flat non-zero output values. `work` is
            # the graph's temporary storage and is required even when no Python
            # intermediate values are visible at this level.
            outputs = [
                torch.empty((B, n), device=self.device, dtype=self.dtype).contiguous()
                for n in _NNZ_OUT
            ]
            work = torch.empty((B, _SZ_W), device=self.device, dtype=self.dtype)
            
            # Call generated computation
            gen._{iname}(outputs, inputs, work)
            
            # Restore the dense matrix/vector shape promised by the Robot API.
            out = outputs[0].reshape((B, {out_shape[0]}, {out_shape[1]}))
            return out
        """)
        methods.append(method)

    return wrapper_header + "\n".join(
        textwrap.indent(method, "    ")
        for method in parameter_accessors + methods
    ) + "\n"


def sanitize_package_name(raw_name: str) -> str:
    """The yaml config may contain whatever so we sanitize it to a valid Python package name."""
    cleaned = re.sub(r"[^0-9a-zA-Z_]+", "_", (raw_name or "").strip())
    if not cleaned:
        cleaned = "thunder_generated"
    if cleaned[0].isdigit():
        cleaned = f"thunder_{cleaned}"
    return cleaned


def generate_package_init(module_basename: str, wrapper_class_name: Optional[str]) -> str:
    """Create __init__.py content for generated package exports."""
    lines = [
        '"""Auto-generated Thunder Dynamics PyTorch package."""',
        "",
        f"from . import {module_basename}",
    ]
    exports = [f'"{module_basename}"']

    if wrapper_class_name:
        lines += [f"from .torch_robot_wrapper import {wrapper_class_name}"]
        exports.append(f'"{wrapper_class_name}"')

    lines += ["", f"__all__ = [{', '.join(exports)}]", ""]
    return "\n".join(lines)


def generate_pyproject(package_name: str) -> str:
    """Create a minimal pyproject.toml for editable installs."""
    return textwrap.dedent(f"""
    [build-system]
    requires = ["setuptools>=61"]
    build-backend = "setuptools.build_meta"

    [project]
    name = "{package_name}"
    version = "0.1.0"
    description = "Auto-generated Thunder Dynamics PyTorch package"
    requires-python = ">=3.8"
    dependencies = [
      "torch",
      "casadi",
    ]

    [tool.setuptools]
    py-modules = ["gen_torch_functions", "torch_robot_wrapper", "thunder_robot_torch"]
    """).strip() + "\n"


# ============================================================================
# Main Plugin Class
# ============================================================================


class TorchGenerator(BaseGenerator):
    """
    Thunder PyTorch code generator plugin.

    Generates batched PyTorch implementations of CasADi functions from the Robot model.
    Produces two files:
    1. Generated function code (torch operations)
    2. Wrapper class inheriting from ThunderRobotTorch

    Config options:
    - output_dir: Directory to write generated code (default: "./torch_generated")
    - generate_wrapper: Whether to generate wrapper class (default: true)
    - verbose: Print detailed generation progress (default: false)
    """

    class ConfigModel(BaseModel):
        """ Pydantic model. It will be validated at runtime and will throw errors if the config is invalid. """

        output_dir: str = Field(
            default="./torch_generated",
            description="Directory for generated PyTorch code",
        )
        generate_wrapper: bool = Field(
            default=True,
            description="Generate wrapper class inheriting from ThunderRobotTorch",
        )
        verbose: bool = Field(
            default=False,
            description="Print detailed generation progress",
        )
        generate_install_files: bool = Field(
            default=True,
            description="Generate minimal pyproject.toml and setup.py for pip install",
        )

    def generate(self, robot) -> None:
        """
        Generate PyTorch code.

        Args:
            robot: Thunder Robot instance 
        """
        output_dir = Path(self.config.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        if self.config.verbose:
            print(f"[TorchGenerator] Generating PyTorch code to: {output_dir}")

        # Collect all functions from robot
        functions = robot.functions
        if not functions:
            print("[TorchGenerator] No functions found in robot model. Skipping generation.")
            return

        if self.config.verbose:
            print(f"[TorchGenerator] Found {len(functions)} functions to convert")

        # Generate code for each function
        all_generated_code = textwrap.dedent("""
        # ! AUTOMATICALLY GENERATED CODE
        # Generated by Thunder Dynamics PyTorch Code Generator
        # 
        # WARNING: Do not edit manually - changes will be overwritten
        # Regenerate using the TorchGenerator plugin
        
        import torch
        
        """)

        functions_meta: List[Dict[str, Any]] = []
        successful = 0
        failed = []
        wrapper_class_name: Optional[str] = None

        for func_name, func_obj in functions.items():
            try:
                if self.config.verbose:
                    print(f"  Converting: {func_name}")

                # Access CasADi function: we use the compiled 'fun'
                casadi_func = None
                # func_obj is the entry from robot.functions mapping (C++ Function wrapper)
                if hasattr(func_obj, "fun") and getattr(func_obj, "fun") is not None:
                    casadi_func = func_obj.fun
                elif hasattr(func_obj, "expr") and getattr(func_obj, "expr") is not None:
                    # We cannot reliably generate from raw SX expressions here; skip with message
                    raise ValueError(
                        f"Function '{func_name}' has only symbolic expr (SX). Export or compile to a CasADi Function first."
                    )
                else:
                    raise ValueError(f"Function '{func_name}' has no accessible CasADi Function object")

                # Generate PyTorch code from casadi.Function
                generated_code = modified_generate_pytorch_code(casadi_func)
                all_generated_code += generated_code + "\n"

                # Collect metadata used by the high-level wrapper. The raw
                # function is still needed for shapes and instruction code.
                output_shape = (
                    casadi_func.sparsity_out(0).size1(),
                    casadi_func.sparsity_out(0).size2(),
                )
                # `func_obj.args` have the names and order used by add_function()
                # CasADi  `name_in()` only say i0, i1...
                parameter_input_names = list(func_obj.args)
                explicit_args = [
                    {"name": arg.name, "size": int(arg.size())}
                    for arg in getattr(func_obj, "explicit_args", [])
                ]
                expected_inputs = len(parameter_input_names) + len(explicit_args)
                # A mismatch indicates stale/incomplete metadata; fail during
                # generation instead of producing a wrapper that fails later.
                if expected_inputs != casadi_func.n_in():
                    raise ValueError(
                        f"Function '{func_name}' metadata has {expected_inputs} inputs, "
                        f"but its CasADi function has {casadi_func.n_in()}"
                    )
                functions_meta.append({
                    "name": f"get_{func_name}",
                    "internal_name": casadi_func.name(),
                    "nnz_in": [casadi_func.nnz_in(i) for i in range(casadi_func.n_in())],
                    "nnz_out": [casadi_func.nnz_out(i) for i in range(casadi_func.n_out())],
                    "parameter_input_names": parameter_input_names,
                    "explicit_args": explicit_args,
                    "output_names": [casadi_func.name_out(i) for i in range(casadi_func.n_out())],
                    "sz_w": casadi_func.sz_w(),
                    "output_shape": output_shape,
                })

                successful += 1

            except Exception as e:
                failed.append((func_name, str(e)))
                if self.config.verbose:
                    print(f"    ERROR: {e}")
                all_generated_code += f"\n# ERROR: Could not convert {func_name}\n"
                all_generated_code += f"# Reason: {e}\n\n"

        # Write generated code
        gen_file = output_dir / "gen_torch_functions.py"
        try:
            with open(gen_file, "w") as f:
                f.write(all_generated_code)
            print(f"[TorchGenerator] Generated: {gen_file}")
            print(f"[TorchGenerator] Successfully converted {successful}/{len(functions)} functions")
            if failed:
                print(f"[TorchGenerator] Failed conversions: {len(failed)}")
                for name, err in failed:
                    print(f"  - {name}: {err}")
        except Exception as e:
            print(f"[TorchGenerator] ERROR writing generated code: {e}")
            return

        # Generate wrapper class if requested
        if self.config.generate_wrapper and functions_meta:
            try:
                robot_name = str(robot.robotName)
                class_suffix = re.sub(r"[^0-9a-zA-Z_]+", "_", robot_name).title() or "Robot"
                class_name = f"ThunderRobot{class_suffix}Torch"
                module_basename = "gen_torch_functions"

                inventory = get_robot_inventory(robot)

                wrapper_code = generate_wrapper_class(
                    class_name,
                    module_basename,
                    inventory["n_joints"],
                    inventory["parameter_sizes"],
                    inventory["parameter_defaults"],
                    robot_name,
                    functions_meta,
                )

                wrapper_file = output_dir / "torch_robot_wrapper.py"
                with open(wrapper_file, "w") as f:
                    f.write(wrapper_code)
                print(f"[TorchGenerator] Generated wrapper: {wrapper_file}")
                print(f"[TorchGenerator] Wrapper class name: {class_name}")
                wrapper_class_name = class_name

            except Exception as e:
                print(f"[TorchGenerator] ERROR generating wrapper: {e}")

        # Copy runtime base class dependency into generated package folder.
        runtime_src = Path(__file__).resolve().parent / "thunder_robot_torch.py"
        runtime_dst = output_dir / "thunder_robot_torch.py"
        try:
            if runtime_src.exists():
                with open(runtime_src, "r") as src, open(runtime_dst, "w") as dst:
                    dst.write(src.read())
                if self.config.verbose:
                    print(f"[TorchGenerator] Copied runtime base: {runtime_dst}")
            else:
                print(f"[TorchGenerator] WARNING: Runtime base not found at {runtime_src}")
        except Exception as e:
            print(f"[TorchGenerator] WARNING: Could not copy runtime base class: {e}")

        # Ensure output directory is importable as a package.
        init_file = output_dir / "__init__.py"
        try:
            with open(init_file, "w") as f:
                f.write(generate_package_init("gen_torch_functions", wrapper_class_name))
            if self.config.verbose:
                print(f"[TorchGenerator] Generated package init: {init_file}")
        except Exception as e:
            print(f"[TorchGenerator] WARNING: Could not generate __init__.py: {e}")

        # Add minimal installation metadata so users can `pip install -e <output_dir>`.
        if self.config.generate_install_files:
            package_name = sanitize_package_name(output_dir.name)
            pyproject_file = output_dir / "pyproject.toml"
            setup_file = output_dir / "setup.py"

            try:
                with open(pyproject_file, "w") as f:
                    f.write(generate_pyproject(package_name))

                setup_code = textwrap.dedent("""
                from setuptools import setup

                setup()
                """).strip() + "\n"
                with open(setup_file, "w") as f:
                    f.write(setup_code)

                if self.config.verbose:
                    print(f"[TorchGenerator] Generated install metadata: {pyproject_file}, {setup_file}")
            except Exception as e:
                print(f"[TorchGenerator] WARNING: Could not generate install metadata: {e}")

        if self.config.verbose:
            print(f"[TorchGenerator] Generation complete")
