"""
Thunder Dynamics PyTorch Code Generator

This module generates PyTorch-compatible code from CasADi functions stored in .casadi files.

Dependencies:
- CasADi
- PyTorch
- cusadiops: Custom dictionary mapping CasADi operations to PyTorch equivalents

Author: Thunder Dynamics Team
Status: Work in Progress - Handle with care, validate outputs
"""

import os
import textwrap
import argparse
from casadi import *
from cusadiops import *
from tqdm import tqdm
import shutil

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

    # Extract input and output names for metadata
    input_names = [f.name_in(i) for i in range(n_in)]    # Symbolic names (e.g., 'q', 'dq', 'tau')
    output_names = [f.name_out(i) for i in range(n_out)] # Output names (e.g., 'ddq', 'tau_ext')

    # ==================== Code Generation ====================
    # We will append '_' to all variables and functions to avoid conflicts in the wrapper
    n_in_str = f"_{f_name}_N_IN = {n_in}\n"
    n_out_str = f"_{f_name}_N_OUT = {n_out}\n"
    n_instr_str = f"_{f_name}_N_INSTR = {n_instr}\n"
    
    # Sparsity pattern information (number of non-zeros)
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
        
        if op == OP_CONST:
            # Load constant value: work[o_idx] = constant
            str_operations += OP_PYTORCH_DICT[op] % (o_idx[0], const_instr[k])
            
        elif op == OP_INPUT:
            # Copy from input buffer: work[o_idx] = inputs[input_idx][element_idx]
            str_operations += OP_PYTORCH_DICT[op] % (o_idx[0], i_idx[0], i_idx[1])
            
        elif op == OP_OUTPUT:
            # Copy to output buffer: outputs[output_idx][element_idx] = work[i_idx]
            str_operations += OP_PYTORCH_DICT[op] % (o_idx[0], o_idx[1], i_idx[0])
            
        elif op == OP_SQ:
            # Square operation: work[o_idx] = work[i_idx] * work[i_idx]
            str_operations += OP_PYTORCH_DICT[op] % (o_idx[0], i_idx[0], i_idx[0])
            
        # Generic operation handling based on argument count
        elif OP_PYTORCH_DICT[op].count("%d") == 3:
            # Binary operations: work[o_idx] = work[i_idx[0]] op work[i_idx[1]]
            str_operations += OP_PYTORCH_DICT[op] % (o_idx[0], i_idx[0], i_idx[1])
            
        elif OP_PYTORCH_DICT[op].count("%d") == 2:
            # Unary operations: work[o_idx] = op(work[i_idx[0]])
            str_operations += OP_PYTORCH_DICT[op] % (o_idx[0], i_idx[0])
            
        else:
            # Operation not yet implemented in OP_PYTORCH_DICT
            raise NotImplementedError(
                f"CasADi operation '{op}' is not supported. "
                f"Add mapping to OP_PYTORCH_DICT in cusadiops.py"
            )

    return str_operations + "\n"

def _derive_robot_name(robot_dir: str) -> str:
    base = os.path.basename(os.path.normpath(robot_dir)) if robot_dir else "robot"
    # Heuristics to strip common suffixes
    for suf in ["_generatedFiles", "-generatedFiles", "generatedFiles"]:
        if base.endswith(suf):
            base = base[: -len(suf)] or base
            break
    return base.replace("-", "_")


def main(cusadi_folder="casadi_functions", output_dir=None, output_file="gen_file.py"):
    """
    Batch processes all .casadi files in a directory and generates a unified PyTorch module.
    
    Args:
        cusadi_folder (str): Path to directory containing .casadi files
                           Expected structure: robot_dir/casadi_functions/*.casadi
        output_dir (str, optional): Target directory for generated code
                                  Creates directory if it doesn't exist
        output_file (str): Name of generated Python file (default: "gen_file.py")
        
    """

    if not os.path.exists(cusadi_folder):
        print(f"Error: The folder '{cusadi_folder}' was not found.")
        print(f"Expected structure: robot_dir/casadi_functions/*.casadi")
        return

    # Discover all CasADi function files
    cusadi_files = [f for f in os.listdir(cusadi_folder) if f.endswith(".casadi")]

    if not cusadi_files:
        print(f"No .casadi files were found in '{cusadi_folder}'.")
        print(f"Ensure CasADi functions have been exported to this directory.")
        return

    print(f"Found {len(cusadi_files)} .casadi files to process:")
    for file in cusadi_files:
        print(f"  - {file}")

    if output_dir:
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, output_file)
        print(f"Output directory: {output_dir}")
    else:
        output_path = output_file
        print(f"Output to current directory: {os.getcwd()}")

    # ==================== Code Generation ====================
    # Initialize
    all_generated_code = textwrap.dedent('''
    # ! AUTOMATICALLY GENERATED CODE
    # Generated by Thunder Dynamics PyTorch Code Generator
    # Source: CasADi functions from robotics dynamics computations
    # 
    # WARNING: Do not edit manually - changes will be overwritten
    # Regenerate using: python thunder_torch_gen.py --robot_dir <path>
    
    import torch

    ''')

    # Process each .casadi file and accumulate generated code
    successful_conversions = 0
    failed_conversions = []

    # Track metadata for wrapper generation
    functions_meta = []  # list of dicts: name, nnz_in, nnz_out, input_names, output_names, sz_w

    pbar = tqdm(cusadi_files, desc="Processing files", unit="file")
    for cusadi_file in pbar:
        try:
            func_path = os.path.join(cusadi_folder, cusadi_file)

            f = Function.load(func_path)
            
            generated_code = modified_generate_pytorch_code(f)
            
            all_generated_code += generated_code

            #function name is 'get_' + cusadi_file_name without .casadi
            function_name = os.path.splitext(cusadi_file)[0]

            # get output shape
            output_shape = (f.sparsity_out(0).size1(), f.sparsity_out(0).size2())
            # stash meta used for wrapper
            functions_meta.append({
                "name": "get_" + function_name,
                "internal_name": f.name(),
                "nnz_in": [f.nnz_in(i) for i in range(f.n_in())],
                "nnz_out": [f.nnz_out(i) for i in range(f.n_out())],
                "input_names": [f.name_in(i) for i in range(f.n_in())],
                "output_names": [f.name_out(i) for i in range(f.n_out())],
                "sz_w": f.sz_w(),
                "output_shape": output_shape,
            })
            
            successful_conversions += 1
            
        except Exception as e:
            failed_conversions.append((cusadi_file, str(e)))
            
            all_generated_code += f"\n# ERROR: Could not convert {cusadi_file}\n"
            all_generated_code += f"# Reason: {e}\n\n"

    # Write all the generated code to the output file
    try:
        with open(output_path, "w") as out_f:
            out_f.write(all_generated_code)
        
        print(f"\n{'='*60}")
        print(f"Code generation completed!")
        print(f"Output file: {output_path}")
        print(f"Successfully converted: {successful_conversions}/{len(cusadi_files)} functions")
        
        if failed_conversions:
            print(f"\nFailed conversions:")
            for file, error in failed_conversions:
                print(f"  - {file}: {error}")
                
    except Exception as e:
        print(f"\nFATAL ERROR: Could not write output file '{output_path}': {e}")
        return

    print(f"{'='*60}")

    # ==================== Wrapper Class Generation ====================
    # Derive names and sizes
    robot_name = _derive_robot_name(os.path.dirname(cusadi_folder))
    module_basename = os.path.splitext(os.path.basename(output_file))[0]
    class_name = f"thunder_{robot_name}"
    wrapper_filename = f"{class_name}_torch.py"  # module containing the wrapper class
    wrapper_path = os.path.join(output_dir if output_dir else os.getcwd(), wrapper_filename)

    # Aggregate input sizes across functions to infer robot dimensions
    agg_in_sizes = {}
    for meta in functions_meta:
        for nm, sz in zip(meta["input_names"], meta["nnz_in"]):
            agg_in_sizes[nm] = max(sz, agg_in_sizes.get(nm, 0))

    def _infer_int(name: str, default: int = 0) -> int:
        return int(agg_in_sizes.get(name, default))

    # Infer core sizes
    n_joints = _infer_int("q", default=_infer_int("dq", default=_infer_int("ddq", default=0)))
    numElasticJoints = _infer_int("x", default=0)
    # Orders via division where available
    Dl_order = 0
    if n_joints > 0 and agg_in_sizes.get("par_Dl", 0) % max(n_joints, 1) == 0:
        Dl_order = agg_in_sizes.get("par_Dl", 0) // max(n_joints, 1)
    K_order = 0
    if numElasticJoints > 0 and agg_in_sizes.get("par_K", 0) % max(numElasticJoints, 1) == 0:
        K_order = agg_in_sizes.get("par_K", 0) // max(numElasticJoints, 1)
    D_order = 0
    if numElasticJoints > 0 and agg_in_sizes.get("par_D", 0) % max(numElasticJoints, 1) == 0:
        D_order = agg_in_sizes.get("par_D", 0) // max(numElasticJoints, 1)
    Dm_order = 0
    if numElasticJoints > 0 and agg_in_sizes.get("par_Dm", 0) % max(numElasticJoints, 1) == 0:
        Dm_order = agg_in_sizes.get("par_Dm", 0) // max(numElasticJoints, 1)

    # Build wrapper source
    wrapper_header = textwrap.dedent(f"""
    # ! AUTOMATICALLY GENERATED WRAPPER
    # Wrapper class for robot '{robot_name}'
    import torch
    from thunder_robot import ThunderRobotTorch

    import {module_basename} as gen

    class {class_name}(ThunderRobotTorch):
        def __init__(self, batch_size: int = 1, device: str = 'cpu'):
            super().__init__(
                n_joints={n_joints or 0},
                numElasticJoints={numElasticJoints or 0},
                K_order={K_order or 0},
                D_order={D_order or 0},
                Dl_order={Dl_order or 0},
                Dm_order={Dm_order or 0},
                batch_size=batch_size,
                device=device,
            )

       
    """)

    # Generate one method per function
    methods_code = []
    for meta in functions_meta:
        fname = meta["name"]
        iname = meta["internal_name"]
        method = [f"    def {fname}(self):"]
        method.append("        B = self.batch_size")
        # method.append(f"        _N_OUT = gen._{fname}_N_OUT")
        method.append(f"        _NNZ_OUT = gen._{iname}_NNZ_OUT")
        # method.append(f"        _N_IN = gen._{fname}_N_IN")
        # method.append(f"        _NNZ_IN = gen._{fname}_NNZ_IN")
        method.append(f"        _INPUT_NAMES = gen._{iname}_INPUT_NAMES")
        method.append(f"        _SZ_W = gen._{iname}_SZ_W")
        method.append( "        inputs = [getattr(self, in_name) for in_name in _INPUT_NAMES]")
        method.append( "        outputs = [torch.empty((B, n), device=self.device, dtype=self.dtype).contiguous() for n in _NNZ_OUT]")
        method.append( "        work = torch.empty((B, _SZ_W), device=self.device, dtype=self.dtype)")
        method.append(f"        gen._{iname}(outputs, inputs, work)")
        method.append(f"        out = outputs[0].reshape((B, {meta['output_shape'][0]}, {meta['output_shape'][1]}))")
        method.append( "        return out")
        methods_code.append("\n".join(method))

    wrapper_code = wrapper_header + "\n" + "\n\n".join(methods_code) + "\n"

    try:
        with open(wrapper_path, "w") as wf:
            wf.write(wrapper_code)
        print(f"Wrapper class generated: {wrapper_path}")
        print(f"Class name: {class_name}")
    except Exception as e:
        print(f"ERROR: Could not write wrapper file '{wrapper_path}': {e}")
   
    # Determine destination base directory once
    try:
        script_dir = os.path.dirname(os.path.realpath(__file__))
    except NameError:
        script_dir = os.getcwd()
    dest_base_dir = output_dir if output_dir else os.getcwd()

    # Copy thunder_robot.py (located next to this script) into the output directory (if provided)
    src_thunder = os.path.join(script_dir, "thunder_robot.py")
    if os.path.isfile(src_thunder):
        try:
            os.makedirs(dest_base_dir, exist_ok=True)
            dest_thunder = os.path.join(dest_base_dir, "thunder_robot.py")
            shutil.copy2(src_thunder, dest_thunder)
            print(f"Copied thunder_robot.py -> {dest_thunder}")
        except Exception as e:
            print(f"Warning: failed to copy thunder_robot.py to '{dest_base_dir}': {e}")
    else:
        print(f"Warning: thunder_robot.py not found at '{src_thunder}', skipping copy.")

    # Create a simple installable entry (setup.py) and a package initializer (__init__.py)
    try:
        dest_dir = dest_base_dir
        os.makedirs(dest_dir, exist_ok=True)

        wrapper_module = os.path.splitext(wrapper_filename)[0]  # e.g. 'thunder_robotname_py'
        pkg_name = f"thunder_{robot_name}_torch"

        # Write __init__.py to make the output directory importable and expose convenient names
        init_path = os.path.join(dest_dir, "__init__.py")
        init_content = textwrap.dedent(f"""
        # Auto-generated package initializer for '{pkg_name}'

        from {wrapper_module} import {class_name}

        __all__ = ["{class_name}"]
        """)
        with open(init_path, "w") as f_init:
            f_init.write(init_content)
        print(f"Created package initializer: {init_path}")

        # Write a minimal setup.py so the generated module directory can be installed with pip
        setup_path = os.path.join(dest_dir, "setup.py")


        setup_content = textwrap.dedent(f"""
        from setuptools import setup

        setup(
            name="{pkg_name}",
            version="0.0.1",
            description="Auto-generated Thunder Dynamics PyTorch modules for robot {robot_name}",
            py_modules=["{wrapper_module}"],
            install_requires=["torch"],
            author="Thunder Dynamics",
            classifiers=[
                "Programming Language :: Python :: 3",
            ],
        )
        """)
        with open(setup_path, "w") as f_setup:
            f_setup.write(setup_content)
        print(f"Created installer file: {setup_path}")

    except Exception as e:
        print(f"Warning: failed to create setup/init files: {e}")



if __name__ == '__main__':
    """
    Command-line interface for Thunder Dynamics PyTorch code generator.
    
    
    Usage Examples:
        # Specify robot directory (looks for casadi_functions/ subfolder)
        python thunder_torch_gen.py --robot_dir /path/to/franka_generatedFiles
        
        # Full specification with custom output
        python thunder_torch_gen.py \\
            --robot_dir /path/to/franka_generatedFiles \\
            --output_dir /path/to/torch_lib \\
    
    """
    parser = argparse.ArgumentParser()
    
    parser.add_argument('--robot_dir', type=str, 
                       help='Path to robot generated files directory containing casadi_functions/ folder')
    parser.add_argument('--output_dir', type=str,
                       help='Path to output directory for generated PyTorch library')
    
    args = parser.parse_args()
    
    # ==================== Path Resolution ====================
    if args.robot_dir:
        # Look for casadi_functions folder in the robot directory
        # This follows Thunder Dynamics convention: robot_dir/casadi_functions/*.casadi
        cusadi_folder = os.path.join(args.robot_dir, "casadi_functions")
        print(f"Using robot directory: {args.robot_dir}")
        print(f"Looking for CasADi functions in: {cusadi_folder}")
    else:
        cusadi_folder = "casadi_functions"
        print(f"Using default CasADi functions directory: {cusadi_folder}")
    
    # ==================== Execute Code Generation ====================
    main(cusadi_folder, args.output_dir)
