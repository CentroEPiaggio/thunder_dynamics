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

    pbar = tqdm(cusadi_files, desc="Processing files", unit="file")
    for cusadi_file in pbar:
        try:
            func_path = os.path.join(cusadi_folder, cusadi_file)


            f = Function.load(func_path)
            
            generated_code = modified_generate_pytorch_code(f)
            
            all_generated_code += generated_code
            
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


