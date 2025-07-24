import torch
import gen_file as f

print('Loaded gen_file.py')

import time



device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def run_dense():
    """
    Bare implementation to run a generated function, based on the assumption that in and outs are dense.
    Bare == uses only the generated stuff, no .casadi file
    """
    func = f._franka_C_fun
    n_in = f._franka_C_fun_N_IN
    n_out = f._franka_C_fun_N_OUT
    n_instr = f._franka_C_fun_N_INSTR
    n_w = f._franka_C_fun_SZ_W
    
    nnz_in = f._franka_C_fun_NNZ_IN
    nnz_out = f._franka_C_fun_NNZ_OUT

    NINSTANCES = 1
    work = torch.zeros((NINSTANCES, n_w), dtype=torch.double, device=device).contiguous()

    inputs = [torch.zeros((NINSTANCES, nnz_in[0]), dtype=torch.double, device=device).contiguous(),
                torch.randn((NINSTANCES, nnz_in[1]), dtype=torch.double, device=device).contiguous()]
    
    outputs = [torch.zeros((NINSTANCES, nnz_out[0])).contiguous()]

    start_time = time.time()

    check = func(outputs, inputs, work)
    torch.cuda.synchronize() 
    eval_time = time.time() - start_time
    print("C shape: ",outputs[0].shape)
    print("Computation time: ", eval_time)


    print('Running 1024 instances')
    NINSTANCES = 1024
    work = torch.zeros((NINSTANCES, n_w), dtype=torch.double, device=device).contiguous()

    inputs = [torch.zeros((NINSTANCES, nnz_in[0]), dtype=torch.double, device=device).contiguous(),
                torch.randn((NINSTANCES, nnz_in[1]), dtype=torch.double, device=device).contiguous()]
    
    outputs = [torch.zeros((NINSTANCES, nnz_out[0])).contiguous()]

    start_time = time.time()
    check = func(outputs, inputs, work)
    torch.cuda.synchronize() 
    eval_time = time.time() - start_time
    print("C shape: ",outputs[0].shape)
    print("Computation time: ", eval_time)

    return outputs




if __name__ == "__main__":

    run_dense()
    exit()
    