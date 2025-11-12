import casadi as ca
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver

# Load functions from files .casadi
casadi_func = ca.Function.load('funcx.casadi')

# Define state and control action
x = ca.SX.sym('x', 1)
u = ca.SX.sym('u', 0)  # no control in this case

# Create acados model
model = AcadosOcp.Model()
model.x = x
model.u = u
model.f_impl_expr = ca.SX.zeros(1)  # null dynamic static problem
model.name = 'RRR_test'

# Crete OCP
ocp = AcadosOcp()
ocp.model = model
ocp.dims.N = 1  # static problem, one step
ocp.solver_options.tf = 1.0

# Use the external function as the cost
ocp.cost.cost_type = 'EXTERNAL'  # external cost
ocp.cost.external_cost_expr = casadi_func(x)
ocp.cost.external_cost_expr_0 = casadi_func(x)
ocp.cost.W = np.array([[1.0]])  # weight

# Initial state
ocp.constraints.x0 = np.array([2.0])

# Generate solver
ocp_solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')
ocp_solver.generate()  # generate C-code

print("Generation complete!")
