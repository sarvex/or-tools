# math_opt

The code in this directory provides a generic way of accessing mathematical
optimization solvers (sometimes called mathematical programming solvers), such
as GLOP, CP-SAT, SCIP and Gurobi. In particular, a single API is provided to
make these solvers largely interoperable.

The goal of this code is to eventually replace `MPSolver`, as defined in
cs/ortools/linear_solver/linear_solver.h, and the associated
code.
