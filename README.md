# Locomotion Control using Mixed-Integer_Optimization

## Features:

- Mixed-inter Nonlinear Optimization Formulation
- Currently using Knitro to solve the mixed-integer optimization problem, using its Matlab interface
- Using centroidal robot model, currently the robot lives in 2D plane

## Trouble Shooting

1. Solver reports the optimization problem is infeasible

- Solver's problems, it accidentally identified all branches are infeasible which is wrong in most of the cases. Re-run the solver will solve the problem in most of the cases.
- Initial and Terminal robot height violates the kinematics constraint and complementarity constraint. If the robot height is set in a way that the upper level of the boudning box cannot reach the terrain height, then the problem is infeasible. Retune the initial and terminal robot height to solve the problem.
- The (go-to-goal) task is infeasible for the current foot-ground reaction force limits, foot swing velocity, kinematics contraint. Either change the task or tune the parameters for big-M formulation (goerns foot swing velocity and foot-ground reaction forces limits), kinematics contraint will solve the problem. 


