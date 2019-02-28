# Locomotion Control using Mixed-Integer_Optimization

## Features:

- Mixed-inter Nonlinear Optimization Formulation
- Currently using Knitro to solve the mixed-integer optimization problem, using its Matlab interface
- Using centroidal robot model, currently the robot lives in 2D plane

## Notes

- Control Input has the same length with states --> u_1:N and x_1:N, but dynamics constraint only acts from time step 0 to N-1 (in other words, only u_1:N-1 are useful). But currently u_N is included in the cost function which has no effects of the final result. u_N will be useful when higher order quadrature method is used (trapzoidal), then we need to check all indice in cost functions and constraints (ALL! dynamic constraint, kinematics constraint, complementarity constraint...,check all constraint)

- For phase-based formulation, particularly the formulation that each phase has same number of time steps, the dynamics, kinematics and complementarity constraints are not enforced on the last time step (tf), because we are using Euler Integration that the control and foot step location of last time step has no effects on the state trajectory.


## Trouble Shooting

1. Solver reports the optimization problem is infeasible

- Solver's problems, it accidentally identified all branches are infeasible which is wrong in most of the cases. Re-run the solver will solve the problem in most of the cases.
- Initial and Terminal robot height violates the kinematics constraint and complementarity constraint. If the robot height is set in a way that the upper level of the boudning box cannot reach the terrain height, then the problem is infeasible. Retune the initial and terminal robot height to solve the problem.
- The (go-to-goal) task is infeasible for the current foot-ground reaction force limits, foot swing velocity, kinematics contraint. Either change the task or tune the parameters for big-M formulation (goerns foot swing velocity and foot-ground reaction forces limits), kinematics contraint will solve the problem. 

## Future Work

- Re-program the containts --> Build a big for loop whose index is based on TimeSeries

- Object-Oriented Porgramming Method --> Just add limbs into robot class

- For the kienmatics constraint, we can use tensor-based formualtion to speed up computation time and simplfy the program -->Ask Chris and Theo



