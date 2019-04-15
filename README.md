# Locomotion Control using Mixed-Integer_Optimization

## Features:

- Mixed-inter Nonlinear Optimization Formulation
- Currently using Knitro to solve the mixed-integer optimization problem, using its Matlab interface
- Using centroidal robot model, currently the robot lives in 2D plane

## Notes

- Control Input has the same length with states --> u_1:N and x_1:N, but dynamics constraint only acts from time step 0 to N-1 (in other words, only u_1:N-1 are useful). But currently u_N is included in the cost function which has no effects of the final result. u_N will be useful when higher order quadrature method is used (trapzoidal), then we need to check all indice in cost functions and constraints (ALL! dynamic constraint, kinematics constraint, complementarity constraint...,check all constraint)

- For phase-based formulation, particularly the formulation that each phase has same number of time steps, the dynamics, kinematics and complementarity constraints are not enforced on the last time step (tf), because we are using Euler Integration that the control and foot step location of last time step has no effects on the state trajectory.

- Friction Cone formulation is different in 2D and 3D case. 2D case is a linear friction cone where 3D case is nonlinear.


## Knitro and CasADi

1. Knitro and CaSADi only works well in Linux, with CasADi Matlab interface:
    - In Mac, it is difficult to identify the location of dynamic libraries, due to difficulties to set DYLD_LIBRARY_DIR environemtn variable.
    - In Windows, CasADi team does not ship interfaces for Knitro. Compiling CasADi in Windows is difficult and not recommended.
    - Due to unknown reasons, Python interface for Knitro always got "Segmentation Fault", no matter pre-built binaries or compiled from source

2. To make CasADi and Knitro running in Matlab in Linux, we need to:
    - Firstly, Put Path of "knitroampl" and "lib" (in Knitro installation folder) into environment variable "PATH" and "LD_LIBRARY_PATH"
    - Secondly, Copy the "libknitroXXXX.so"/"libknitro.so" in the "lib" folder of the Knitro installation path and rename to "libknitro1030.so", since binary installation works with "libknitro1030.so". Alternatively, we can re-compile the interface from source (Not recommended now, but indeed the re-compiled interface gets slightly faster computation speed).
    - Thirdly, open Matlab from command line to ensure we load all the environment variable we have defined and use "addpath" function to add the "casadi" installation folder.

3. Pending Issues

    - CasAdi Python interface
        - Always show "Segmentation Fault", no matter call Knitro from either pre-built binary CasADi Python interface or the interfaces compiled from source. 
        - **May consider to ask for the solution in the forum or github, when matlab interface is not good enough.**
    
    - Knitro Dynamic Library File Name/Version
        - Currently, due to unknown reasons, we need to rename the Knitro dynamic library "libknitroXXXX.so"/"libknitro.so" to "libknitro1030.so".
        - Recompile CasADi from source can solve the problem.
        - **May need to install Knitro 1030 to adapt to the CasADi interface**
        - **Ask solution from google forum or Github if something unexpected happened.**

    - Computing Speed Issue
        - Cross computer: Thinkpad P51 is slower than Henrique's Desktop --> Not Acceptable
        - Cross OS: Windows is faster than Ubuntu 18, when using Bonmin to solver the example problem

## Trouble Shooting

1. Solver reports the optimization problem is infeasible

    - Solver's problems, it accidentally identified all branches are infeasible which is wrong in most of the cases. Re-run the solver will solve the problem in most of the cases.
    - Initial and Terminal robot height violates the kinematics constraint and complementarity constraint. If the robot height is set in a way that the upper level of the   boudning box cannot reach the terrain height, then the problem is infeasible. Retune the initial and terminal robot height to solve the problem.
    - The (go-to-goal) task is infeasible for the current foot-ground reaction force limits, foot swing velocity, kinematics contraint. Either change the task or tune the parameters for big-M formulation (goerns foot swing velocity and foot-ground reaction forces limits), kinematics contraint will solve the problem. 

## Future Work

    - Object-Oriented Porgramming Method --> Just add limbs into robot class

    - Make a function to generate variable name lists, and another function to check the if the varibales name list match with the CasADi decision variable names


