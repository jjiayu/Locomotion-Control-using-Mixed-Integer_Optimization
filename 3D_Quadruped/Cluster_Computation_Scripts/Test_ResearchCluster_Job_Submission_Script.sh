#!/bin/sh

cd /disk/scratch/s1545529/Locomotion-Control-using-Mixed-Integer_Optimization/3D_Quadruped/TrajectoryComputation/

#use Eddie
export Eddie=1

#Define Casadi Path
export Casadi_Path=/disk/scratch_fast/s1545529/bin/casadi/matlab

#Specify gait if using fixed gait optimization
#export UserDefinedGaitNumber=1

#Speficy get Parameters from Files
export Paras_Define_Method=1

#Specify Computing Platform
export Remote_Computing_Flag=2

#*************Specify Parameter File Dir (Change based on Tasks and Parameter Setup)*******************
export ParamFileDir=/disk/scrach_fast/s1545529/Results/

#Specify Time Horizon Type
export Tend_flag=1

#Specify Time Horizon
export Tend=1

#Change speed specification
export ChangSpeedFlag=1

#Specify Speed Direction, 1-> Horizontal in World Frame, 2-> Horizontal along Terrain Frame
export SpeedDirection=1

#Specify Minimum Speed
export MinSpeed=0.3

#Specify Maximum Speed
export MaxSpeed=0.3

#Specify Speed Resolution
export SpeedResolution=0.2
 
# Run the program
matlab -nodesktop < Main_MINLP_and_FixedGait.m
