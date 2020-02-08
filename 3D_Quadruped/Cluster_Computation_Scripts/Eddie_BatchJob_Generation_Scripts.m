clear;
clc;

WorkingDir = uigetdir();

disp(['Working Directory: ', WorkingDir]);

%if exist([WorkingDir,'/EddieJobScript'],'dir') == 0
%    mkdir([WorkingDir,'/EddieJobScript'])
%end

%Specify Task Requirement
%   Stride Period
MinStridePeriod = input('Specify the Minimum Stride Period (i.e. 0.6): \n');
MaxStridePeriod = input('Specify the Maximum Stride Period (i.e. 1.6): \n');
StridePeriodResolution = input('Specify the Stride Period Resolution (i.e. 0.2): \n');
StridePeriodVector = MinStridePeriod:StridePeriodResolution:MaxStridePeriod;
disp(' ')

%   Speed Range
MinSpeed = input('Specify the Minimum Speed (i.e. 0.3): \n');
MaxSpeed = input('Specify the Maximum Speed (i.e. 3.5): \n');
SpeedResolution = input('Specify the Speed Resolution (i.e. 0.2): \n');
SpeedVector = MinSpeed:SpeedResolution:MaxSpeed;
disp(' ')

%Specify Number of Phase
PhaseNum = input('Specify Number of Phase(i.e. 2, 4): \n');

%Specify Other Parameters
PhaselbPercent=input('Specify Phase Lower Bound Duration, in Percentage (i.e. 20, do not type percentage symbol): \n');

BoundingBoxType=input('Specify BoundingBox Type: \n1-> Slim  2->Wide: \n');
if BoundingBoxType==1
   BoundingBoxTypeName = "Slim"; 
elseif BoundingBoxType==2
    BoundingBoxTypeName = "Wide";
else
    error('Unknown BoundingBox Type');
end

TerrainSlope = input('Specify Terrain Slope (i.e. 0): \n');

CostType = input('Specify Cost Type: \n1-> MiniForces \n2-> MiniBodyVib_Forces_LateralLimbDisplacement \n3-> MiniBodyVib \n4-> SmoothForceProfile \n5-> MiniBodyVib_LateralLimbDisplacement \n');

if CostType==1
    costname = "MiniForces";
elseif CostType==2
    costname = "MiniBodyVib_Forces_LateralLimbDisplacement";
elseif CostType==3
    costname = "MiniBodyVib";
elseif CostType==4
    costname = "SmoothForceProfile";
elseif CostType==5
    costname = "MiniBodyVib_LateralLimbDisplacement";
else
    error('Unknown Cost Label')
end

disp(' ')

NumberofBatches = input('Specify Number of Batches: \n');
disp(' ')

NumofRuns = input('Specify Number of Runs for Each Batch: \n');
disp(' ')

%Loop over all the options
for strideperiod_index = 1:length(StridePeriodVector)
    strideperiod = StridePeriodVector(strideperiod_index);
    
    
    if exist([WorkingDir,'/StridePeriod_',num2str(strideperiod),'/EddieJobScript'],'dir') == 0
        mkdir([WorkingDir,'/StridePeriod_',num2str(strideperiod),'/EddieJobScript']);
    end
    
    %Writing different script for different speed
    for speed_index = 1:length(SpeedVector)  
        
        speed = SpeedVector(speed_index);
        fid = fopen([WorkingDir,'/StridePeriod_',num2str(strideperiod),'/EddieJobScript','/Speed_',num2str(speed),'.sh'],'wt');

        %Parameter Set up
        fprintf(fid, '#!/bin/sh\n');
        fprintf(fid, '# Grid Engine options (lines prefixed with #$)\n');
        fprintf(fid, strcat('#$ -N C',num2str(CostType),'T',num2str(strideperiod),'S',num2str(speed),'\n'));
        fprintf(fid, '#$ -cwd \n');
        fprintf(fid, '#$ -l h_rt=72:00:00\n');
        fprintf(fid, '#$ -l h_vmem=8G\n');
        fprintf(fid, '#$ -V\n');
        fprintf(fid, '#  These options are:\n');
        fprintf(fid, '#  job name: -N\n');
        fprintf(fid, '#  use the current working directory: -cwd\n');
        fprintf(fid, '#  runtime limit of 5 minutes: -l h_rt\n');
        fprintf(fid, '#  memory limit of 1 Gbyte: -l h_vmem\n');
        fprintf(fid, '\n');
        
        fprintf(fid, '# Initialise the environment modules\n');
        fprintf(fid, '. /etc/profile.d/modules.sh\n');
        fprintf(fid, '\n');
        
        fprintf(fid, '# Load Matlab\n');
        fprintf(fid, 'module load matlab/R2018a\n');
        fprintf(fid, '\n');
        
        fprintf(fid, 'cd /home/s1545529/Locomotion-Control-using-Mixed-Integer_Optimization/3D_Quadruped/TrajectoryComputation/\n');
        fprintf(fid, '\n');

        fprintf(fid, '#use Eddie\n');
        fprintf(fid, 'export Eddie=1\n');
        fprintf(fid, '\n');
        
        fprintf(fid, '#Define Casadi Path\n');
        fprintf(fid, 'export Casadi_Path=/home/s1545529/bin/casadi/matlab/\n');
        fprintf(fid, '\n');
        
        fprintf(fid, '#Specify gait if using fixed gait optimization\n');
        fprintf(fid, '#export UserDefinedGaitNumber=1\n');
        fprintf(fid, '\n');
        
        fprintf(fid, '#Speficy get Parameters from Files\n');
        fprintf(fid, 'export Paras_Define_Method=1\n');
        fprintf(fid, '\n');
        
        fprintf(fid, '#Specify Computing Platform\n');
        fprintf(fid, 'export Remote_Computing_Flag=2\n');
        fprintf(fid, '\n');
        
        fprintf(fid, '#*************Specify Parameter File Dir (Change based on Tasks and Parameter Setup)*******************\n');
        fprintf(fid, strcat('export ParamFileDir=/home/s1545529/Eddie_3D_ANYmal_Gait_Dsicovery/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'\n'));
        fprintf(fid, '\n');
        
        fprintf(fid, '#Specify Time Horizon Type\n');
        fprintf(fid, 'export Tend_flag=1\n');
        fprintf(fid, '\n');
        
        fprintf(fid, '#****************Specify Time Horizon************(Change based on Task and Parameter Setup)\n');
        fprintf(fid, strcat('export Tend=',num2str(strideperiod),'\n'));
        fprintf(fid, '\n');
        
        fprintf(fid, '#Change speed specification\n');
        fprintf(fid, 'export ChangSpeedFlag=1\n');
        fprintf(fid, '\n');
        
        fprintf(fid, '#Specify Speed Direction, 1-> Horizontal in World Frame, 2-> Horizontal along Terrain Frame\n');
        fprintf(fid, 'export SpeedDirection=1\n');
        fprintf(fid, '\n');

        fprintf(fid, '#*****************Specify Minimum Speed**********(Change based on Task and Parameter Setup)\n');
        fprintf(fid, strcat('export MinSpeed=',num2str(speed),'\n'));
        fprintf(fid, '\n');

        fprintf(fid, '#*****************Specify Maximum Speed**********(Change based on Task and Parameter Setup)\n');
        fprintf(fid, strcat('export MaxSpeed=',num2str(speed),'\n'));
        fprintf(fid, '\n');

        fprintf(fid, '#Specify Speed Resolution\n');
        fprintf(fid, 'export SpeedResolution=0.2\n');
        fprintf(fid, '\n');
        
        fprintf(fid, '#Specify Number of Runs per Batch\n');
        fprintf(fid, strcat('export NumofRuns=',num2str(NumofRuns),'\n'));
        fprintf(fid, '\n');

        fprintf(fid, '# Run the program\n');

        fprintf(fid, 'matlab -nodesktop < Main_MINLP_and_FixedGait.m\n');
        fprintf(fid, '\n');

        fclose(fid);
    end
     
    %make a full batch script
    fid = fopen([WorkingDir,'/StridePeriod_',num2str(strideperiod),'/EddieJobScript','/FullBatchScript','.sh'],'wt');
    for speed_index = 1:length(SpeedVector) 
        speed = SpeedVector(speed_index);
        for batch_count=1:NumberofBatches
            fprintf(fid, strcat('qsub Speed_',num2str(speed),'.sh','\n'));
            fprintf(fid, '\n');
            
            fprintf(fid, 'sleep 10\n');
            fprintf(fid, '\n');
        end
    end
    fclose(fid);
    
end