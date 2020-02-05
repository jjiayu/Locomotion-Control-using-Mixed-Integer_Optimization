clear;
clc;

WorkingDir = uigetdir();

disp(['Working Directory: ', WorkingDir]);

%if exist([WorkingDir,'/BatchJobScript'],'dir') == 0
%    mkdir([WorkingDir,'/BatchJobScript'])
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

NumberofBatches = input('Specify Number of Batches (Note:Check Parameters.m to get how many runs of a batch we have, 10): \n');

%Loop over all the options
for strideperiod_index = 1:length(StridePeriodVector)
    strideperiod = StridePeriodVector(strideperiod_index);
    
    
    if exist([WorkingDir,'/StridePeriod_',num2str(strideperiod),'/BatchJobScript'],'dir') == 0
        mkdir([WorkingDir,'/StridePeriod_',num2str(strideperiod),'/BatchJobScript']);
    end
    
    %Writing different script for different speed
    for speed_index = 1:length(SpeedVector)  
        
        for batch_count=1:NumberofBatches
            speed = SpeedVector(speed_index);
            fid = fopen([WorkingDir,'/StridePeriod_',num2str(strideperiod),'/BatchJobScript','/Speed_',num2str(speed),'_Batch_',num2str(batch_count),'.sh'],'wt');

            fprintf(fid, '#!/bin/sh\n');

            fprintf(fid, '\n');

            fprintf(fid, '#Make working folder\n');
            fprintf(fid, strcat('mkdir -p /disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/code','\n'));
            fprintf(fid, strcat('mkdir -p /disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/results','\n'));
            %fprintf(fid, strcat('mkdir -p /disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/software','\n'));
            fprintf(fid, '\n');
            
            %fprintf(fid, '#Copy and Unzip Software\n');
            %fprintf(fid, strcat('cp ~/software_stack.zip /disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/software','\n'));
            %fprintf(fid, strcat('cd /disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/software','\n'));
            %fprintf(fid, 'unzip software_stack.zip\n');
            
            %fprintf(fid, '#Add Path\n');
            %fprintf(fid, strcat('export KNITRODIR=/disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/software/bin/knitro-11.1.2-z-Linux-64','\n'));
            %fprintf(fid, strcat('export PATH=/disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/software/bin/knitro-11.1.2-z-Linux-64/knitroampl:$PATH','\n'));
            %fprintf(fid, strcat('export LD_LIBRARY_PATH=/disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/software/bin/knitro-11.1.2-z-Linux-64/lib:$LD_LIBRARY_PATH','\n'));
            %fprintf(fid, strcat('export LD_LIBRARY_PATH=/disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/software/bin/pcre/lib:$LD_LIBRARY_PATH','\n'));
            %fprintf(fid, strcat('export PATH=/disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/software/bin:$PATH','\n'));
            %fprintf(fid, strcat('export PATH=/disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/software/bin/swig:$PATH','\n'));            
            %fprintf(fid, strcat('export PATH=/disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/software/git/swig:$PATH','\n'));

            %Copy code and parameters
            fprintf(fid, '#Copy code and parameters\n');
            fprintf(fid, strcat('cp -r ~/Locomotion-Control-using-Mixed-Integer_Optimization/3D_Quadruped/TrajectoryComputation/',' /disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/code','\n'));
            fprintf(fid, strcat('cp -r ~/Cluster3D_ANYmal_Gait_Discovery_ExpScripts/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','Parameters.m',' /disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/results','\n'));
            fprintf(fid, '\n');
            
            %Setup Computation
            fprintf(fid, '#use Eddie\n');
            fprintf(fid, 'export Eddie=1\n');
            fprintf(fid, '\n');
            
            fprintf(fid, '#Define Casadi Path\n');
            fprintf(fid, '\n');
            %fprintf(fid, strcat('export Casadi_Path=/disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/software/bin/casadi/matlab','\n'));
            %fprintf(fid, strcat('export Casadi_Path=/disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/software/bin/casadi/lib','\n'));
            fprintf(fid, 'export Casadi_Path=/disk/scratch_fast/s1545529/bin/casadi/matlab\n');
            
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
            fprintf(fid, strcat('export ParamFileDir=/disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/results','\n'));
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
            
            fprintf(fid, '# Run the program\n');
            
            fprintf(fid, strcat('cd /disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/code','/TrajectoryComputation/','\n'));
            fprintf(fid, '\n');
            
            fprintf(fid, 'matlab -nodesktop < Main_MINLP_and_FixedGait.m\n');
            fprintf(fid, '\n');
            
            %Copy Results
            fprintf(fid, '# Copy Results\n');
            fprintf(fid, strcat('mkdir -p ~/Cluster3D_ANYmal_Gait_Discovery_ExpScripts/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','batch_',num2str(batch_count),'_results','\n'));                        
            fprintf(fid, '\n');
            
            fprintf(fid, strcat('cp -r /disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/results/*.*', ' ~/Cluster3D_ANYmal_Gait_Discovery_ExpScripts/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','batch_',num2str(batch_count),'_results','\n'));
            fprintf(fid, '\n');
            
            fprintf(fid, strcat('cp -r /disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'/results/*', ' ~/Cluster3D_ANYmal_Gait_Discovery_ExpScripts/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','batch_',num2str(batch_count),'_results','\n'));
            fprintf(fid, '\n');
            
            %due to the code, there is one more strid period folder in the
            %temporary result folder
            fprintf(fid, strcat('cd ~/Cluster3D_ANYmal_Gait_Discovery_ExpScripts/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','batch_',num2str(batch_count),'_results','/','StridePeriod_',num2str(strideperiod),'\n'));
            fprintf(fid, '\n');
            %Therefore move double time
            fprintf(fid, 'mv * ../..\n');
            fprintf(fid, '\n');
            %fprintf(fid, 'mv * ..\n');
            
            fprintf(fid, strcat('rm -rf ~/Cluster3D_ANYmal_Gait_Discovery_ExpScripts/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','batch_',num2str(batch_count),'_results','\n'));
            fprintf(fid, '\n');

            fprintf(fid, strcat('rm -rf /disk/scratch/s1545529/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent','/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','Speed_',num2str(speed),'/batch_',num2str(batch_count),'\n'));
            fprintf(fid, '\n');
            
            %Also remove script
            fprintf(fid, '# Remove Job Script\n');
            fprintf(fid, strcat('rm -rf ~/Cluster3D_ANYmal_Gait_Discovery_ExpScripts/',num2str(PhaseNum),'Phases','/','Phaselb_',num2str(PhaselbPercent),'_Percent/',BoundingBoxTypeName,'_KineBox','/',num2str(TerrainSlope),'_Degrees','/',num2str(CostType),'_',costname,'/','StridePeriod_',num2str(strideperiod),'/','BatchJobScript','/Speed_',num2str(speed),'_Batch_',num2str(batch_count),'.sh','\n'));
            
            fclose(fid);
        end
        
    end
     
    %make a full batch script
    fid = fopen([WorkingDir,'/StridePeriod_',num2str(strideperiod),'/BatchJobScript','/FullBatchScript','.sh'],'wt');
    for speed_index = 1:length(SpeedVector) 
        speed = SpeedVector(speed_index);
        for batch_count=1:NumberofBatches
            fprintf(fid, strcat('sbatch -p PGR-Standard Speed_',num2str(speed),'_Batch_',num2str(batch_count),'.sh','\n'));
            fprintf(fid, '\n');
            
            fprintf(fid, 'sleep 10\n');
            fprintf(fid, '\n');
            
        end
    end
    fclose(fid);
    
end