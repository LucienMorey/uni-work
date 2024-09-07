    %% SetUP
close all;
clear;

% Pose Constants
UR3BasePose = transl(0.3,0,0);
UR5BasePose = transl(-0.6,-0.4,0);
brickPositions = [ ...
    0.3,0.2;
    -0.3,0;
    0,0.3;
    0,-0.1;
    0,0.5;
    0,-0.5;
    0.3,-0.1;
    -0.3,-0.3;
    0.3,-0.3;
    ];

wallPose = transl(0,0,0)*trotz(pi/2);
wallSize = [3,3]; 

% Create Sim
Sim = RoboticsSimulator(UR3BasePose, UR5BasePose, brickPositions, wallPose, wallSize);

% %% RANGE DISPLAY
% disp('displaying volume workspace');
% Sim.DisplayVolumeWorkSpaces(true);
% pause
% Sim.DisplayVolumeWorkSpaces(false);
% disp('displaying reach nets');
% Sim.DisplayReachNets(true);
% pause
% Sim.DisplayReachNets(false);
% 
% %% RUN SIM
% % Build Wall
% disp('beginning placement');
% Sim.BuildWall();