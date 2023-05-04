%% EXAMPLE: Differential drive vehicle following waypoints using the 
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:280;         % Time array

initPose = [7;0;pi];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
waypoints = [7,0; 6,0; 5.5,1; 5,2; 5,3; 5,4; 5,5; 4,6; 3,6; 2,6; 1,6; 0.5,7; 0,8; 1,9; 0,9; 0,8; 1,9; 2,9; 2,10; 3,10; 4,10; 3,9; 3,10; 3,10.3; 4,10.6; 4,12; 5,11; 4,10.6; 5,11; 6,11; 6,10; 6,11; 6,12; 6.3,11; 6.8,10; 7,9; 6,8; 7,9; 6.8,10; 6.3,11; 6,12; 6.5,11; 7,10; 7.5,9; 8,8; 9,7; 8,6.5; 7,6; 6,5.5; 5,5; 5,4; 6,4.4; 7,4.9; 7.5,5.1; 8,5.3; 9,5.7; 10,6; 9,7; 10,6; 11,6; 11,5; 11,4; 11,3; 11,2; 11,1; 11,0; 10,0; 9,0; 9,1; 9,0; 8,0; 7,0; 6,0]
%% 

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.10;
controller.DesiredLinearVelocity = 0.45;
controller.MaxAngularVelocity = 20.5;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    waitfor(r);
end