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

initPose = [4;1;pi];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
waypoints = [4,1; 3,1; 2,1; 1,2; 0,3; 1,3; 2,3; 3,2; 4.1,1; 5,1; 6,1; 7,2; 8,3; 7,3; 6,3; 5,2; 4,1; 4,2; 4,3; 4,4; 4,5; 3,4; 2,3; 2,4; 2,5; 1,5; 0,5; 1,6; 2,7; 3,7; 3,8; 4,8; 5,8; 5,7; 5,6; 4,6; 3,6; 3,7; 2,7; 1,8; 0,9; 1,9; 2,9; 2,10; 2,11; 3,10; 4,9; 5,10; 6,11; 6,10; 6,9; 7,9; 8,9; 7,8; 6,7; 7,6; 8,5;7,5; 6,5; 6,4; 6,3; 5,4; 4,5]

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.15;
controller.DesiredLinearVelocity = 0.25;
controller.MaxAngularVelocity = 18.5;

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