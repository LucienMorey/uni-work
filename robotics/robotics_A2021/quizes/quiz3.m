%% Collision Checking
close all;
clear;

%specify these
% plane definition
normal = [1, 0, 0];
point = [6, 0, 0];
%robot angles
q = [-pi/8,0,0]; 

%create robot
mdl_3link3d;

for i = 0:1:R3.n -1
    %determine robot link line
    robot_link_start = R3.base;
    if i > 0
        for j=1:1:i
          robot_link_start = robot_link_start * trotz(q(j)+R3.offset(j)) * transl(0, 0, R3.d(j)) * transl(R3.a(j),0,0) * trotx(R3.alpha(j));
        end
    end

    robot_link_end = R3.base;
    for j=1:1:i+1
      robot_link_end = robot_link_end * trotz(q(j)+R3.offset(j)) * transl(0, 0, R3.d(j)) * transl(R3.a(j),0,0) * trotx(R3.alpha(j));
    end
    %determine intersection of line/plane from last robot link and specified
    %plane
    line_start_point = robot_link_start(1:3,4)';
    line_end_point = robot_link_end(1:3,4)';

    [intersectionPoint, check] = LinePlaneIntersection(normal, point, line_start_point, line_end_point);

    if (check ==1) || (check == 2)
        % display result
        disp(['intersection between joint ', num2str(i), ' ', num2str(i+1), ' of type ', num2str(check), ' at ', mat2str(intersectionPoint)]);
    end

end

%% 5DOF Planar Robot
close all;
clear;

% specify these
q = deg2rad([30,-60,45,-30,0]);

% Create robot
L1 = Link('d', 0, 'a', 1, 'alpha', 0, 'qlim', [-pi,pi]);
L2 = Link('d', 0, 'a', 1, 'alpha', 0, 'qlim', [-pi,pi]);
L3 = Link('d', 0, 'a', 1, 'alpha', 0, 'qlim', [-pi,pi]);
L4 = Link('d', 0, 'a', 1, 'alpha', 0, 'qlim', [-pi,pi]);
L5 = Link('d', 0, 'a', 1, 'alpha', 0, 'qlim', [-pi,pi]);
robot = SerialLink([L1, L2, L3, L4, L5]);

% calculate end effector pose
pose = robot.fkine(q);

% display position
disp(pose(1:3, 4)');

%% Puma Distance Sense
close all;
clear;

% specifiy these
q = deg2rad([0, 45, -80, 0, 45, 0]);
sensor_pose = transl(1, -1.562, 1);

% create puma
mdl_puma560;
distToSensor
% calculate displacement to sensor
robot_pose = p560.fkine(q);
dist = -sqrt(distToSensor^2 - (1 - robot_pose(3, 4))^2 - (sensor_pose(1, 4) - robot_pose(1, 4))^2) + robot_pose(2, 4)

% display result
% disp(distance_to_sensor);

%% Puma Distance Sense checker
close all;
clear;

% specifiy these
q = deg2rad([0, 45, -80, 0, 45, 0]);
sensor_pose = transl(-0.2165, -0.1500, 1.0620);

% create puma
mdl_puma560;

% calculate displacement to sensor
robot_pose = p560.fkine(q);
distance_to_sensor = sqrt(sum(sum((sensor_pose(1:3, 4) - robot_pose(1:3, 4)).^2)));

% display result
disp(distance_to_sensor);
%% Point in Puma560 Coordinate Frame
clear;clc;close all;

%specify these
ball = transl(0.5,0.1,0.6) * trotx(pi/2);
q = deg2rad( [90, 30, -80, 0, 45, 0] );

%create robot
mdl_puma560;

%determine end effector pose
ef = p560.fkine(q);

%get relative transform to ball
ef2ball = ef \ ball;

%display distance
xzy = ef2ball(1:3, 4)
%% Puma Ikine
clear;clc;close all;

%specify these
t = transl(0.7,0.1,0.2);

%create robot
mdl_puma560;

%calculate end effector pose. orientation is masked off
q = p560.ikine(t, qn, [1 1 1 0 0 0]);
p560.plot(q);

%display result
disp(q);
%% Puma intersection along Z
close all;
clear;
clc;

% plane definition
normal = [-1, 0, 0];
point = [1.8, 0, 0];

%robot joint angles
q = [pi/12,0,-pi/2,0,0,0];

%create robot
mdl_puma560;

%determine end effector pose
ee = p560.fkine(q);

v = ee(1:3, 1:3) * [0; 0; 1];

ee2 = ee;
ee2(1:3, 4) = ee2(1:3, 4) + v * 0.25;

p1 = ee(1:3, 4)';
p2 = ee2(1:3, 4)';

% determine intersection of line/plane from last robot link and specified
%plane
[intersectionPoint,check] = LinePlaneIntersection(normal, point, p1, p2);

disp(intersectionPoint);
% 0 - no intersect
% 1 - line plane intersect
% 2 - parallel coincident
% 3 - intersect outside of segment

%% Safety discussion

%% Sawyer discussion

%% Assignment 1 Trajectories

s = lspb(0,1,steps);
% First, create the scalar function
qMatrix = nan(steps,6);
% Create memory allocation for variables
for i = 1:steps
    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
    % Generate interpolated joint angles
end
velocity = zeros(steps,6);
acceleration  = zeros(steps,6);
for i = 2:steps
    velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);
    % Evaluate relative joint velocity
    acceleration(i,:) = velocity(i,:) - velocity(i-1,:);
    % Evaluate relative acceleration
end