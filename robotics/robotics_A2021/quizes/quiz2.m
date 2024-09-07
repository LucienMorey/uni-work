clear;
clc;
close all;

%% Baxter
qright =   [0,0,0,0,0,0,0];    
qleft = [0,0,0,0,0,0,0] ;

mdl_baxter();
right.plot(qright);
hold on;
left.plot(qleft);

% forward kinematics
efr = right.fkine(qright);
efl = left.fkine(qleft);

% distance between arms
d = sqrt( ( efr(1,4) - efl(1,4) )^2 + ( efr(2,4) - efl(2,4) )^2 + ( efr(3,4) - efl(3,4) )^2 );

% left base 2 right arm
lb = left.base;
d2 = sqrt( ( efr(1,4) - lb(1,4) )^2 + ( efr(2,4) - lb(2,4) )^2 + ( efr(3,4) - lb(3,4) )^2 );

% right base 2 left arm
rb = right.base;
d3 = sqrt( ( efl(1,4) - rb(1,4) )^2 + ( efl(2,4) - rb(2,4) )^2 + ( efl(3,4) - rb(3,4) )^2 );

% inv kinematics
qinv = right.ikine(efr);

%% Puma560
clear;
clc;
close all;

q = [0 0 0 0 0 0];
mdl_puma560

%% Point to Plane - perpendicular
clear;
clc;
close all;

% end effector point
P = [5 5 5];

% plane defined by point/normal method
Q = [6 9 8];
N = [3 2 1];

% A(x − xq)+B(y − yq)+C(z − zq)=0
D = -1 * (dot(Q, N));

% normal distance
d = abs(dot(P, N) + D) / norm(N)

%% Point to Plane Dist - along-Z
clear;
clc;
close all;

mdl_puma560();

% specify puma's joints (radians)
q = [0 0 pi/2 0 pi/3 0];
p560.plot(q);

% end effector pose
ef = p560.fkine(q);

% End effector point
P = [ef(1,4), ef(2, 4), ef(3, 4)];

% get z-axis
v = ef(1:3, 1:3) * [0; 0; 1];

% plane defined by point/normal method
Q = [0 0 -0.5];
N = [0 0 1];

% Calculate d for plane equation
D = -1 * (dot(Q, N));

% Get t, the line parameter
t = -1 * (dot(P, N) + D) / dot(N, v);

% Get the x,y,z components of vector from ef->surface along z-axis
xp = t * v(1);
yp = t * v(2);
zp = t * v(3);

% Get distance
d = sqrt( xp^2 + yp^2 + zp^2 )