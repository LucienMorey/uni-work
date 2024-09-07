%% Question 1
A = [-3 0; 0 1];
B = [0; 1];
C = [1,0];

% a) determine the eigen values of matrix a
% Note System not stable because one of the eigen values has a positive
% real component
E = eig(A);

c_m = ctrb(A,B);
rank_c = rank(c_m);
system = ss(A,B,C,0);

%% Question 2
m = 1000;
k = 50;
c = 20;

A = [0,1; -k/m, -c/m];
B = [0;1/m];
C = [1,0];
D = 0;

system_spring_mass = ss(A,B,C,D);

step(system_spring_mass);

% t = 0:0.1:300;
% u = 50*ones(size(t));
% 
% x0 = [10;-3];
% 
% [Y,T,X] = lsim(system_spring_mass, u, t, x0);

P1 = -0.1;
P2 = -0.3;

K = place(A,B,[P1,P2]);
A_dash = A-B*K;

closed_loop_spring_mass = ss(A_dash, B,C,D);
step(closed_loop_spring_mass/0.33);
dcgain(closed_loop_spring_mass)



%% Question 3