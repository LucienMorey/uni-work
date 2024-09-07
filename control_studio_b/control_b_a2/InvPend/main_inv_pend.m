% Assessment Task 5 
% Assignment 2: Optimal Control
% 
%
% 48580 Control Studio B
% University of Technology Sydney, Australia
% Autumn 2023
%
% Ricardo P. Aguilera
%
% Go Little Rockstar!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear;

%% Simulation Settings
SN = 12904090; %Insert here your student number

Tf = 60;    %Simulation length

Ts = 1/20;  %Sampling time for the controller

CL = 1;     % Enable closed-loop

obs = 1;    % 0: without  Observer, i.e., -F*x
            % 1: with Observer, i.e., -F*x_hat

noise = 1;  %Enable measurement output noise

controller = 1; %1: LQR
                %2:   MPC
                
N=15;            % Prediction Horizon (increase as required it)

animation = 0;  % 1: Animate the inverter pendulum

% make this non zero to use the specified matrices (q.1.3 and beyond)
provided_matrices = 1;
% make this non zero to saturate the lqr input
saturate = 0;

% multi sim
multi_sim = 0;

%% Input and State Constraints for MPC
% not required for LQR
% change these constraints as required
umin=-15;
umax=15;
theta_max=3*pi/180;            
theta_min=-theta_max;

xmin=[-10000;-10000;theta_min;-10000];    %Large number implies no constraint
xmax=[10000;10000;theta_max;10000];       %Large number implies no constraint

%% Initial Condition
x_o = -1;                    %cart position 
xsp_o = 0;                  %cart speed
th_o = 0;                   %0: pendulum vertically pointing upwards
                            %pi: pendulum vertically pointing downwards
w_o = 0;                    %angular speed of the pendulum

xo = [x_o xsp_o th_o w_o]'; %State initial condition
xo_hat=[0 0 0 0]';          %Observer initial condition

%% System Parameters
disp("System Parameters" + newline)
[Mc,mp,l,g,b] = MySysParam(SN)

disp("Press any key to continue"+ newline)
%pause;  

%% Continupus-Time LTI Model
% Please, all of you (especially Alberic)
% Modify the system matrices according to your derived model
n=4;    %Number of states
m=1;    %number of inputs
p=2;    %number of outputs

Ac=[0    1        0                    0
    0   -b/Mc     -mp*g/Mc            0
    0    0        0                    1
    0   b/(Mc*l)   g*(mp + Mc)/(Mc*l)   0];

Bc= [0 1/Mc 0 -1/(Mc*l)]';

Cc=[1 0 0 0;
   0  0 1 0];

%% Controllability
disp('*******************************')
CM=ctrb(Ac,Bc);   %Compute this accordingly
rank_CM = rank(CM);
if (rank_CM==n)
    disp('System is Controllable')
else
    disp('System is Not Controllable')
end

%% Discretization
sys_ct=ss(Ac,Bc,Cc,0);
sys_dt=c2d(sys_ct,Ts);

A=sys_dt.A;
B=sys_dt.B;
C=sys_dt.C;

%% LQR design
%tune your weighting matrices for your controller
Q=diag([22.0 2.0 20.0 8.1]);
R= 0.4;
if (provided_matrices > 0)
    R=0.1;
    Q=diag([5.0 0.0 1.0 0.0]);
end

disp("LQR Gain Matrix: " + newline)
[K,P]=dlqr(A,B,Q,R)

AK = A-B*K;
disp(newline + "Closed-loop eigenvalues (poles):")
eig_AK = eig(AK)'

%% Steady-Sate Kalman Filter design
disp(newline + "*******************************")
OM=obsv(A,C);   %Compute this accordingly
rank_OM=rank(OM);
if (rank_OM==n)
    disp('System is Observable')
end

%tune your weighting matrices for your Kalman Filter
Qf=diag([1e-4, 1e-08, 1e-20, 1e-08]);
Rf = diag([0.0290, 1.2900e-06]); %The diagonal of matrix Rf is the sensores covariance

if (provided_matrices > 0)
    Qf=0.0000001*eye(4,4);
end
[Pf,po_dt,Kf_t] = dare(A',C',Qf,Rf,[],[]);
%Pf: Lyapunonv matrix for KF
%po_dt: discrete-time observation eigenvalues
%Kf_t: Transpose of Kalman filter gain
Kf=Kf_t'

AL = A-Kf*C;

disp(newline + "Observer eigenvalues (poles): ")
eig_AL=eig(AL)'

disp(newline + "Press any key to continue" + newline)
%pause; 

%% MPC MATRIX GENERATION
gen_mpc_matrices;

%% Simulation
disp(newline + "*******************************")
%warning('off','all')
if (CL==0)
    disp("Simulating Open-Loop case..." + newline)
    sim('sim_2023_A2_inv_pend_OPT.slx')
    save('data_OL.mat');  
else
    if (controller==1)
        disp("Simulating LQR case..." + newline)
        sim('sim_2023_A2_inv_pend_OPT.slx');
        save('data_LQR.mat');  
        plot_LQR;
    end
    if (controller==2)
        disp(['MPC with N=',int2str(N)])
        %disp("Checking Matrices...")
        %stop=check_MPC_matrices(W,F,Lambda,AN,Umax,Umin,Xmax,Xmin,N,n,m);
        %if (stop==0)
        %    disp("Matrices have the right dimensions!!!")
            disp("Simulating MPC case..." + newline)
            sim('sim_2023_A2_inv_pend_OPT.slx');
            save('data_MPC.mat');
            plot_MPC;
        %end

        if multi_sim > 0
            multi_sim_data_xc = xc_hat(:,1);
            multi_sim_data_theta = theta_hat(:,1);
            for i = 1:6
                N = N+1;
                delete xc_hat;
                delete theta_hat;
                gen_mpc_matrices;
                fprintf("Simulating MPC case for N = %d...\n", N);
                sim('sim_2023_A2_inv_pend_OPT.slx');
                multi_sim_data_xc = horzcat(multi_sim_data_xc, xc_hat(:,1));
                multi_sim_data_theta = horzcat(multi_sim_data_theta, theta_hat(:,1));
                
            end
            plot_MPC_MULTI;
        end
    end
end
warning('on','all')
%% Create your own plots Here

%%
if (animation==1)
    
    disp('Press any key to continue with animation')
    pause;
    disp('Animating...') 
    Ndata = length(time);    
    figure(5)
    for k=1:500:Ndata
        animated_inv_pend_uw_clkw(y(k,1),y(k,2),mp,Mc,l);
    end
end
disp('Done!!!')

