% Assessment Task 5 
% Assignment 1: State Feedback Control
% 
%
% 48580 Control Studio B
% University of Technology Sydney, Australia
% Autumn 2023
%
% A/Prof Ricardo P. Aguilera
%
% ***Check the script for possible Easter Eggs!!!
%
% Enjoy it!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;

%% Simulink file
simu = 'A1_Inverter_sim';

%% Simulation Settings
CL = 1;     % 1: Enable closed-loop using SFC
obs = 1;    % 0: SFC without observer, i.e., u = -Fx +r
            % 1: SFC with observer, i.e., u = -Fx_hat + r
noise = 1;  % add noise to measurement

%% Times and Frequencies
Tsim = 80e-3;   %Total simulation time
fc = 10000;     %PWM Carrier frequency
Tc = 1/fc;      %PWM Carrier period
fs = 2*fc;      %Controller sampling frequency
Ts = 1/fs;      %Controller sampling time
Tstep = Ts/100; %Simulation time step

%% System Parameters
l=2.5e-3;         %Inductance
Co=5e-6;       %Output capacitance     
Ro = 0.1;       %Output resistance
w=2*pi*50;      %Output frequency
Vdc=100;        %DC-voltage

%% Continuous time model
Ac = [0 -Ro/l;
      1/(Co*Ro) -1/(Co*Ro)];
Bc = [Vdc/l 0]';
Cc = [0 Ro];
Dc = 0;

sys_c = ss(Ac,Bc,Cc,Dc);

%% discrete time model
sys_d = c2d(sys_c, Ts);
A=sys_d.A;
B=sys_d.B;
C=sys_d.C;
D=sys_d.D;

%% Controller Pole Placement 
% implement the pole placement technique here to obtain the 
% state feedback gain matrix F
zeta_c = 0.85;
wn_c = 2*pi*4000;

s1_c = -zeta_c *wn_c + wn_c*sqrt(zeta_c^2 -1);
s2_c = -zeta_c *wn_c - wn_c*sqrt(zeta_c^2 -1);
sfc_poles_c = [s1_c,s2_c];

sfc_poles = exp(sfc_poles_c*Ts);

F= place(A,B,sfc_poles);

%% Observer Pole Placement 
% implement the pole placement technique here to obtain the 
% observer matrix L
zeta_o = 0.85;
wn_o = 2*pi*20000;

obs_s1_c = -zeta_o *wn_o + wn_o*sqrt(zeta_o^2 -1);
obs_s2_c = -zeta_o *wn_o - wn_o*sqrt(zeta_o^2 -1);
obs_poles_c = [obs_s1_c,obs_s2_c];
obs_poles = exp(obs_poles_c*Ts);

L = place(A',C',obs_poles)';

sim(simu)

%% Plots
A1_plots; %% Check the A1_plots.m file for plots


