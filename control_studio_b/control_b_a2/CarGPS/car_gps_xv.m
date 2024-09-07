% Assessment Task 5 
% Assignment 2: Optimal Filtering
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

clc;
clear;
%%
fgps=10; %GPS data per second
Ts=1/fgps;

m=1000;     %mass
b=50;       %friction

xo_hat=[2 0 0 2 0 0]';  

%% Discrete time model
A = [1, Ts, 0.5*Ts^2, 0, 0,  0;
     0, 1,  Ts,       0, 0,  0;
     0, 0,  1,        0, 0,  0;
     0, 0,  0,        1, Ts, 0.5*Ts^2;
     0, 0,  0,       0, 1,  Ts;
     0, 0,  0,        0, 0,  1];

C = [1, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0];

%% Steady-State Kalman Filter Design
Qf = diag([0.5e-1, 1e-9, 1e-5, 0.5e-1, 1e-9, 1e-5]);

Rf = diag([220.9372, 0.1160, 769.0212, 0.4752]);


[P,po_dt,Kf_t] = dare(A',C',Qf,Rf,[],[]);
Kf=Kf_t'

%% simulation
disp('Simulating...')
sim('sim_car_gps_xv.slx')


%% plot
disp('Plotting...')

%% Add you plots here
figure(101)
plot(xc_n, yc_n, 'cyan', xc,yc,"r-",xc_hat, yc_hat, "k");
grid on
legend('Measurement', 'Ground Truth', 'Estimation' );
ylabel('y(m)');
xlabel('x(m)');

figure(102);
subplot(211)
plot(time, xc_n, 'cyan', time, xc, "r", time, xc_hat, 'k');
legend('Measurement', 'Ground Truth', 'Estimation')
ylabel('x(m)')
grid on
subplot(212)
plot(time, yc_n, 'cyan', time, yc, "r", time, yc_hat, 'k');
legend('Measurement', 'Ground Truth', 'Estimation')
ylabel('y(m)');
xlabel('time(s)')
grid on

figure(103);
error = sqrt((xc-xc_hat).^2 + (yc-yc_hat).^2);
plot(time, error);
xlabel('time(s)');
ylabel('Position Error(m)');
grid on

figure(104);
subplot(211)
plot(time, vx_n, 'cyan', time, vx, "r", time, vx_hat, 'k');
legend('Measurement', 'Ground Truth', 'Estimation')
ylabel('$v_x(m)$')
grid on
subplot(212)
plot(time, vy_n, 'cyan', time, vy, "r", time, vy_hat, 'k');
legend('Measurement', 'Ground Truth', 'Estimation')
ylabel('$v_y(m)$');
xlabel('time(s)')
grid on
