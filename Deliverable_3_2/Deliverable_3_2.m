%addpath(fullfile('..', 'src'));
clc
clear all
close all
%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20; % Sample time 
rocket = Rocket(Ts);
[xs, us] = rocket.trim(); 
sys = rocket.linearize(xs, us); 
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);               
Tf = 10;
%% MPC controller for x
H = 6;          % Horizon length in seconds
mpc_x = MpcControl_x(sys_x, Ts, H);

% Initial Position x
x0 = [0 ; 0; 0 ; 0];
% Reference Position x
x_ref = -4; 

% Open loop
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x0, x_ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us, x_ref); % Plot as usual
sgtitle("MPC controller for X open-loop");

% Closed loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, x_ref);
sgtitle("MPC controller for X closed-loop");
%% MPC controller for y
H = 6;          % Horizon length in seconds
mpc_y = MpcControl_y(sys_y, Ts, H);

% Initial Position x
y0 = [0 ; 0; 0 ; 0];
% Reference Position x
y_ref = -4; 

% Open loop
[u, T_opt, X_opt, U_opt] = mpc_y.get_u(y0, y_ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us, y_ref); % Plot as usual
sgtitle("MPC controller for Y open-loop");

% Closed loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, y_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, y_ref);
sgtitle("MPC controller for Y closed-loop");
%% MPC controller for z
H = 3;          % Horizon length in seconds
mpc_z = MpcControl_z(sys_z, Ts, H);

% Initial Position x
z0 = [0 ; 0];
% Reference Position x
z_ref = -4; 

% Open loop
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(z0, z_ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us, z_ref); % Plot as usual
sgtitle("MPC controller for Z open-loop");

% Closed loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, z_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, z_ref);
sgtitle("MPC controller for Z closed-loop");
%% MPC controller for roll
H = 1;          % Horizon length in seconds
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Initial angle 
roll0 = [0 ; 0];
% Reference Position x
roll_ref = deg2rad(35); 

% Open loop
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(roll0, roll_ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us, roll_ref); % Plot as usual
sgtitle("MPC controller for roll open-loop");

% Closed loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, roll_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, roll_ref);
sgtitle("MPC controller for roll closed-loop");