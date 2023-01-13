addpath(fullfile('..', 'src'));
clc
clear all
close all
%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Design MPC controller                 
Tf = 10;

%% MPC controller for x
H = 6;  % Horizon length in seconds
mpc_x = MpcControl_x(sys_x, Ts, H);

% Initial x
x0 = [0; 0; 0 ;4];

% Open loop
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x0);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us); % Plot as usual
sgtitle("MPC controller for X open-loop");

% Closed loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
sgtitle("MPC controller for X closed-loop");


%% MPC controller for Y
H = 6; % Horizon length in seconds
mpc_y = MpcControl_y(sys_y, Ts, H);

% Initial y
y0 = [0; 0; 0 ;4];

% Open loop
[u, T_opt, X_opt, U_opt] = mpc_y.get_u(y0);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us); % Plot as usual
sgtitle("MPC controller for Y open-loop");

% Closed loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
sgtitle("MPC controller for Y closed-loop");

%% MPC controller for Z
H = 3;  % Horizon length in seconds
mpc_z = MpcControl_z(sys_z, Ts, H);

% Initial z
z0 = [0;4];

% Open loop
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(z0);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us); % Plot as usual
sgtitle("MPC controller for Z open-loop");

% Closed loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);
sgtitle("MPC controller for Z closed-loop");
%% MPC controller for roll
H = 1; % Horizon length in seconds
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Initial angle
roll0 = [0;deg2rad(35)];

% Open loop
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(roll0);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us); % Plot as usual
sgtitle("MPC controller for roll open-loop");

% Closed loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);
sgtitle("MPC controller for roll closed-loop");