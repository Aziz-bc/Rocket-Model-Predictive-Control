%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20; % Sample time [s]
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Design MPC controllers for each subsystem
H = 10; % Horizon length [s]
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_zwe = MpcControl_zwe(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);
mpc_zwe_merged = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_zwe, mpc_roll);

% Setup reference function
Tf = 30;
ref = @(t_, x_) ref_EPFL(t_);
x0 = zeros(12,1);

rocket.mass = 1.794; % Manipulate mass for simulation
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
[~, X_we, ~, ~, ~] = rocket.simulate_est_z(x0, Tf, @mpc_zwe_merged.get_u, ref, mpc_zw, sys_z);

figure;
plot(T, Ref(3,:),T, X(12,:),T,X_we(12,:));
title('Offset-Free Tracking');
legend('Reference','Original Controller','Offset-Free Tracking');
xlabel('Time [s]');
ylabel('z [m]');


