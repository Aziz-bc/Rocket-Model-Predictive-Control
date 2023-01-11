addpath ('C:\Users\User\Desktop\Kraya\casadi-windows-matlabR2016a-v3.5.5')
% C:\gurobi1000\win64\matlab
%% Part 1 
%Todo1.1
d1=Rocket.indu.d1;
d2=Rocket.indu.d2;
Pavg=Rocket.indu.Pavg;
Pdiff=Rocket.indu.Pdiff;
w=Rocket.indx.omega;
phi=Rocket.indx.phi;
v=Rocket.indx.vel;
p=Rocket.indx.pos;
Ts = 1/20;
rocket = Rocket(Ts);
u = [d1, d2, Pavg, Pdiff]'; % (Assign appropriately)
[b_F, b_M] = rocket.getForceAndMomentFromThrust(u)
x = [w, phi, v, p]'; % (Assign appropriately)
x_dot = rocket.f(x, u)
%% Todo1.2
rocket = Rocket(Ts);
Tf = 2.0; % Simulation end time
x0 = [deg2rad([2 -2 0, -2 2 0]), 0 0 0, 0 0 0]'; % (w, phi, v, p) Initial state
u = [deg2rad([2 0]), 60, 0 ]'; % (d1 d2 Pavg Pdiff) Constant input
[T, X, U] = rocket.simulate(x0, Tf, u); % Simulate unknown, nonlinear model
rocket.anim_rate = 1.0; % Visualize at 1.0x real−time
rocket.vis(T, X, U);
%% Part 2 : Linearization 
% Todo2.1
rocket = Rocket(Ts);
[xs, us] = rocket.trim() % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us) % Linearize the nonlinear model about trim point
%% Todo2.2
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us)
