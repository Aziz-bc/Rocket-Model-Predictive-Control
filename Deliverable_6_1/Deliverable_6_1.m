Ts = 1/20; 
rocket = Rocket(Ts);
H = 3;
x0 = zeros(12,1);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%----------First experiment: NMPC and default maximum angle (15°)-----------%
% change Q and R cost matrices as follows : Q = diag([0 0 0 0 0 100 0 0 0 100 100 10]);R = diag([0.1 0.1 0.1 0.1]);
nmpc = NmpcControl(rocket, H);
ref15 = @(t_ , x_ ) ref_EPFL(t_);
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref15);
rocket.anim_rate = 3;
ph = rocket.plotvis(T, X, U,Ref); 

%----------Second experiment: NMPC and specified maximum angle (50°)-----------%
% change Q and R cost matrices as follows : Q = diag([0 0 0 0 0 100 0 0 0 100 100 10]);R = diag([0.1 0.1 0.1 0.1]);
roll_max = deg2rad(50);
ref50 = @(t_ , x_ ) ref_EPFL(t_,roll_max);
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref50);
ph = rocket.plotvis(T, X, U,Ref); 

%----------Third experiment: Linear MPC and the specified maximum angle (50°)-----------%
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);
% Setup reference function
ref = @(t_ , x_ ) ref_EPFL(t_,roll_max);
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
ph = rocket.plotvis(T, X, U, Ref);

