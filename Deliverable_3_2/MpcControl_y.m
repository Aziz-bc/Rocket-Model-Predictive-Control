classdef MpcControl_y < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = [];
            
            % State Constraints 
            F = [eye(4); -eye(4)];
            f = [inf; deg2rad(7); inf; inf; inf; deg2rad(7); inf; inf];

            % Input constarints 
            M = [1; -1];
            m = [deg2rad(15);deg2rad(15)];          % constraints on the deflexion angle sigma 2
            
            % System dynamics 
            A = mpc.A; 
            B = mpc.B;
            C = mpc.C;
            D = mpc.D;
            
            % Cost matrices 
            Q = diag ([1 10 1 1]);
            R = 10;
            
            % LQR controller for unconstrained system
            [K, Qf, ~] = dlqr(A,B,Q,R);
            K = -K;      % MATLAB defines K as -K 

            % Compute the maximal invariant set 
            Xf = polytope([F;M*K],[f;m]);
            Acl = [A+B*K];      % Closed loop 
            while 1
                prevXf = Xf;
                [T,t] = double(Xf);
                preXf = polytope(T*Acl,t);
                Xf = intersect(Xf, preXf);
                if isequal(prevXf, Xf)
                    break
                end
            end
           [Ff,ff] = double(Xf);  
           
           % Objective function and constrains 
          
           for i = 1:N-1
              con = con + ( X(:,i+1) == A*X(:,i) + B*U(:,i) );
              con = con + ( F*X(:,i) <= f) + (M*U(:,i) <= m );
              obj = obj + (X(:,i) - x_ref)'*Q*(X(:,i) - x_ref) + (U(:,i) - u_ref)'*R*(U(:,i) - u_ref) ;
           end
      
           con = con + (Ff*( X(:,N) - x_ref ) <= ff);
           obj = obj + (X(:,N) - x_ref )'*Qf*(X(:,N) - x_ref);
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, {U(:,1), X, U});
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);

            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            obj = 0;
            con = [xs == 0, us == 0];
            
            % State Constraints 
            F = [eye(4); -eye(4)];
            f = [inf; deg2rad(7); inf; inf; inf; deg2rad(7); inf; inf];

            % Input constarints 
            M = [1; -1];
            m = [deg2rad(15);deg2rad(15)];          % constraints on the deflexion angle sigma 2
            
            % System dynamics 
            A = mpc.A; 
            B = mpc.B;
            C = mpc.C;
            D = mpc.D;
            
            con = [xs == A*xs + B*us, ref == C*xs + D*us, F*xs <= f, M*us <= m];
            obj = us'*us;
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
