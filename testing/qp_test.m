% start with a simple 2D minimum jerk trajectory generation
% Assume 3rd order polynomial to start, but will probably need to increase
% this
% the time period is 1
% start location is 0,0
% end location is 1,1
t_start = 0;
t_end = 1;

f = [0 0 0 0];

A = [];%[0 0 2 6*t_end]; % can put in velocity, accleration and jerk constraints here
B = [];%[10];

Aeq = [1 t_end t_end^2 t_end^3;
	   1 0 0 0;
	   0 1 0 0]; % can put in location and derivative continuity constraints at the waypoints here
Beq = [1; 0; 1];

% H should be a Hessian matrix, specifically of the objective function
% H = [6*t_end 0 0 0 0 0 0 0; 
% 	 0 0 0 0 6*t_end 0 0 0;
% 	 0 0 0 0 0 0 0 0;
% 	 0 0 0 0 0 0 0 0;
% 	 0 0 0 0 0 0 0 0;
% 	 0 0 0 0 0 0 0 0;
% 	 0 0 0 0 0 0 0 0;
% 	 0 0 0 0 0 0 0 0];  

H = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 12];

OPTIONS = optimoptions('quadprog', 'Algorithm', 'interior-point-convex');
tic;
x = quadprog(H, f, A, B, Aeq, Beq, [], [], [], OPTIONS)
toc
% x_3*t^3 + 