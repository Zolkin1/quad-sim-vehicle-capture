% Orientation Controller
% Inputs:
% Desired b3
% Current b3
% Thrust
% kp
% Outputs:
% Commanded rotation

function [M, omega_d, e_r, a_norm, a_des] = orientationController(b3_d, b3_prev, a, v, linearVel, Rc, kp, kd, psi, J, g, dt, d, e_r_prev)
% TODO: Implement Controller as in the paper
% TODO: check the gains such that the inequality in the theorem is
% satisfied

% TODO: Maybe break the dynamics model in one with only rotation to test
% the rotation controller and one with only thrust to test the thrust
% controller.
	
	a_des = -1*(linearVel-v*b3_d) + a*b3_d + v*(b3_d-b3_prev)/0.001;
	a_norm = a_des/norm(a_des);
	%b3 = b3_d;
	e1 = [cos(psi); sin(psi); 0];
	
	% Create the body frame axis from the trajectory
	b3 = (a_des - g*[0; 0; 1])/norm(a_des - g*[0; 0; 1]); 
	b2 = cross(b3, e1)/norm(cross(b3, e1));
	b1 = cross(b2, b3);
	% Not accounting for G rn.
	
	Rd = [b1 b2 b3];
	Re = Rc*inv(Rd);
	
	if trace(Rd) == 3
		omega_d = [0 0 1];
	else
		omega_d = veeMap(lieLog(Rd));
	end
	e_omega = veeMap(lieLog(Re)) - omega_d; % error in my orientation/angular velocity
	
	%M = -kp*e_omega - kd*omega_d; % may need to adjust this

	% testing:
 	e_r = veeMap(0.5*(Rd'*Rc - Rc'*Rd)); % same direction as the above error, just different magnitudes
	e_rd = (e_r - e_r_prev)/dt;
	M = -10*e_r - 10*e_rd - 0*omega_d;
% 	e_rd = (w_be - e_r_prev)/dt;
% 	w_bde = (w_be - omega_d);
% 	M = -kp*w_bde - kd*e_rd - 0*omega_d - (cross(omega_d', J*omega_d'))'; % + kp*e_r' + kd*omega_d')';
% 	e_r = w_be;

	
end

function s = lieLog(R) % sometimes giving imaginary results
temp = trace(R);
phi = acos((temp-1)/2);
s = phi/(2*sin(phi)) * (R-R');
end

function s = veeMap(R)
s = [R(3,2), R(1,3), R(2,1)];
end

function d = timeDeriv(v, v_prev, dt)
d = (v - v_prev)/dt;
end