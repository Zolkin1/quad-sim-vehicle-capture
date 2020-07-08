clear
syms t p0x p1x p2x p3x p0y p1y p2y p3y p0z p1z p2z p3z
bx = (1-t)^3*p0x + 3*(1-t)^2*t*p1x + 3*(1-t)*t^2*p2x + t^3*p3x;
by = (1-t)^3*p0y + 3*(1-t)^2*t*p1y + 3*(1-t)*t^2*p2y + t^3*p3y;
bz = (1-t)^3*p0z + 3*(1-t)^2*t*p1z + 3*(1-t)*t^2*p2z + t^3*p3z;
bx_d = diff(bx);
by_d = diff(by);
bz_d = diff(bz);
mag = sqrt(bx_d^2 + by_d^2 + bz_d^2);

T = [bx_d/mag, by_d/mag, bz_d/mag];

kappa = sqrt(T(1)^2 + T(2)^2 + T(3)^2)/(mag);

ki = int(kappa, t, 0, 1);

% r= [bx by bz];
% r_prime = diff(r);
% 
% Jx = int(r_prime(1)^2+r_prime(2)^2+r_prime(3)^2, t, 0, 1);
% Hx = double(hessian(Jx, [p0x p1x p2x p3x p0y p1y p2y p3y p0z p1z p2z p3z]));
% 
% f = [0 0 0 0 0 0 0 0 0 0 0];
% A = [0 1 0 0 0 0 0 0 0 0 0 0;
% 	 0 0 1 0 0 0 0 0 0 0 0 0;
% 	 0 0 0 0 0 1 0 0 0 0 0 0;
% 	 0 0 0 0 0 0 1 0 0 0 0 0;
% 	 0 0 0 0 0 0 0 0 0 1 0 0;
% 	 0 0 0 0 0 0 0 0 0 0 1 0];
% b = [-2; -2.25; ];
% Aeq = [1 0 0 0 0 0 0 0 0 0 0 0;
% 	   0 0 0 1 0 0 0 0 0 0 0 0;
% 	   0 0 0 0 1 0 0 0 0 0 0 0;
% 	   0 0 0 0 0 0 0 1 0 0 0 0;
% 	   0 0 0 0 0 0 0 0 1 0 0 0;
% 	   0 0 0 0 0 0 0 0 0 0 0 1];
% Beq = [2; 2.25; 1; 1; 0; 8];
% 
% quadprog(Hx, f, A, b, Aeq, Beq);