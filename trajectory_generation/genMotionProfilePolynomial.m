function [pos_f, vel_f, acc_f, jerk_f] = genMotionProfile(arc_len, init_cond, final_cond, path, ds, dt)
% Generate a thrust profile
% Then integrate the thrust profile over the path to generate the 3 different
% axis

% Could potentially make this a receding horizon problem when tracking the
% moving target where the horizon would need to be large enough such that
% you could decelerate to v = 0 within one horizon otherwise you would need
% to calc proper final jerks and thrusts

% Min integral of the derivate of the thrust (which is like minimizing the
% jerk)

% model the thrust as a nth order polynomial
n = 5;
num_terms = n + 1;


s_start = 0;
s_final = arc_len(1);% + arc_len(2) + arc_len(3);

max_vel = 20;
max_acc = 10;
max_jerk = 10;

syms a b c d e s g f;

thrust = a*s^4 + b*s^3 + c*s^2 + d*s + e;
jerk = diff(thrust);
snap = diff(jerk);
J = int((snap^2), 0, s_final);
H_sym = hessian(J, [a b c d e]);
H = double(H_sym); % right now this minimzes the jerk. To minimize time, I would want to maximize jerk with in-equality constraints

f = [0 0 0 0 0];

A = [];
B = [];
	 
Aeq = [0 0 0 0 1;
	   s_final^4, s_final^3, s_final^2, s_final, 1;
	   (1/30)*s_final^6, (1/20)*s_final^5, (1/12)*s_final^4, (1/6)*s_final^3, (1/2)*s_final^2
	   (1/5)*s_final^5, (1/4)*s_final^4, (1/3)*s_final^3, (1/2)*s_final^2, s_final];

Beq = [1; 0; s_final; 0]; % assumes the initial conditions have been dealt with

x = quadprog(H, f, A, B, Aeq, Beq);
% The resulting function should NEVER go above the final arc length line
s = 0:ds:s_final;
y = x(1).*s.^4 + x(2).*s.^3 + x(3)*s.^2 + x(4)*s + x(5); 
y_int = (1/30)*x(1).*s.^6 + (1/20)*x(2).*s.^5 + (1/12)*x(3)*s.^4 + (1/6)*x(4)*s.^3 + (1/2)*x(5)*s.^2 + 0 + 0;
y_vel = 1/5*x(1)*s.^5 + 1/4*x(2)*s.^4 + 1/3*x(3)*s.^3 + 1/2*x(4)*s.^2 + x(5)*s;
y_der = 4*x(1).*s.^3 +3* x(2).*s.^2 + 2*x(3)*s + x(4);
figure;
plot(s,y);
hold on;
plot(s, y_int);
plot(s, y_der);
plot(s, y_vel);
legend('thrust', 'pos', 'jerk', 'vel');
hold off;

t(1) = 0;
acc(1) = x(5);
vel(1) = 0;
len(1) = 0;
for k = 1:3
	pos(k, 1) = getLocation(path(k, 1), path(k, 2), path(k, 3), path(k, 4), 0);
end

vel_space(:,1) = [0 0 0];
acc_space(:,1) = [0.5 0 3];
jerk_space(:,1) = [0 0 0];
counter = 2;
path_seg = 0;
length_next_seg = arc_len(1);
start_len = 0;
switch_point = 0;
first_switch = 0;
s_temp = 0;
while(len(length(len)) <= s_final-.01) % can put in a "close enough factor"
	acc(counter) = getThrust(x, s_temp);
	s_temp = s_temp + dt; % i think this is the way to go
	vel(counter) = vel(counter-1) + dt*acc(counter);
	len(counter) = len(counter-1) + dt*vel(counter);
	% need to figure out how to switch between splines. prob just use how
	% far you have moved in arc length
	if len(counter) >= length_next_seg
		path_seg = path_seg + 1;
		if path_seg < length(arc_len)
			length_next_seg = length_next_seg + arc_len(path_seg+1);
		end
		if path_seg == length(arc_len)
			path_seg = length(arc_len) -1;
		end
		
		start_len = start_len + arc_len(path_seg);		
	end
	
	for k = 1:3
		pos(k, counter) = getLocation(path(k, 1+(path_seg*4)), path(k, 2+(path_seg*4)), path(k, 3+(path_seg*4)), path(k, 4+(path_seg*4)), (len(counter)-start_len)/arc_len(path_seg+1));
		vel_space(k, counter) = (pos(k, counter) - pos(k, counter-1))/dt;
		acc_space(k, counter) = (vel_space(k, counter) - vel_space(k, counter-1))/dt;
		jerk_space(k, counter) = (acc_space(k, counter) - acc_space(k, counter-1))/dt;
	end
	
	t(counter) = t(counter-1) + dt;
	counter = counter + 1;
end
figure;
plot(t, len);
figure;
plot3(pos(1, :), pos(2, :), pos(3, :), 'MarkerSize', 5, 'Marker', 'o');
title('position in 3D');
grid on;
axis equal;

figure;
plot(t, 1:counter-1);
title('t vs counter');

% switch_point = counter-1;
% figure;
% plot(len(1:switch_point), pos(1,1:switch_point));
% hold on;
% plot(len(1:switch_point), pos(2,1:switch_point));
% plot(len(1:switch_point), pos(3,1:switch_point));
% hold off;
% %title('Position 1st Segment fcn of Arc Len');
% legend('x1', 'y1', 'z1')
% hold on;
% plot(len(switch_point:counter-1), pos(1,switch_point:counter-1), 'color', 'r');
% plot(len(switch_point:counter-1), pos(2,switch_point:counter-1), 'color', 'g');
% plot(len(switch_point:counter-1), pos(3,switch_point:counter-1), 'color', 'm');
% hold off;
% title('Position 1st Segment fcn of Arc Len');
% legend('x2', 'y2', 'z2')


figure;
plot(len, vel_space(1,:));
hold on;
plot(len, vel_space(2,:));
plot(len, vel_space(3,:));
hold off;
title('Velocity as a fcn of Arc Length');
legend('x', 'y', 'z');
grid on;

figure;
plot(len, acc_space(1,:));
hold on;
plot(len, acc_space(2,:));
plot(len, acc_space(3,:));
hold off;
title('Acceleration as a fcn of Arc Length');
legend('x', 'y', 'z');

pos_f = pos;
vel_f = vel_space;
acc_f = acc_space;
jerk_f= jerk_space;

end

function thrust = getThrust(x, pos)
	thrust = x(1)*pos^4 + x(2)*pos^3 + x(3)*pos^2 + x(4)*pos + x(5);
end

function loc = getLocation(p_0, p_1, p_2, p_3, s)
	loc = (1-s)^3*p_0 + 3*(1-s)^2*s*p_1 + 3*(1-s)*s^2*p_2 + s^3*p_3;
end

function vel = getVel(p_0, p_1, p_2, p_3, s)
	vel = 3*(1-s)^2*(p_1-p_0) + 6*(1-s)*s*(p_2-p_1) + 3*s^2*(p_3-p_2);
end

function acc = getAcc(p_0, p_1, p_2, p_3, s)
	acc = 6*(1-s)*(p_2-2*p_1+p_0) + 6*s*(p_3-2*p_2+p_2);
end