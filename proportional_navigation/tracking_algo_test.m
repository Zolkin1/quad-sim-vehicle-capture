close all;
clear;

tf = 60;
dt = 0.001;

target_start = [20; 30];
target_position = target_start;
target_velocity = [1; 1];

q_start = [10; 10];
q_position = q_start;
q_velocity = [0.1; 0.1]; % test what happens with different intital values
q_acceleration = [0;0];

simCounter = 1;

psi_i = 0;
psi_des = 0;
psi_e_prev = 0;
kp_psi = .01;
kd_psi = 0;
ki_psi = .01;

pos_int = 0;
pos_i = 0;
pos_e_prev = 0;
kp_pos = 0.5;
kd_pos = 7;
ki_pos = 0;


for t = 0:dt:tf

	lambda = 5;
	
	% this implementation is not correct. look at the graphs. try to plot
	% the angle and see thats it derivative goes to 0
	LOS = -target_position + q_position;
	LOS_hist(:, simCounter) = LOS;
	
	psi = acos(dot(target_velocity, LOS)/(norm(LOS) * norm(target_velocity)));
	psi_hist(simCounter) = psi;
	
	% track to an angle of 0
	psi_e = psi_des - psi;
	psi_p = kp_psi * psi_e;
	psi_d = kd_psi * (psi_e - psi_e_prev)/dt;
	psi_i = ki_psi * (psi_i + psi_e*dt);
	psi_e_prev = psi_e;
	
	psi_a = psi_p + psi_d + psi_i;
	% do the math of the effect of psi on the system
	% need to figure out how to incorporate this into the acceleration
	
	% starting by just tracking the current location
	pos_e = target_position - q_position;
	pos_e_hist(:, simCounter) = pos_e;
	pos_p = kp_pos * pos_e;
	pos_d = kd_pos * (pos_e - pos_e_prev)/dt;
	pos_i = ki_pos * (pos_i + pos_e*dt);
	pos_e_prev = pos_e;
	
	pos_a = pos_p + pos_d + pos_i;
	
	ang_vel = getAngle(target_velocity, q_velocity);
	ang_vel_hist(simCounter) = ang_vel;
	
	rotMat = [cos(ang_vel) sin(ang_vel); -sin(ang_vel) cos(ang_vel)];
	
	a = pos_a; %+ (rotMat * (pos_a/norm(pos_a)))*psi_a;
	
	q_accleration = a;
	q_acc_past(:, simCounter) = q_accleration;
	
	q_velocity = q_velocity + q_accleration * dt;
	q_vel_past(:, simCounter) = q_velocity;
	
	q_position = q_position + q_velocity * dt;
	q_past_pos(:, simCounter) = q_position;
	
	target_position = target_position + target_velocity * dt;
	target_past_pos(:, simCounter) = target_position;
	
% 	if (abs(target_position - q_position) < 0.001)
% 		fprintf('Target intercepted. Time: %i\n', t);
% 		t = tf + 1;
% 	end
	
	simCounter = simCounter + 1;
end

figure;
plot(q_past_pos(1, :), q_past_pos(2, :));
hold on
plot(target_past_pos(1, :), target_past_pos(2, :), 'Color', 'red', 'Linestyle', '--');
% for n = 1:1000:simCounter-1
% 	x_temp = [target_past_pos(1, n), target_past_pos(1, n) + LOS_hist(1, n)];
% 	y_temp = [target_past_pos(2, n), target_past_pos(2, n) + LOS_hist(2, n)];
% 	line(x_temp, y_temp, 'Color', 'red', 'Linestyle', '--');
% end
hold off

figure;
subplot(2,1,1);
plot(0:dt:tf, q_vel_past);
legend('vx', 'vy');
subplot(2,1,2);
plot(0:dt:tf, q_acc_past);
legend('ax', 'ay');

figure;
plot(0:dt:tf, psi_hist);
legend('psi');

figure;
plot(0:dt:tf, ang_vel_hist)
title('angle between velocity vectors');

figure;
plot(0:dt:tf, pos_e_hist);

function angle = getAngle(vector1, vector2)
	angle = acos(dot(vector1, vector2)/(norm(vector1) * norm(vector2)));
end

