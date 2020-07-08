close all;
clear;

tf = 60;
dt = 0.001;

target_start = [10; 10; 10];
target_position = target_start;
target_velocity = [1; 1; 1];

% if you start directly in line in a plane then the algo does not work
q_start = [60; 30; 40];
q_positionXY = [q_start(1); q_start(2)];
q_velocityXY = [0.1; 0.1];
q_accelerationXY = [0;0];
% not convinced this will work, because in theory you can have 2 different
% velocities in Y
q_positionYZ = [q_start(2); q_start(3)];
q_velocityYZ = [0.1; 0.1];
q_accelerationYZ =[0;0];

q_position = q_start;

simCounter = 1;

psi_int = 0;
for t = 0:dt:tf

	lambda = 4.975;
	[a_xy, histories_xy] = calcPNPlane(simCounter, lambda, [target_position(1); target_position(2)], [target_velocity(1); target_velocity(2)], [q_positionXY(1); q_positionXY(2)], [q_velocityXY(1); q_velocityXY(2)]);
	[a_yz, histories_yz] = calcPNPlane(simCounter, lambda, [target_position(2); target_position(3)], [target_velocity(2); target_velocity(3)], [q_positionYZ(1); q_positionYZ(2)], [q_velocityYZ(1); q_velocityYZ(2)]);
	% START debug by ifguring out why YZ doesn't track well
	LOS_xy = histories_xy; 
	LOS_yz = histories_yz;
	
	LOS_hist_yz(:, simCounter) = LOS_yz;
	
	LOS_hist(:, simCounter) = [LOS_xy(1); (LOS_xy(2)); LOS_yz(2)];
	% its weird that using xy for the y compoenent works, but not using yz
	% something is up with the anlges and tracking after a few
	% oscilliations
	% can work on minimizing overshoot/oscillations by maybe adding PID.
	% should probably do some math rather than guess and check
	% can eventually look at the underactuated side of things
	
	% the paper does not mix acclerations and instead independetly evaluates
	% the dynamics for each plane. then the shared coordinate is averaged
	q_acclerationXY = [sign(LOS_xy(2)) * a_xy(1); sign(LOS_xy(2)) * a_xy(2)]; %sign(LOS_hist(:, simCounter)) .*
	q_acc_pastXY(:, simCounter) = q_acclerationXY;
	
	q_velocityXY = q_velocityXY + q_acclerationXY * dt;
	q_vel_pastXY(:, simCounter) = q_velocityXY;
	
	q_positionXY = q_positionXY + q_velocityXY * dt;
	q_past_posXY(:, simCounter) = q_positionXY;
	
	%
	
	q_acclerationYZ = [sign(LOS_yz(2)) * a_yz(1); sign(LOS_yz(2)) * a_yz(2)]; %sign(LOS_hist(:, simCounter)) .*
	q_acc_pastYZ(:, simCounter) = q_acclerationYZ;
	
	q_velocityYZ = q_velocityYZ + q_acclerationYZ * dt;
	q_vel_pastYZ(:, simCounter) = q_velocityYZ;
	
	q_positionYZ = q_positionYZ + q_velocityYZ * dt;
	q_past_posYZ(:, simCounter) = q_positionYZ;
	
	%
	% for some reason the y position of the YZ plane tracking is not
	% correct
	temp = (q_positionXY(2) + q_positionYZ(1))/2;
	q_position = [q_positionXY(1); temp; q_positionYZ(2)];
	
	q_positionXY(2) = temp;
	q_positionYZ(1) = temp;
	
	q_pos_hist(:, simCounter) = q_position;
	
	target_position = target_position + target_velocity * dt;
	target_past_pos(:, simCounter) = target_position;
	
	if (norm(target_position - q_position) < 0.01)
		fprintf('Target intercepted. Time: %i\n', t);
		t = tf + 1;
	end
	
	simCounter = simCounter + 1;
end

figure;
plot3(q_pos_hist(1, :), q_pos_hist(2, :), q_pos_hist(3, :), '-o','Color', 'm', 'MarkerSize', 4);
hold on
plot3(target_past_pos(1, :), target_past_pos(2, :), target_past_pos(3, :), '-o','Color', 'b', 'MarkerSize', 4);
for n = 1:1000:simCounter-1
	x_temp = [target_past_pos(1, n), target_past_pos(1, n) + LOS_hist(1, n)];
	y_temp = [target_past_pos(2, n), target_past_pos(2, n) + LOS_hist(2, n)];
	z_temp = [target_past_pos(3, n), target_past_pos(3, n) + LOS_hist(3, n)];
	line(x_temp, y_temp, z_temp, 'Color', 'red', 'Linestyle', '--');
end
hold off
grid on;
axis('equal');
xlabel('x');
ylabel('y');
zlabel('z');

figure;
subplot(2,1,1)
plot(q_past_posXY(1, :), q_past_posXY(2, :), '-o','Color', 'm', 'MarkerSize', 4)
hold on
plot(target_past_pos(1, :), target_past_pos(2, :), '-o','Color', 'b', 'MarkerSize', 4);
hold off
legend('XY Q', 'XY Target');
xlabel('x');
ylabel('y');

subplot(2,1,2)
plot(q_past_posYZ(2, :), q_past_posYZ(1, :), '-o','Color', 'm', 'MarkerSize', 4)
hold on
plot(target_past_pos(3, :), target_past_pos(2, :), '-o','Color', 'b', 'MarkerSize', 4);
for n = 1:10000:simCounter-1
	x_temp = [target_past_pos(3, n), target_past_pos(3, n) + LOS_hist_yz(2, n)];
	y_temp = [target_past_pos(2, n), target_past_pos(2, n) + LOS_hist_yz(1, n)];
clc
line(x_temp, y_temp, 'Color', 'red', 'Linestyle', '--');
end
hold off
xlabel('z');
ylabel('y');
legend('YZ Q', 'YZ Target');


figure;
subplot(2,1,1);
plot(0:dt:tf, q_vel_pastXY);
legend('vx', 'vy');
subplot(2,1,2);
plot(0:dt:tf, q_acc_pastXY);
legend('ax', 'ay');

figure;
subplot(2,1,1);
plot(0:dt:tf, q_vel_pastYZ);
legend('vy', 'vz');
subplot(2,1,2);
plot(0:dt:tf, q_acc_pastYZ);
legend('ay', 'az');

figure;
plot(0:dt:tf, LOS_hist)
legend('losX', 'losY', 'losZ');

% figure;
% plot(0:dt:tf, psi_hist);
% hold on;
% plot(0:dt:tf, psi_dot_hist);
% plot(0:dt:tf, psi_int_hist + 0.78);
% hold off;
% legend('psi', 'psi dot', 'psi_int');

% anywhere where the LOS is very small will probably have a few issues
% because we will be diving by an almost 0 value thus making the angles
% shoot up to a very large number
% should consider what steps can be taken to mitidate that
function [acc, histories] = calcPNPlane(simCounter, lambda, target_position, target_velocity, q_position, q_velocity)
	LOS = -target_position + q_position;
	LOS_hist(:, simCounter) = LOS;
	
	% velocities cannot be 0
	psi = acos(dot(target_velocity, LOS)/(norm(LOS) * norm(target_velocity)));
	psi_hist(simCounter) = psi;
	chi_diff_q = acos(dot(q_velocity, LOS)/(norm(LOS) * norm(q_velocity)));
	chi_diff_t = acos(dot(target_velocity, LOS)/(norm(LOS) * norm(target_velocity)));
	
	psi_dot = (norm(target_velocity)*sin(chi_diff_t) - norm(q_velocity)*sin(chi_diff_q))/norm(LOS);
	psi_dot_hist(simCounter) = psi_dot;
	
% 	psi_int = psi_int + psi_dot * dt; 
% 	psi_int_hist(simCounter) = psi_int;
	
	v_r = q_velocity - target_velocity;
	
	a_des_mag = lambda * norm(v_r) * psi_dot;

	rot_matrix = [0, -1; 1, 0];
	
	v_r_norm = v_r/norm(v_r);

	acc = (a_des_mag * (rot_matrix * v_r_norm));
	histories = LOS; %psi_hist; psi_dot_hist]; 
end


