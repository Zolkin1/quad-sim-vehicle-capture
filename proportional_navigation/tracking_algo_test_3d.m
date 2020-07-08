close all;
clear;

tf = 60;
dt = 0.001;

target_start = [10; 10; 10];
target_position = target_start;
target_velocity = [10; 5; 1];

% if you start directly in line in a plane then the algo does not work
q_start = [20; 20; 40];
q_velocity = [0.1; 0.1; 0];
q_acceleration = [0;0;0];
q_position = q_start;

simCounter = 1;

psi_int = 0;


pos_int = 0;
pos_i = 0;
pos_e_prev = 0;
kp_pos = 0.5;
kd_pos = 7;
ki_pos = 0;

for t = 0:dt:tf

	pos_e = target_position - q_position;
	pos_e_hist(:, simCounter) = pos_e;
	pos_p = kp_pos * pos_e;
	pos_d = kd_pos * (pos_e - pos_e_prev)/dt;
	pos_i = ki_pos * (pos_i + pos_e*dt);
	pos_e_prev = pos_e;
	
	pos_a = pos_p + pos_d + pos_i;

	q_accleration = pos_a;
	q_acc_past(:, simCounter) = q_accleration;
	
	q_velocity = q_velocity + q_accleration * dt;
	q_vel_past(:, simCounter) = q_velocity;
	
	q_position = q_position + q_velocity * dt;
	q_past_pos(:, simCounter) = q_position;
	q_pos_hist(:, simCounter) = q_position;
	
	target_position = target_position + target_velocity * dt;
	target_past_pos(:, simCounter) = target_position;
	
% 	if (norm(target_position - q_position) < 0.01)
% 		fprintf('Target intercepted. Time: %i\n', t);
% 		t = tf + 1;
% 	end
	
	simCounter = simCounter + 1;
end

figure;
plot3(q_pos_hist(1, :), q_pos_hist(2, :), q_pos_hist(3, :), '-o','Color', 'm', 'MarkerSize', 4);
hold on
plot3(target_past_pos(1, :), target_past_pos(2, :), target_past_pos(3, :), '-o','Color', 'b', 'MarkerSize', 4);
hold off
grid on;
axis('equal');
xlabel('x');
ylabel('y');
zlabel('z');

figure;
plot(0:dt:tf, pos_e_hist);

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


