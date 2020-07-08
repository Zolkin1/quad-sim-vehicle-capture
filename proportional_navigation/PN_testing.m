close all;
clear;

tf = 60;
dt = 0.001;

target_start = [20; 30];
target_position = target_start;
target_velocity = [1; 0];

q_start = [10; 10];
q_position = q_start;
q_velocity = [0.1; 0.1]; % test what happens with different intital values
q_acceleration = [0;0];

simCounter = 1;

psi_int = 0;
for t = 0:dt:tf

	lambda = 5;
	
	% this implementation is not correct. look at the graphs. try to plot
	% the angle and see thats it derivative goes to 0
	LOS = -target_position + q_position;
	LOS_hist(:, simCounter) = LOS;
	
	psi = acos(dot(target_velocity, LOS)/(norm(LOS) * norm(target_velocity)));
	psi_hist(simCounter) = psi;
	chi_diff_q = acos(dot(q_velocity, LOS)/(norm(LOS) * norm(q_velocity)));
	chi_q_hist(simCounter) = chi_diff_q;
	chi_diff_t = acos(dot(target_velocity, LOS)/(norm(LOS) * norm(target_velocity)));
	chi_t_hist(simCounter) = chi_diff_t;
	
	psi_dot = (norm(target_velocity)*sin(chi_diff_t) - norm(q_velocity)*sin(chi_diff_q))/norm(LOS);
	psid_t_hist(simCounter) = norm(target_velocity)*sin(chi_diff_t);
	psid_q_hist(simCounter) = norm(q_velocity)*sin(chi_diff_q);
	psi_dot_hist(simCounter) = psi_dot;
	
	
	psi_int = psi_int + psi_dot * dt; 
	psi_int_hist(simCounter) = psi_int;
	
	v_r = q_velocity - target_velocity;
	
	a_des_mag = lambda * norm(v_r) * psi_dot;

	rot_matrix = [0, -1; 1, 0];
	
	v_r_norm = v_r/norm(v_r);

	
	a = (sign(LOS(2)) * (a_des_mag * (rot_matrix * v_r_norm))); %null(temp(:).');
	q_accleration = a;
	q_acc_past(:, simCounter) = q_accleration;
	
	q_velocity = q_velocity + q_accleration * dt;
	q_vel_past(:, simCounter) = q_velocity;
	
	q_position = q_position + q_velocity * dt;
	q_past_pos(:, simCounter) = q_position;
	
	target_position = target_position + target_velocity * dt;
	target_past_pos(:, simCounter) = target_position;
	
	if (abs(target_position - q_position) < 0.001)
		fprintf('Target intercepted. Time: %i\n', t);
		t = tf + 1;
	end
	
	simCounter = simCounter + 1;
end

figure;
plot(q_past_pos(1, :), q_past_pos(2, :));
hold on
plot(target_past_pos(1, :), target_past_pos(2, :));
for n = 1:1000:simCounter-1
	x_temp = [target_past_pos(1, n), target_past_pos(1, n) + LOS_hist(1, n)];
	y_temp = [target_past_pos(2, n), target_past_pos(2, n) + LOS_hist(2, n)];
	line(x_temp, y_temp, 'Color', 'red', 'Linestyle', '--');
end
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
hold on;
plot(0:dt:tf, psi_dot_hist);
plot(0:dt:tf, psi_int_hist + 0.78);
hold off;
legend('psi', 'psi dot', 'psi_int');

% figure;
% plot(0:dt:tf, chi_q_hist);
% hold on;
% plot(0:dt:tf, chi_t_hist);
% hold off;
% legend('chi q', 'chi t');

% figure;
% plot(0:dt:tf, psid_t_hist);
% hold on;
% plot(0:dt:tf, psid_q_hist);
% hold off;
% legend('t portion of psi dot', 'q portion of psi dot');


