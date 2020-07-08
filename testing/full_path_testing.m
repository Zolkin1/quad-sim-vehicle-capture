clear;
close all;

obs_pos = [1 1 8];
obs_radi = [0.5 0.5 0.25];
buffer = 0.75;
start_pos = [1 1 0];
tilt_angle = pi/6;
claw_length = 0.4;
ds = 0.001;

[pathCoefs, path, arc_len, T, N] = makeGeoPath(obs_pos, obs_radi, buffer, start_pos, tilt_angle, claw_length);
[r_s, v_s, a_s, j_s] = genMotionProfile2(arc_len, pathCoefs, ds, 0.01, path);
pos = [6; 6; 0];

simCounter = 2;
dt = 0.01;
len_traveled = 0;
s = 0;
vel = zeros(3,1);
acc = zeros(3,1);
index = 1;
while index < length(a_s)%len_traveled < arc_len
	%[s, len_traveled] = getClosestPoint(pos(:, size(pos, 2)), path, pathCoefs, arc_len, s);
	if (s<=1)
	for k = 1:3
% 		index = cast(s*length(r_s), 'uint32');
% 		
% 		if index == 0
% 			index = index + 1;
% 		end

		acc(k, simCounter) = a_s(k, index); % + 0.7*(v_s(k, index) - vel(k, simCounter-1)) + 0.00*(r_s(k, index) - pos(k, simCounter-1));
		vel(k, simCounter) = vel(k, simCounter-1) + acc(k, simCounter)*dt;
		vel_des(k, simCounter) = v_s(k, index);
		pos(k, simCounter) = pos(k, simCounter-1) + vel(k, simCounter)*dt;
	end
	index = index + 1;

	%s = getArcLength
	s_hist(simCounter) = s;

	simCounter = simCounter + 1;
	end
end

% figure;
% plot3(pos(1, :), pos(2,:), pos(3,:), 'Marker', 'o', 'MarkerSize', 5, 'Color', 'm');
% grid on;
% axis equal;
% 
% figure;
% plot(1:simCounter-1, acc);
% legend('x', 'y', 'z');
% title('Acclerations');
% 
% figure;
% plot(1:simCounter-1, vel);
% legend('x', 'y', 'z');
% title('Velocities');
% 
% figure;
% plot(1:simCounter-1, vel_des);
% legend('x', 'y', 'z');
% title('Velocities Desired');