function [pos_f, vel_f, acc_f, jerk_f] = genMotionProfile2(arc_len, path, ds, dt, path2)
% Generate a thrust profile
s_final = arc_len(1);% + arc_len(2);% + arc_len(3);
[len vel acc jerk] = optimizeSplineTrajectory(dt, s_final);

t(1) = 0;
for k = 1:3
	pos(k, 1) = getLocation(path(k, 1), path(k, 2), path(k, 3), path(k, 4), 0);
end

vel_space(:,1) = [0 0 0];
acc_space(:,1) = [0.5 0 3];
jerk_space(:,1) = [0 0 0];
counter = 1;
path_seg = 0;
length_next_seg = arc_len(1);
start_len = 0;
switch_point = 0;
first_switch = 0;
s_temp = 0;
ind(1) = 2;
for h = 1:length(len)
if h > 1
	temp(h) = (len(h)-start_len)/arc_len(path_seg+1);
	if temp(h) >= 1 %len(h) >= length_next_seg
		path_seg = path_seg + 1;
		if path_seg < length(arc_len)
			length_next_seg = length_next_seg + arc_len(path_seg+1);
		end
		if path_seg == length(arc_len)
			path_seg = length(arc_len) -1;
		end
		
		start_len = start_len + arc_len(path_seg);		
	end
	
	[param(h), ind(h)] = invArcLen(path2(:,1), path2(:,2), path2(:,3), len(h), ind(h-1), len(h-1));%(length(path(:,1))/3);
	for k = 1:3
		pos(k, h) = getLocation(path(k, 1+(path_seg*4)), path(k, 2+(path_seg*4)), path(k, 3+(path_seg*4)), path(k, 4+(path_seg*4)), param(h));%(len(h)-start_len)/arc_len(path_seg+1));
		dp = getLocation(path(k, 1+(path_seg*4)), path(k, 2+(path_seg*4)), path(k, 3+(path_seg*4)), path(k, 4+(path_seg*4)), param(h)-.00001);
		vel_space(k, h) = (pos(k, h) - pos(k, h-1))/dt;
		acc_space(k, h) = (vel_space(k, h) - vel_space(k, h-1))/dt;
		jerk_space(k, h) = (acc_space(k, h) - acc_space(k, h-1))/dt;
		if (abs(acc_space(k, h)) > 15)
			acc_space(k,h) = sign(acc_space(k,h))*15;
		end
		if (abs(jerk_space(k, h)) > 9)
			jerk_space(k,h) = sign(jerk_space(k,h))*9;
		end
		
	end
	t(h) = t(h-1) + dt;
end
end
figure;
plot(t, len);
figure;
plot3(pos(1, :), pos(2, :), pos(3, :), 'MarkerSize', 5, 'Marker', 'o');
title('position in 3D');
grid on;
axis equal;

% figure;
% plot(t, temp);
% title('arc length from start of seg');
% grid on;

figure;
plot(t, pos(3,:));
title('z pos vs time');
grid on;

figure;
subplot(2,2,1)
plot(t, vel_space(1,:));
hold on;
plot(t, vel_space(2,:));
plot(t, vel_space(3,:));
hold off;
title('Velocity as a fcn of time');
legend('x', 'y', 'z');
grid on;

subplot(2,2,2);
plot(len, vel_space(1,:));
hold on;
plot(len, vel_space(2,:));
plot(len, vel_space(3,:));
hold off;
title('Velocity as a fcn of Arc Length');
legend('x', 'y', 'z');
grid on;

subplot(2,2,3)
plot(t/t(length(t)), vel_space(1,:));
hold on;
plot(t/t(length(t)), vel_space(2,:));
plot(t/t(length(t)), vel_space(3,:));
hold off;
title('Velocity as a normalized fcn of time');
legend('x', 'y', 'z');
grid on;

subplot(2,2,4);
plot(len/len(length(len)), vel_space(1,:));
hold on;
plot(len/len(length(len)), vel_space(2,:));
plot(len/len(length(len)), vel_space(3,:));
hold off;
title('Velocity as a normalized fcn of Arc Length');
legend('x', 'y', 'z');
grid on;

figure;
plot(len,param);
title('Arc len vs param');
grid on;

figure;
plot(t,len);
title('Arc Length Travled Vs Time');
grid on;

figure;
subplot(3,1,1);
plot(t, pos);
title('Position in space vs time');
legend('x', 'y', 'z');
grid on;

subplot(3,1,2);
plot(len/len(length(len)), pos);
title('Position in space vs of normalized arc length');
legend('x','y','z');
grid on;

for h = 1:1:length(len)
for k = 1:3
	extra(k, h) =  getLocation(path(k, 1+(path_seg*4)), path(k, 2+(path_seg*4)), path(k, 3+(path_seg*4)), path(k, 4+(path_seg*4)), h/length(len)-1/length(len));
end
end

subplot(3,1,3);
plot(1/length(len):1/length(len):1, extra);
title('Position as a fcn of parameter');
legend('x', 'y', 'z');
grid on;

figure;
plot(len, acc_space(1,:));
hold on;
plot(len, acc_space(2,:));
plot(len, acc_space(3,:));
hold off;
title('Acceleration as a fcn Arc Length');
legend('x', 'y', 'z');

pos_f = pos;
vel_f = vel_space;
acc_f = acc_space;
jerk_f= jerk_space;

count = 1;
for s = 0:0.01:1
	w(count) = getLocation(path(3,1), path(3,2), path(3,3), path(3,4), s);
	count = count + 1;
end

figure;
scatter(0:.01:s,w);

count = 1;
for s = 0:0.01:1
	w(count) = getLocation(path(3,5), path(3,6), path(3,7), path(3,8), s);
	count = count + 1;
end
hold on;
scatter(1:.01:2,w);
hold off;

end

function loc = getLocation(p_0, p_1, p_2, p_3, s)
	loc = (1-s)^3*p_0 + 3*(1-s)^2*s*p_1 + 3*(1-s)*s^2*p_2 + s^3*p_3;
end

function arc_len = getArcLen(x, y, z)
	c_f = length(x)/2;
	arc_len = 0;
	for k = 2:c_f
		arc_len = arc_len + sqrt((x(k)-x(k-1))^2 + (y(k)-y(k-1))^2 + (z(k)-z(k-1))^2);
	end
end

function [k, ind] = invArcLen(x,y,z,len, prevk, prevlen)
arc_len = prevlen;
k = prevk;
while arc_len <= len
	arc_len = arc_len + sqrt((x(k)-x(k-1))^2 + (y(k)-y(k-1))^2 + (z(k)-z(k-1))^2);
	k = k + 1;
end
ind = k;
k = k / 10000; %length(x);
end