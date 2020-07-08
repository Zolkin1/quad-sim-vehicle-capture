close all;
clear;

dist_final = 20;

max_snap = 200;
max_jerk = 20;
max_acc = 20;
max_vel = 20;

snap(1) = max_snap;
jerk(1) = 0;
acc(1) = 0;
vel(1) = 0;
pos(1) = 0;
counter = 2;

dt = 0.001;
t = 0;

while jerk(counter-1) < max_jerk && acc(counter-1) < max_acc && vel(counter-1) < max_vel
	snap(counter) = max_snap;
	jerk(counter) = jerk(counter-1) + dt*max_snap;
	acc(counter) = acc(counter-1) + dt*jerk(counter);
	vel(counter) = vel(counter-1) + dt*acc(counter);
	pos(counter) = pos(counter-1) + dt*vel(counter);
	counter = counter + 1;
	t = t + dt;
end

while acc(counter-1) < max_acc && vel(counter-1) < max_vel
	snap(counter) = 0;
	jerk(counter) = max_jerk;
	acc(counter) = acc(counter-1) + dt*jerk(counter);
	vel(counter) = vel(counter-1) + dt*acc(counter);
	pos(counter) = pos(counter-1) + dt*vel(counter);
	counter = counter + 1;
	t = t + dt;
end

while vel(counter-1) < max_vel
	snap(counter) = 0;
	jerk(counter) = 0;
	acc(counter) = max_acc;
	vel(counter) = vel(counter-1) + dt*acc(counter);
	pos(counter) = pos(counter-1) + dt*vel(counter);
	counter = counter + 1;
	t = t + dt;
end

while pos(counter-1) < dist_final
	snap(counter) = 0;
	jerk(counter) = 0;
	acc(counter) = 0;
	vel(counter) = max_vel;
	pos(counter) = pos(counter-1) + dt*vel(counter);
	counter = counter + 1;
	t = t + dt;
end

snap_back(1) = snap(counter-1);
jerk_back(1) = jerk(counter-1);
acc_back(1) = acc(counter-1);
vel_back(1) = 0; %vel(counter-1);
pos_back(1) = pos(counter-1);

counter_back = 2;
t_back = t;

while vel_back(counter_back-1) < vel(counter-counter_back)
	vel_back(counter_back) = max_vel;
	pos_back(counter_back) = pos_back(counter_back-1) + dt*vel_back(counter_back);
	counter_back = counter_back + 1;
end

t = t+dt;
figure;
subplot(5,1,1);
plot(0:dt:t, snap);
legend('snap');
subplot(5,1,2);
plot(0:dt:t, jerk);
legend('jerk');
subplot(5,1,3);
plot(0:dt:t, acc);
legend('acc');
subplot(5,1,4);
plot(0:dt:t, vel);
hold on;
plot(t:-dt:t_back, vel_back);
hold off;
legend('vel');
subplot(5,1,5);
plot(0:dt:t, pos);
legend('pos');