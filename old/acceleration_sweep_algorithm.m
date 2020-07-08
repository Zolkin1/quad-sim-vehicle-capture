clear
close all;

obs_pos = [1 1 8];
obs_radi = [0.5 0.5 0.25];
buffer = 0.75;
start_pos = [1 1 0];
tilt_angle = pi/6;
claw_length = 0.4;
ds = 0.001;

[pathCoefs, path, arc_len, T, N, B, curvature, torsion] = makeGeoPath(obs_pos, obs_radi, buffer, start_pos, tilt_angle, claw_length);

v_max = 5;
a_max = 3;
j_max = 11;

k = 1:(length(path)*2/3);
max_angle_speed = deg2rad(100)/1;
v_al(k) = min(max_angle_speed./curvature(k), v_max);
table = makeArcLenLookUpTable(path(1:20002, :), 1);

v0 = 1;
a0 = 0;
j(1) = 0;
v_a(1) = 1;
r0 = 0;
t0 = 0;

r(1) = r0;
v_a(1) = 1;
a(1) = a0;
t(1) = t0;

i = 2;
f = 2;
dt = 0.001;
q(1) = 1;
qb(2) = 1;
d(1) = 0;
h = 1;
k_start = 0;
f_prev = 1;

while r(f-1) < table(length(table))-0.01
	[rt, v_at, at, tt, i, qt] = forwardInt(r0, v0, a0, t0, v_al, v_max, a_max, dt, f_prev, table, qb(2), table(length(table)));

	r = horzcat(r, rt);
	v_a = horzcat(v_a, v_at);
	a = horzcat(a, at);
	t = horzcat(t, tt);
	q = horzcat(q, qt);

	f = length(r);
	lambda(h) = r(length(r));
	h = h+1;
	% Get Next Switching Point %
	mu = findMinVel(v_al, k_start);
	k_start = mu;

	[r2, v2, a2, qb, g, k] = backwardInt(mu, lambda(h-1), table, v_al, a_max, dt, v_max, q, r, v_a);

	alpha = g-1;
	for f = k(length(k))+1:k(length(k))+g-1%f+1:f+g-1
		r(f) = r2(alpha);
		v_a(f) = v2(alpha);
		a(f) = a2(alpha);
		t(f) = t(f-1) + dt;
		q(f) = qb(alpha);
		alpha = alpha - 1;
	end

	f_prev = f;
	r0 = r(f);
	v0 = v_a(f);
	a0 = a(f);
	t0 = t(f); 
end

% Now clean up the extra stuff
r = r(1:f);
v_a = v_a(1:f);
a = a(1:f);
t = t(1:f);

i = i+1;
figure;
plot(1:length(table), table);

figure;
plot(1:length(v_a), v_a);
legend('Actual Velocity');

figure;
plot(1:length(r), r);
legend('Pos');

figure;
plot(1:length(a), a);
hold on;
plot(1:length(dt), dt);
hold off;
legend('a', 'dt');

figure;
plot(table, min(v_al, v_max));
title('Arc Len vs Max Vel');

figure;
plot(r, v_a);
hold on;
plot(r, a);
hold off;
legend('Velocity', 'Acceleration')

function table = makeArcLenLookUpTable(path, dt)
table(1) = 0;
for k = 2:length(path)
	table(k) = table(k-1) + sqrt(((path(k,1)-path(k-1,1))/dt)^2 + ((path(k,2)-path(k-1,2))/dt)^2 + ((path(k,3)-path(k-1,3))/dt)^2);
end
end

function [k, d] = getClosestIndex(table, r, k_p)
minNotFound = true;
k = min(k_p + 1, length(table));
if abs(table(k_p) - r) < abs(table(k) - r)
	k = k_p;
	d = abs(table(k_p) - r);
else
	d_p = 0;
	while minNotFound && k < length(table)
		d = abs(table(k) - r);
		d_p = abs(table(k_p) - r);
		if d_p < d
			minNotFound = false;
		end
		k = k + 1;
		k_p = k_p + 1;
	end
	k = k_p;
	d = d_p;
end

end

function [k, d] = getClosestIndexBackwards(table, r, k_p)
minNotFound = true;
k = min(k_p - 1, length(table));
if abs(table(k_p) - r) < abs(table(k) - r)
	k = k_p;
	d = abs(table(k_p) - r);
else
	while minNotFound && k < length(table)
		d = abs(table(k) - r);
		d_p = abs(table(k_p) - r);
		if d_p < d
			minNotFound = false;
		end
		k = k - 1;
		k_p = k_p - 1;
	end
	k = k_p;
	d = d_p;
end

end

function k = findMinVel(v_al, k_start)
minNotFound = true;
k = k_start+1;
k_next = k + 1;
while minNotFound && k_next < length(v_al)
	if v_al(k_next) > v_al(k) && v_al(k_start) > v_al(k)
		minNotFound = false;
	else
		k_start = k;
		k = k_next;
		k_next = k_next + 1;
	end
end
if k > length(v_al)
	k = length(v_al);
end
end

function [r, v_a, a, t, i, q] = forwardInt(r0, v0, a0, t0, v_al, v_max, a_max, dt, i_prev, table, qf_prev, r_final)
i = 2;
r(1) = r0;
v_a(1) = v0;
a(1) = a0;
t(1) = t0;
v(1) = 5;
q_prev = qf_prev;
while v_a(i-1) <= v_max% && r(i-1) < r_final%v(i-1)
	[q(i), d(i)] = getClosestIndex(table, r(i-1), q_prev);
	alen(i) = table(q(i));
	q_prev = q(i);
 	v(i) = v_al(q(i));
	a(i) = a_max;
	v_a(i) = v_a(i-1) + a(i)*dt;
	r(i) = r(i-1) + v_a(i)*dt;
	t(i) = t(i-1) + dt;
	i = i + 1;
end
i = i-1;
q(1) = q(2);
end

function [r2, v2, a2, q2, g, k] = backwardInt(table_index, target_position, lookup_table_positions, v_allowed, a_max, dt, v_max, q2, position, velocity)
if table_index ~= length(lookup_table_positions)-1
	% Set the initial conditions
	r2(1) = lookup_table_positions(table_index);
	a2(1) = 0;
	v2(1) = v_allowed(table_index);
	q_prev = length(lookup_table_positions);
	k_start = length(position)-1; % always start the first minimization at the furthest location
	g = 2;

	% Get the velocity of the previous velocity curve that is associated with
	% the closest position
	[v, k(1)] = getVelWithPosBackwards(velocity, position, r2(1), k_start);
	while abs(v2(g-1) - v) > 0.01 || r2(g-1) > target_position % may need to change the allowable range %r2(g-1) > lambda
		[q2(g), d(g)] = getClosestIndexBackwards(lookup_table_positions, r2(g-1), q_prev);
		q_prev = q2(g);

		a2(g) = -a_max; % Apply maximum decelleration in backwards time
		v2(g) = v2(g-1) + -a2(g-1)*dt; % Update teh velocity

		if v2(g) >= v_max % Saturate the velocity and acceleration if needed
			v2(g) = v_max;
			a2(g) = 0;
		end

		r2(g) = r2(g-1) + -v2(g)*dt; % Update the position
		[v, k(g)] = getVelWithPosBackwards(velocity, position, r2(g), k(g-1));

		g = g+1;	
	end
	q2(1) = q2(2);
else
	% Set the initial conditions
	r2(1) = lookup_table_positions(table_index);
	a2(1) = 0;
	v2(1) = 0;
	q_prev = length(lookup_table_positions);
	k_start = length(position)-1; % always start the first minimization at the furthest location
	g = 2;

	% Get the velocity of the previous velocity curve that is associated with
	% the closest position
	[v, k(1)] = getVelWithPosBackwards(velocity, position, r2(1), k_start);
	while abs(v2(g-1) - v) > 0.01 || r2(g-1) > target_position % may need to change the allowable range %r2(g-1) > lambda
		[q2(g), d(g)] = getClosestIndexBackwards(lookup_table_positions, r2(g-1), q_prev);
		q_prev = q2(g);

		a2(g) = -a_max; % Apply maximum decelleration in backwards time
		v2(g) = v2(g-1) + -a2(g-1)*dt; % Update teh velocity

		if v2(g) >= v_max % Saturate the velocity and acceleration if needed
			v2(g) = v_max;
			a2(g) = 0;
		end

		r2(g) = r2(g-1) + -v2(g)*dt; % Update the position
		[v, k(g)] = getVelWithPosBackwards(velocity, position, r2(g), k(g-1));

		g = g+1;	
	end
end
end

function [v, k] = getVelWithPosBackwards(velocity, position, r, k_start)
% Find the index in position that is closest to r
% Return the velocity associated with this position (at the same index)
minNotFound = true;
k = k_start; % - 1;
k_next = k - 1;
while minNotFound && k_next >= 1
	if abs(position(k_next)-r) > abs(position(k)-r) %&& abs(position(k_start)-r) > abs(position(k)-r)
		minNotFound = false;
	else
		k_start = k;
		k = k_next;
		k_next = k_next - 1;
	end
end
if k < 1
	k = 1;
end
v = velocity(k+1);
end
