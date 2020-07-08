clear;
close all;
%% Simulation Parameters
dt = 0.001; % simulation time step

controllerRate = 0.01; % interval at which the controller activates
plan_rate = 0.5;
gps_rate = 0.1; % 10Hz gps update
vicon_rate = 0.0025; %400hz update for the vicon system
motor_control_period = 1/500;
using_vicon = true;

lab_z = 12.2;
lab_y = 9.1;
lab_x = 9.1;

% for repeatability:
rng('default');

%% Target Craft Parameters
target_velocity = [0; 0; 0];	% velocity of the target

obs_pos = [1 1 8]';				% position of the target

% Capturing Parameters
obs_radi = [0.5 0.5 0.25];
buffer = 0.75;
start_pos = [3,3,0];
tilt_angle = pi/10;
claw_length = 0.4;
ds = 0.001;
%% State Machine Parameters
state = 0;
generated_trajectory = 0;
in_hover = 0;
position_for_hover = [0;0;0];
%% Quadrotor Parameters
I = [0.1 0 0;
	 0 0.1 0;
	 0 0 0.1];	% moment of inertia matrix
m = 4;			% mass - smaller mass helps performance
g = -9.8;		% gravity

L = 0.25;		% length of quad arm

c_T = 0.00000545;
c_Q = 0.0000002284; 
gamma = [c_T, c_T, c_T, c_T;
		 0, L*c_T, 0, -L*c_T;
		 -L*c_T, 0, L*c_T, 0;
		 -c_Q, c_Q, -c_Q, c_Q];  
gammainv = inv(gamma);

A1c = 1;
A1s = 1; 
dx = 0.058; % this should be roughly correct. Note that we drag I believe I have a speed cap
dy = 0.058; % drag in y
D = diag([dx, dy, 0]);
%% Motor Parameters
V_batt = 12; % volts 
% max rotor speed ~2000.0 rad/s (at 12v) (which translates to about 87N of thrust at max)
%% Sensor Parameters
gps_sigma = 0.02;			% 2cm standard deviation
vicon_sigma = 0.0001;		% 0.1mm standard deviation

imu_rate = controllerRate;
R_dot_hat = [0 0 0; 0 0 0; 0 0 0];	% Initial rotation matrix derivative estimate
R_hat = [1 0 0; 0 1 0; 0 0 1];		% Initial rotation matrix estimate

b_dot_hat = [0; 0; 0];
b_hat = [0; 0; 0];

bias_a = [0.000; 0.000; 0.000];	 % bias of the accelerometer in newtons - can revise this number/make it different between runs
mu_a = 0;						 % mean of the accelerometer noise
sigma_a = 0.0006;				 % standard deviation of the accelerometer noise

bias_b = 0;						 % bias of the gyro. Can also be changed/made random between runs
mu_b = 0;						 % mean of the gyro noise
sigma_b = 0.000015;				 % standard deviation of the rate measurements in rad/s

ka = 1;
kb = 1;

k_w = 1;
%% Initial Conditions
eulerAngles = zeros(3,1); % ZYX: yaw, roll, pitch
omega = zeros(3,1);
omegadot = zeros(3,1);

q_position = start_pos';		% this is position of the center of mass of the quad in the world frame
q_velocity = [0; 0; 0];			% in the world frame
q_acceleration = [0; 0; 0];		% in the world frame
q_jerk = [0;0;0];

r_hat = [0; 0; 0];				% initial estimate of the position

omega_motor_act = zeros(4,1);

rotMat = eul2rotm(eulerAngles', 'XYZ');

xw = [1; 0; 0]; % world coordinate x axis
yw = [0; 1; 0]; % world coordinate y axis
zw = [0; 0; 1]; % world coordinate z axis

a_hat = [0; 0; 0];
thrust = 0;

% "History" of the given variables
anglesHist = zeros(3,1);
positionHist = zeros(3,1);
rotMatHist = zeros(3, 3, 1);
velHist = zeros(3, 1);
omega_motor_hist = zeros(4,1);
moment_des_hist = zeros(3,1);
b1_hist = zeros(3, 1);
b2_hist = zeros(3, 1);
b3_hist = zeros(3, 1);
e_R_hist = zeros(3, 1);
R_com_hist = zeros(3, 3, 1);
b1_com_hist = zeros(3,1);
b2_com_hist = zeros(3,1);
b3_com_hist = zeros(3,1);
in_bounds_hist = [true; true; true];
target_position_hist = obs_pos;

r_effort = zeros(3, 1);
v_effort = zeros(3, 1);

%% Gains
k_motor = 0.00001;
k_motor_ff = 0.006;

kp_attitude = 19.5;
kd_attitude = 6;

kp_thrust = [35.5 0 0;
			 0 35.5 0;
			 0 0 35.5];
kd_thrust = 75.5;

%% Trajectory Planning
j_max = 11; % max jerk
a_max = 15; % max acceleration
v_max = 20; % max velocity

%% Path Planning
[pathCoefs, path, arc_len, T, N, B, curvature, torsion, lookup_table] = makeGeoPath(obs_pos', obs_radi, buffer, start_pos, tilt_angle, claw_length);
T = T';
N = N';
B = B';
[len, vel, acc, jerk] = optimizeSplineTrajectory(controllerRate, arc_len(1)+arc_len(2));

psi_s = zeros(length(acc),1); % Right now, I don't use yaw
psi_dot_s = zeros(length(acc),1);
endtime = length(acc)*controllerRate - controllerRate + 1;
%% Simulation Params %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simCounter = 1;
controllerCounter = 1;
endtime_mod = endtime+1;

geo_path_index = 2;
profile_index = 2;

%% Simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for t = 0:dt:endtime
	tspan = [t, t+dt];

	%% Sensors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Position estimate will come from a "GPS" that will report at 10Hz
	% with a 2cm standard deviation
	% if using VICON, the update rate will be faster and sigma will be ~2
	% orders of magnitude smaller
	if(~using_vicon)
		if(mod(t, gps_rate) == 0)
			r_hat = normrnd(q_position, gps_sigma);
		end
	else
		if(mod(t, vicon_rate) == 0)
			r_hat = normrnd(q_position, vicon_sigma);
		end
	end
	
	% IMU complementary observer with gyro and accelerometer. magnetometer
	% is not used because in most practical cases it yields to much noise
	% to be effective
	% future improvements could involve using the vicon data to improve the
	% attitude estimate
	if(mod(t, imu_rate) == 0)
		eta_a = normrnd(mu_a, sigma_a);
		a_imu = rotMat' * (q_acceleration - g*zw) + bias_a + eta_a; % the reading of the imu
		a_hat = rotMat * a_imu;
		
		eta_b = normrnd(mu_b, sigma_b);
		omega_imu = omega + bias_b + eta_b;

		% this filter assumes minimal up and down accelerations I believe
		alpha = (ka/g^2)*cross(rotMat'*zw, a_imu); % can add in vicon terms here

		b_dot_hat_prev = b_dot_hat;
		b_dot_hat = kb*alpha;
		b_hat = b_hat + ((b_dot_hat + b_dot_hat_prev)/2) * imu_rate; % this is a simple trapezoidal intregration routine

		R_dot_hat_prev = R_dot_hat;
		R_dot_hat = (R_hat * hatmap(omega_imu - b_hat))- alpha;
		R_hat = R_hat + ((R_dot_hat + R_dot_hat_prev)/2) * imu_rate; % this is a simple trapezoidal intregration routine
	end
	

	%% Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	if(mod(t, controllerRate) == 0)
	if t <= 1
		u1_com = norm(-g*m);
		M = [0;0;0];
	else
	%% Path Following
	index = controllerCounter - 1/controllerRate;

	% Step 1: Get the location on the path that is closests to the current
	% location
	geo_path_index = getClosestLocation(geo_path_index, path, q_position);
	
	% Step 2: Get the corresponding arc length of the path
	profile_index = getClosestLocationProfile(profile_index, len, lookup_table(geo_path_index));
	
	% Step 3: Get the velocity, acceleration and jerk that correspond to
	% that arc length
	v_des = T(:, geo_path_index)*vel(profile_index);
	a_des = acc(profile_index)*T(:, geo_path_index) + vel(profile_index)^2*curvature(geo_path_index)*N(:, geo_path_index);
	if controllerCounter > 1
		kappa_prime = (curvature(geo_path_index)-curvature(geo_path_index-1))/0.0001;
		j_des = (jerk(profile_index) - curvature(geo_path_index)^2*vel(profile_index)^3)*T(:, geo_path_index) +(3*curvature(geo_path_index)*vel(profile_index)*acc(profile_index)+kappa_prime*vel(profile_index)^2)*N(:, geo_path_index) + curvature(geo_path_index)*torsion(geo_path_index)*vel(profile_index)^3*B(:, geo_path_index);
	else
		j_des = [0;0;0];
	end
	
	e_r = dot((path(geo_path_index,:)'-r_hat),N(:, geo_path_index))*(N(:, geo_path_index)) + dot((path(geo_path_index,:)'-r_hat),B(:, geo_path_index))*(B(geo_path_index));
	e_v = v_des - q_velocity;
	
	e_r_hist(:, controllerCounter) = e_r;
	e_v_hist(:, controllerCounter) = e_v;
	
	a_com = 6.25*e_r + 4.5*e_v + m*a_des -g*m*zw; 
	u1_com = norm(a_com);
	e1 = [cos(psi_s(profile_index)); sin(psi_s(profile_index)); 0];
	
	b3 = (a_com)/norm(a_com); 
	b2 = cross(b3, e1)/norm(cross(b3, e1));
	b1 = cross(b2, b3);
	
	Rd = [b1 b2 b3];
		
	h_omega = (m/norm(m*a_com)) * (j_des - (dot(b3, j_des) * b3));
	omega_des = [dot(-h_omega, b2); dot(h_omega, b1); dot(psi_dot_s(profile_index) * zw, b3)];
	
	e_R = veeMap(0.5*(Rd'*R_hat - R_hat'*Rd))';
	e_w = omega - R_hat' * Rd * omega_des;
	M = -2.0*e_R - 2.75*e_w; 
	
	omega_des_hist(:, controllerCounter) = omega_des;
	j_des_hist(:, controllerCounter) = j_des;
	e_R_hist(:, controllerCounter) = e_R;
	e_w_hist(:, controllerCounter) = e_w;
	a_des_hist(:, controllerCounter) = a_des;
	v_des_hist(:, controllerCounter) = v_des;
	a_com_hist(:, controllerCounter) = a_com;
	b3_com_hist(:, controllerCounter) = b3;
	u1_com_hist(:, controllerCounter) = u1_com;
	M_hist(:, controllerCounter) = M;
	geo_path_index_hist(controllerCounter) = geo_path_index;
 	profile_index_hist(controllerCounter) = profile_index;
	lookup_hist(controllerCounter) = lookup_table(geo_path_index);
	
	end
	u_des = [u1_com; M]; % Thrust mag, moment around X, moment around Y, moment around Z

	omega_motor_des = gammainv * (u_des);
	omega_motor_des = sign(omega_motor_des) .* sqrt(abs(omega_motor_des));

 	omega_motor_des(1) = 1*(omega_motor_des(1));
 	omega_motor_des(2) = -1*(omega_motor_des(2));
 	omega_motor_des(3) = 1*(omega_motor_des(3));
 	omega_motor_des(4) = -1*(omega_motor_des(4));

	omega_motor_des_hist(:, controllerCounter) = omega_motor_des;
	controllerCounter = controllerCounter + 1;

	end
	if (mod(t, motor_control_period) == 0)
		Vin = k_motor*(omega_motor_des - omega_motor_act) + k_motor_ff*omega_motor_des; % motor controller
		
		% saturate Vin to the motor battery
		for n = 1:4
			Vin(n) = min([Vin(n), V_batt]);
			Vin(n) = max([Vin(n), -V_batt]);
		end
		Vin_hist(:, simCounter) = Vin;
	end
	
	
	%% Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Motor Dynamics
 	[~, y] = ode45(@(t_s, omega_motor_act) motorDynamics(Vin, omega_motor_act), tspan, omega_motor_act);
 	omega_motor_act = y(size(y, 1), :)';
 	omega_motor_hist(:, simCounter) = omega_motor_act;

	u = gamma * omega_motor_act.^2;
	moments = u(2:4);
	thrust = u(1) * rotMat*zw;
	thrust_hist(simCounter) = u(1);
	moments_hist(:,simCounter) = u(2:4);
	%% Attitude Dynamics
	[~, y] = ode45(@(t_s, y) attitudeChange(omega, moments, I), tspan,  [eulerAngles; omega]); % is omega or omegadot the initial condition? should be omega
	omega = (y(size(y, 1), [4 5 6]))';
	eulerAngles = (y(size(y, 1), [1 2 3]))';
	
	anglesHist(:, simCounter) = eulerAngles;

	rotMat = eul2rotm(eulerAngles', 'XYZ');

	rotMatHist(:, :, simCounter) = rotMat;
	
	%% Rotor Flapping and Drag Dynamics
	% this will need to be factored into the position dynamics
	% this flapping model is assuming: 
	% - the vehicle's velocity is small compared to the tip velocity
	% - the wind speed is negligble 
	% - the vertical distance from the rotor to the CoM is very small 
	% - the induced drag acts as a torsional spring
	
	A_flap = 1/(omega_motor_des(1)) * [A1c, -A1s, 0; A1s, A1c, 0; 0, 0, 0];
	D_ind = diag([dx, dy, 0]);
	D = A_flap + D_ind;
	F_ext = u(1) * ((D_ind + A_flap) * (rotMat' * q_velocity));
	F_ext_hist(:, simCounter) = F_ext;

	%% Position Dynamics
	[~, y] = ode45(@(t_s, y) sumOfForces(rotMat, g, m, thrust, F_ext), tspan, q_velocity);
	q_velocity = (y(size(y, 1), :))';

	[~, y] = ode45(@(t_s, y) simpleIntegral(q_velocity, 1), tspan, q_position);
	q_position = (y(size(y, 1), :))';
	
	q_acceleration = sumOfForces(rotMat, g, m, thrust, F_ext);
	
	if q_position(3) <= 0 % effect of the ground
		q_position(3) = 0;
		q_velocity(3) = 0;
		q_acceleration(3) = 0;
	end
	
	if(t ~= 0)
		in_bounds_hist(:, simCounter) = check_bounds(q_position, lab_x, lab_y, lab_z, t, in_bounds_hist(:, simCounter - 1));
	end
	
	if simCounter > 1
		q_jerk = (q_acceleration - acc_hist(:, simCounter-1))/dt;
	else
		q_jerk = [0;0;0];
	end
	
	positionHist(:, simCounter) = q_position; 
	acc_hist(:, simCounter) = q_acceleration;
	velHist(:, simCounter) = q_velocity;
	j_hist(:, simCounter) = q_jerk;	
	%% Update the target craft
	obs_pos = obs_pos + target_velocity * dt;
	target_position_hist(:, simCounter) = obs_pos;
	
	simCounter = simCounter + 1;
	endtime_mod = t;
	
end
%% Visualization
n_sim = 0:dt:endtime_mod;
n_controller = 0:controllerRate:endtime_mod;
n_sensors = 0:imu_rate:endtime_mod;
n_motor = 0:motor_control_period:endtime_mod;
n_plan = 0:plan_rate:endtime_mod;
k_simCounter = 1:simCounter-1;
k_plan = 1:(plan_rate/dt):simCounter-1;

figure;
subplot(3, 1, 1);
plot(n_sim, anglesHist(1, k_simCounter));
title('Yaw of Craft vs. Time');

subplot(3,1,2);
plot(n_sim, anglesHist(2, k_simCounter));
title('Pitch of Craft vs Time');

subplot(3,1,3);
plot(n_sim, anglesHist(3, k_simCounter));
title('Roll of Craft vs Time');

figure;
subplot(3, 1, 1);
plot(n_sim, positionHist(1, k_simCounter));
hold on
plot(n_sim, target_position_hist(1, k_simCounter));
hold off
legend('q pos', 't pos');
title('x of Craft vs. Time');
grid on;

subplot(3,1,2);
plot(n_sim, positionHist(2, k_simCounter));
hold on
plot(n_sim, target_position_hist(2, k_simCounter));
hold off
legend('q pos', 't pos');
title('y of Craft vs Time');
grid on;

subplot(3,1,3);
plot(n_sim, positionHist(3, k_simCounter));
hold on
plot(n_sim, target_position_hist(3, k_simCounter));
hold off
legend('q pos', 't pos');
title('z of Craft vs Time');
grid on;

figure;
plot3(positionHist(1, :), positionHist(2, :), positionHist(3, :), '-o','Color', 'm', 'MarkerSize', 3);
hold on;
plot3(path(:,1), path(:,2), path(:,3), '--');
k = 1:20:controllerCounter-1;
h = 1:200:simCounter-1;
% for w = 1:length(h)
% 	v1 = [positionHist(1,h(w)), positionHist(2,h(w)), positionHist(3,h(w))];
% 	v2 = [positionHist(1,h(w))+b3_com_hist(1,k(w)), positionHist(2,h(w))+b3_com_hist(2,k(w)), positionHist(3,h(w))+b3_com_hist(3,k(w))];
% 	v = [v1; v2];
% 	plot3(v(:,1), v(:,2), v(:,3), 'Color', 'r', 'MarkerSize', 3);
% 	
% 	v1 = [positionHist(1,h(w)), positionHist(2,h(w)), positionHist(3,h(w))];
% 	v2 = [positionHist(1,h(w))+rotMatHist(1,3,h(w)), positionHist(2,h(w))+rotMatHist(2,3,h(w)), positionHist(3,h(w))+rotMatHist(3,3,h(w))];
% 	v = [v1; v2];
% 	plot3(v(:,1), v(:,2), v(:,3), 'Color', 'b', 'MarkerSize', 3);
% 	
% 	% 	v1 = [positionHist(1,h(w)), positionHist(2,h(w)), positionHist(3,h(w))];
% 	% 	v2 = [positionHist(1,h(w))+b3_d_hist(1,k(w)), positionHist(2,h(w))+b3_d_hist(2,k(w)), positionHist(3,h(w))+b3_d_hist(3,k(w))];
% 	% 	v = [v1; v2];
% 	% 	plot3(v(:,1), v(:,2), v(:,3), 'Color', 'k', 'MarkerSize', 3);
% 	%
% end
grid on;
%title('position in 3d space');
[x_ell, y_ell, z_ell] = ellipsoid(obs_pos(1), obs_pos(2), obs_pos(3), obs_radi(1), obs_radi(2), obs_radi(3));
surf(x_ell, y_ell, z_ell);

axis equal;
xlabel('Position in X');
ylabel('Position in Y');
zlabel('Position in Z');
hold off;
nC = 1:controllerCounter -1;

figure; 
plot(n_controller, u1_com_hist, 'lineWidth', 2.0)
%title('Commanded Thrust and Actual Thrust');
hold on;
plot(0:dt:endtime_mod, thrust_hist, 'lineWidth', 2.0)
hold off
axis([0 endtime 0 70]);
legend('Commanded Thrust', 'Actual Thrust');
xlabel('Time (s)');
ylabel('Force (N)');

figure; plot(1:controllerCounter-1, M_hist(2,:));
legend('y');
title('Moments Commanded');

figure; plot(1:controllerCounter-1, omega_des_hist)
title('Omega_des');

figure; plot(1:controllerCounter-1, e_R_hist)
title('Rotation error');

figure; plot(1:controllerCounter-1, e_w_hist)
title('Omega error')

figure;
for k = 1:simCounter-1
	magV(k) = norm(velHist(:,k));
	magA(k) = norm(acc_hist(:,k));
end
plot(1:k, magV);
title('Mag of Linear Vel');

figure;
plot(1:k, magA);
title('Mag of Linear Acc');


figure; plot(((1:controllerCounter-1)-1)/100, e_v_hist, 'lineWidth', 2.0)
xlabel('Time (s)');
ylabel('Distance (m)');
legend('Error in X', 'Error in Y', 'Error in Z');
title('Velocity Error');

figure; plot(1:controllerCounter-1, e_r_hist)
title('Position Error');

figure; plot(1:controllerCounter-1, e_R_hist)
title('Rotation Error');

figure; plot(1:controllerCounter-1, e_w_hist)
title('Angular Velocity Error');

figure; plot(1:controllerCounter-1, M_hist)
title('Applied Moments');

figure;
subplot(2,1,1);
plot((1:controllerCounter-1)/100, a_des_hist, 'lineWidth', 2.0);
legend('X Acceleration', 'Y Acceleration', 'Z Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Desired Acceelration');

subplot(2,1,2);
plot((1:simCounter-1)/1000, acc_hist, 'lineWidth', 2.0);
legend('X Acceleration', 'Y Acceleration', 'Z Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Actual Acceleration');

%% Helper Functions %%%%%%%
function M = hatmap(vector)
	M = [0 -vector(3) vector(2);
		 vector(3) 0 -vector(1);
		 -vector(2) vector(1) 0];
end

function s = veeMap(R)
s = [R(3,2), R(1,3), R(2,1)];
end

function k = getClosestLocation(k_prev, path, r)
k = k_prev;
if k+1 < length(path)
	if norm(path(k,:)-r) > norm(path(k+1,:)-r)
		minNotFound = true;
		k = k+1;
		while minNotFound && k+1 < length(path)
			if norm(path(k,:)-r) < norm(path(k+1,:)-r)
				minNotFound = false;
			else
				k = k+1;
			end
		end
	end
else
	k = length(path);
end
end

function k = getClosestLocationProfile(k_prev, path, r)
k = k_prev;
if k+1 < length(path)
	if norm(path(k)-r) > norm(path(k+1)-r)
		minNotFound = true;
		k = k+1;
		while minNotFound && k+1 < length(path)
			if norm(path(k)-r) < norm(path(k+1)-r)
				minNotFound = false;
			else
				k = k+1;
			end
		end
	end
else
	k = length(path);
end
end
