clear;
close all;
%% Simulation Parameters
endtime = 14;
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
target_velocity = [0; 0; 0];

obs_pos = [1 1 8]';
target_start = obs_pos';

% Capturing Parameters
obs_radi = [0.5 0.5 0.25];
buffer = 0.75;
start_pos = [2,1,0];
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
	 0 0 0.1]; % moment of inertia matrix
m = 4; % mass - smaller mass helps performance
g = -9.8; % gravity

L = 0.25; % length of quad arm

c_T = 0.00000545; % no idea if this is the correct ball park, check this
c_Q = 0.0000002284; %1.5; % also not sure if this is the correct order of magnitude
gamma = [c_T, c_T, c_T, c_T;
		 0, L*c_T, 0, -L*c_T;
		 -L*c_T, 0, L*c_T, 0;
		 -c_Q, c_Q, -c_Q, c_Q];  
gammainv = inv(gamma);

A1c = 1; % check order of magnitude
A1s = 1; % check order of magnitude
dx = 0.058; % this should be roughly correct. Note that we drag I believe I have a speed cap
dy = 0.058; % drag in y
%% Motor Parameters
V_batt = 12; % volts 
% max rotor speed ~2000.0 rad/s (at 12v) (which translates to about 87N of thrust at
% max)
%% Sensor Parameters
gps_sigma = 0.02; % 2cm standard deviation
vicon_sigma = 0.0001; % 0.1mm standard deviation

imu_rate = controllerRate; %dt; %0.01; having the wrong imu rate can be terrible
R_dot_hat = [0 0 0; 0 0 0; 0 0 0];
R_hat = [1 0 0; 0 1 0; 0 0 1];

b_dot_hat = [0; 0; 0];
b_hat = [0; 0; 0];

bias_a = [0.05; 0.05; 0.05]; % bias of the accelerometer in newtons - can revise this number/make it different between runs
mu_a = 0; % mean of the accelerometer noise
sigma_a = 0.06; % standard deviation of the accelerometer noise

bias_b = 0; %[0.005; 0.005; 0.005]; % bias of the gyro. Can also be changed/made random between runs
mu_b = 0; % mean of the gyro noise
sigma_b = 0.15; % standard deviation of the rate measurements in rad/s

ka = 1;
kb = 1;

k_w = 1;
%% Initial Conditions
eulerAngles = zeros(3,1); % ZYX: yaw, roll, pitch
omega = zeros(3,1);
omegadot = zeros(3,1);

position = zeros(3,1); % this is position of the center of mass of the quad in the world frame
linearVel = zeros(3,1); % in the world frame
linearAcc = [0; 0; 0];%zeros(3,1); % in the world frame
linearJerk = [0;0;0];

r_hat = [0; 0; 0];

omega_motor_act = zeros(4,1);

rotMat = eul2rotm(eulerAngles', 'XYZ');

xw = [1; 0; 0]; % world coordinate x axis
yw = [0; 1; 0]; % world coordinate y axis
zw = [0; 0; 1]; % world coordinate z axis

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
		 
%% Path Following Constants
L = .5;

%% End of Constants
%% Trajectory Planning
j_max = 11; % max jerk
a_max = 15; % max acceleration
v_max = 20; % max velocity

position = start_pos';
%% Initial Conditions
rotDot = [0 0 0; 0 0 0; 0 0 0];
D = diag([dx, dy, 0]);
v_hat = [0; 0; 0];
a_hat = [0;0;0];
thrust = 0;

%% Path Planning
[pathCoefs, path, arc_len, T, N, B, curvature, torsion] = makeGeoPath(obs_pos', obs_radi, buffer, start_pos, tilt_angle, claw_length);
T = T';
N = N';
B = B';
[len, vel, acc, jerk, arcLenTable] = accSweep(0,0,0,0, path(1:20002, :), curvature);
%[r_s, v_s, a_s, j_s] = genMotionProfile2(arc_len, pathCoefs, 1, controllerRate, path); %genMotionProfile(arc_len, 0, 0, pathCoefs, ds, controllerRate);

psi_s = zeros(length(acc),1);
psi_dot_s = zeros(length(acc),1);
endtime = length(acc)*0.001 + 1;%*controllerRate - controllerRate;
%% Simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simCounter = 1;
controllerCounter = 1;
counter_t = 1;
endtime_mod = endtime+1;
madePath = 0;
path_index = 1;
path_index2 = 2;
profile_index = 2;
arc_len = [10 0 0]; % this should change in the loop. Prob a better way to do this
index = 0; % move this
len_traveled = 0;
s = 0;
r_s = [0 0; 0 0; 0 0];
e_R = [0 0 0];
for t = 0:dt:endtime
	tspan = [t, t+dt];

	%% Sensors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Position estimate will come from a "GPS" that will report at 10Hz
	% with a 2cm standard deviation
	% if using VICON, the update rate will be faster and sigma will be ~2
	% orders of magnitude smaller
	if(~using_vicon)
		if(mod(t, gps_rate) == 0)
			r_hat = normrnd(position, gps_sigma);
		end
	else
		if(mod(t, vicon_rate) == 0)
			r_hat = normrnd(position, vicon_sigma);
		end
	end
	
	% IMU complementary observer with gyro and accelerometer. magnetometer
	% is not used because in most practical cases it yields to much noise
	% to be effective
	% future improvements could involve using the vicon data to improve the
	% attitude estimate
	if(mod(t, imu_rate) == 0)
		eta_a = normrnd(mu_a, sigma_a);
		a_imu = rotMat' * (linearAcc - g*zw) + bias_a + eta_a; % the reading of the imu
		a_hat = rotMat * a_imu;
		
		eta_b = normrnd(mu_b, sigma_b);
		omega_imu = omega + bias_b + eta_b;

		% this filter assumes minimal up and down accelerations I believe
		alpha = (ka/g^2)*cross(rotMat'*zw, a_imu); % can add in vicon terms here

		% choosing not to use a matlab ode solver to solve the observer state
		% equations, because when implemented for a real time system, I will
		% not be using the matlab integration routines
		b_dot_hat_prev = b_dot_hat;
		b_dot_hat = kb*alpha;
		b_hat = b_hat + ((b_dot_hat + b_dot_hat_prev)/2) * imu_rate; % this is a simple trapezoidal intregration routine

		R_dot_hat_prev = R_dot_hat;
		R_dot_hat = (R_hat * hatmap(omega_imu - b_hat))- alpha;
		R_hat = R_hat + ((R_dot_hat + R_dot_hat_prev)/2) * imu_rate; % this is a simple trapezoidal intregration routine
		
		% can potentially improve performance by propogating the signal
		% with a linear time evolution in the time steps where I don't get
		% IMU data
	end
	

	%% Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	if(mod(t, controllerRate) == 0)
	if t <= 1
		u1_com = norm(-g*m);
		M = [0;0;0];
	else
	%% Path Following
% 	[b3_d, path_index, d, a] = NLGL(r_hat', path(1:20002, :)', L, path_index);
%  	path_index_hist(controllerCounter) = path_index;
% 	d_hist(controllerCounter) = d;
% 	b3_d_hist(:, controllerCounter) = b3_d;
	index = controllerCounter - 1/controllerRate;
	
%	path_index2 = getClosestPoint2(r_hat, path(1:20002,:)', path_index2);

	path_index2 = getClosestLocation(path_index2, path(1:20002), position);
	profile_index = getClosestLocation(profile_index, len, arcLenTable(path_index2));
	
	v_des = T(:, path_index2)*vel(profile_index);
	a_des = acc(profile_index)*T(:, path_index2) + vel(profile_index)^2*curvature(path_index2)*N(:, path_index2);
	if controllerCounter > 1
		kappa_prime = (curvature(path_index2)-curvature(path_index2-1))/0.0001;
		j_des = (jerk(profile_index) - curvature(path_index2)^2*vel(profile_index)^3)*T(:, path_index2) +(3*curvature(path_index2)*vel(profile_index)*acc(profile_index)+kappa_prime*vel(profile_index)^2)*N(:, path_index2) + curvature(path_index2)*torsion(path_index2)*vel(profile_index)^3*B(:, path_index2);
	else
		j_des = [0;0;0];
	end	
	
	e_r = dot((path(path_index,:)'-position),N(:, path_index))*(N(:, path_index)) + dot((path(path_index,:)'-position),B(:, path_index))*(B(path_index));
	e_v = v_des - linearVel;
	e_a = a_des - linearAcc; 
	e_j = j_des - linearJerk;
	
	e_j_hist(:, controllerCounter) = e_j;
	e_a_hist(:, controllerCounter) = e_a;
	e_r_hist(:, controllerCounter) = e_r;
	e_v_hist(:, controllerCounter) = e_v;
	
	if index == 1
		F_ext = [0;0;0];
	end
	a_com = 0*e_j + 0*m*e_a + 10*e_r + 3*e_v + m*a_des -g*m*zw + 0*F_ext; % ka = 2.5, kv = 2.5, kr = 12 works well with the external compensation and limits: v=1, a=1, j=3 (3:19pm, 6/29/20) .ka = 1, kv = 2.18, kr = 1 worked well before external force compensation (2:58pm 6/29/20)
	u1_com = norm(a_com);
	e1 = [cos(psi_s(index)); sin(psi_s(index)); 0];
	
	b3 = (a_com)/norm(a_com); 
	b2 = cross(b3, e1)/norm(cross(b3, e1));
	b1 = cross(b2, b3);
	
	Rd = [b1 b2 b3];
		
	h_omega = (m/norm(m*a_com)) * (j_des - (dot(b3, j_des) * b3));
	omega_des = [dot(-h_omega, b2); dot(h_omega, b1); dot(psi_dot_s(index) * zw, b3)]; % may want to use a different set of b's
	%omega_des = [0;0;0];
	e_R = veeMap(0.5*(Rd'*rotMat - rotMat'*Rd))';
	e_w = omega - rotMat' * Rd * omega_des;
	M = -0.75*e_R - 0.05*e_w; % -1.5, -1 worked well with the external compensation on 6/29 TODO: check the SO(3) control law condition
	
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
	path_index2_hist(controllerCounter) = path_index2;
	profile_index_hist(controllerCounter) = profile_index;
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
	
	anglesHist(:, simCounter) = eulerAngles; % make the history of all the angles for graphing

	% I could generate the differential equation for the rotation matrix,
	% but instead I just integrate to get angle, then use the matlab
	% command
	rotMat = eul2rotm(eulerAngles', 'XYZ'); % check this ordering

	rotMatHist(:, :, simCounter) = rotMat;	% get a history of all the rotation matricies
	
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
	F_ext = u(1) * ((D_ind + A_flap) * (rotMat' * linearVel)); % i think this may be applying force in the wrong directions
	F_ext_hist(:, simCounter) = F_ext;
	%F_ext = [0;0;0];
	%% Position Dynamics
	[~, y] = ode45(@(t_s, y) sumOfForces(rotMat, g, m, thrust, F_ext), tspan, linearVel); % should pass in g instead of 0
	linearVel = (y(size(y, 1), :))';

	[~, y] = ode45(@(t_s, y) simpleIntegral(linearVel, 1), tspan, position);
	position = (y(size(y, 1), :))';
	
	linearAcc = sumOfForces(rotMat, g, m, thrust, F_ext); % should pass in g instead of 0
	
	if position(3) <= 0 % put in the ground
		position(3) = 0;
		linearVel(3) = 0;
		linearAcc(3) = 0;
	end
	
	if(t ~= 0)
		in_bounds_hist(:, simCounter) = check_bounds(position, lab_x, lab_y, lab_z, t, in_bounds_hist(:, simCounter - 1));
	end
	
	if simCounter > 1
		linearJerk = (linearAcc - acc_hist(:, simCounter-1))/dt;
	else
		linearJerk = [0;0;0];
	end
	
	positionHist(:, simCounter) = position; % make the history of all the angles for graphing
	acc_hist(:, simCounter) = linearAcc;
	velHist(:, simCounter) = linearVel;
	j_hist(:, simCounter) = linearJerk;	
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
for w = 1:length(h)
	v1 = [positionHist(1,h(w)), positionHist(2,h(w)), positionHist(3,h(w))];
	v2 = [positionHist(1,h(w))+b3_com_hist(1,k(w)), positionHist(2,h(w))+b3_com_hist(2,k(w)), positionHist(3,h(w))+b3_com_hist(3,k(w))];
	v = [v1; v2];
	plot3(v(:,1), v(:,2), v(:,3), 'Color', 'r', 'MarkerSize', 3);
		
	v1 = [positionHist(1,h(w)), positionHist(2,h(w)), positionHist(3,h(w))];
	v2 = [positionHist(1,h(w))+rotMatHist(1,3,h(w)), positionHist(2,h(w))+rotMatHist(2,3,h(w)), positionHist(3,h(w))+rotMatHist(3,3,h(w))];
	v = [v1; v2];
	plot3(v(:,1), v(:,2), v(:,3), 'Color', 'b', 'MarkerSize', 3);
	
% 	v1 = [positionHist(1,h(w)), positionHist(2,h(w)), positionHist(3,h(w))];
% 	v2 = [positionHist(1,h(w))+b3_d_hist(1,k(w)), positionHist(2,h(w))+b3_d_hist(2,k(w)), positionHist(3,h(w))+b3_d_hist(3,k(w))];
% 	v = [v1; v2];
% 	plot3(v(:,1), v(:,2), v(:,3), 'Color', 'k', 'MarkerSize', 3);
	
end
grid on;
%title('position in 3d space');
[x_ell, y_ell, z_ell] = ellipsoid(obs_pos(1), obs_pos(2), obs_pos(3), obs_radi(1), obs_radi(2), obs_radi(3));
surf(x_ell, y_ell, z_ell);

axis equal; %([0 8 0 4 0 12])%[0 lab_x 0 lab_y 0 lab_z+5]); %equal; %([-30 30 -30 30 -30 30]);
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
%axis([0 450 -10 10]);
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

%title('Position Error');

figure; plot(1:controllerCounter-1, e_r_hist)
title('Velocity Error');

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

% subplot(3,1,3);
% plot(1:controllerCounter-1, a_com_hist);
% title('a commanded');

subplot(2,1,2);
plot((1:simCounter-1)/1000, acc_hist, 'lineWidth', 2.0);
legend('X Acceleration', 'Y Acceleration', 'Z Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Actual Acceleration');


figure; 
plot(1:length(velHist), velHist);
title('Lin vel hist');

function M = hatmap(vector)
	M = [0 -vector(3) vector(2);
		 vector(3) 0 -vector(1);
		 -vector(2) vector(1) 0];
end

% TODO: Make a better method
function [trajectory, psi_t] = genTrajectory(end_point, t, time_allowed, r_s, v_s, a_s, j_s, simCounter, dt, target_vel)
final_t = t + time_allowed;
% need a better method. this seems to be seeing numerical instability. i.e.
% A_mat is almost singular
A_mat = [1*t^7 1*t^6 1*t^5 1*t^4 1*t^3 1*t^2 1*t 1;
			 7*(t^6) 6*(t^5) 5*(t^4) 4*(t^3) 3*(t^2) 2*t 1 0;
			 42*(t^5) 30*(t^4) 20*(t^3) 12*(t^2) 6*t 2 0 0;
			 7*6*5*(t^4) 6*5*4*(t^3) 5*4*3*(t^2) 4*3*2*t 6 0 0 0;
			 1*final_t^7 1*final_t^6 1*final_t^5 1*final_t^4 1*final_t^3 1*final_t^2 1*final_t 1;
			 7*(final_t^6) 6*(final_t^5) 5*(final_t^4) 4*(final_t^3) 3*(final_t^2) 2*final_t 1 0;
			 42*(final_t^5) 30*(final_t^4) 20*(final_t^3) 12*(final_t^2) 6*final_t 2 0 0;
			 7*6*5*(final_t^4) 6*5*4*(final_t^3) 5*4*3*(final_t^2) 4*3*2*final_t 6 0 0 0];
		 
		 
	UX = [r_s; v_s; a_s; j_s; end_point; target_vel; 0; 0]; % since I don't have a jerk state, right now assuming that jerk always starts and ends at 0
	% assuming that the derivatives at the target are still 0
	coefsX = pinv(A_mat)*UX; % using PINV for numeric stability, but it does not yield correct results
	counter_t = simCounter;
	for n = t:dt:final_t
		trajectory(counter_t, 1) = coefsX(1)*n^7 + coefsX(2)*n^6 + coefsX(3)*n^5 + coefsX(4)*n^4 + coefsX(5)*n^3 + coefsX(6)*n^2 + coefsX(7)*n + coefsX(8); 
		trajectory(counter_t, 2) = 7*coefsX(1)*n^6 + 6*coefsX(2)*n^5 + 5*coefsX(3)*n^4 + 4*coefsX(4)*n^3 + 3*coefsX(5)*n^2 + 2*coefsX(6)*n + coefsX(7);
		trajectory(counter_t, 3) = 7*6*coefsX(1)*n^5 + 6*5*coefsX(2)*n^4 + 5*4*coefsX(3)*n^3 + 4*3*coefsX(4)*n^2 + 3*2*coefsX(5)*n + 2*coefsX(6);
		trajectory(counter_t, 4) = 7*6*5*coefsX(1)*n^4 + 6*5*4*coefsX(2)*n^3 + 5*4*3*coefsX(3)*n^2 + 4*3*2*coefsX(4)*n + 3*2*1*coefsX(5);

		psi_dot_t(counter_t) = 0;
		psi_t(counter_t) = 0;
		counter_t = counter_t + 1;	
	end
end

function [psi_t, psi_dot_t] = genPsiZero(t, time_allowed, simCounter, dt)
	counter_t = simCounter;
	final_t = t + time_allowed;
	for n = t:dt:final_t
		psi_dot_t(counter_t) = 0;
		psi_t(counter_t) = 0;
		counter_t = counter_t + 1;	
	end
end

function loc = getLocation(p_0, p_1, p_2, p_3, s)
	loc = (1-s)^3*p_0 + 3*(1-s)^2*s*p_1 + 3*(1-s)*s^2*p_2 + s^3*p_3;
end

function s = veeMap(R)
s = [R(3,2), R(1,3), R(2,1)];
end

function k = getClosestLocation(k_prev, path, r)
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
