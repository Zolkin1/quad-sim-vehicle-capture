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
linearAcc = zeros(3,1); % in the world frame

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

kp_attitude = [19.5 0 0;
			   0 19.5 0;
			   0 0 19.5];
kd_attitude = [6 0 0;
			   0 6 0;
			   0 0 6];

kp_thrust = [35.5 0 0;
			 0 35.5 0;
			 0 0 35.5];
kd_thrust = [75.5 0 0; % used to be 35.5 and that worked well (5/25)
			 0 75.5 0;
			 0 0 75.5];

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

%% Simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simCounter = 1;
controllerCounter = 1;
counter_t = 1;
endtime_mod = endtime;
madePath = 0;

arc_len = [10 0 0]; % this should change in the loop. Prob a better way to do this
index = 0; % move this
len_traveled = 0;
s = 0;
r_s = [0 0; 0 0; 0 0];
for t = 0:dt:endtime
	if(controllerCounter < length(r_s)) %len_traveled <= (arc_len(1) + arc_len(2) + arc_len(3))-0.1) %controllerCounter < length(r_s))
		if simCounter > 2
			len_traveled = len_traveled + sqrt(((positionHist(1, simCounter-1)-positionHist(1,simCounter-2)))^2 + ((positionHist(2, simCounter-1)-positionHist(2,simCounter-2)))^2 + ((positionHist(3, simCounter-1)-positionHist(3,simCounter-2)))^2);
		end
	tspan = [t, t+dt];
	%% State Machine %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% do a switch statement on the state variable. I will need to make the
	% trajectory planning slightly different so I can put it in a function,
	% and so I can specify the amount of time that I want it to take to
	% complete an action.
	% The trajectory planning will also need to handle starting with an
	% arbitrary position (and hopefully accel, vel, and jerk)
	% here is a time based state machine switcher:
	
	state = 4;
	
	if(state == 4) % distance based tracking
		if (madePath == 0)
			madePath = 1;
	
			[pathCoefs, path, arc_len] = makeGeoPath(obs_pos', obs_radi, buffer, start_pos, tilt_angle, claw_length);
			[r_s, v_s, a_s, j_s] = genMotionProfile2(arc_len, pathCoefs, 1, controllerRate, path); %genMotionProfile(arc_len, 0, 0, pathCoefs, ds, controllerRate);
			psi_s = zeros(length(r_s),1);
			psi_dot_s = zeros(length(r_s),1);
		end
	end
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
 	index = controllerCounter;

	index_hist(controllerCounter) = index;
	r_s_used_hist(:, controllerCounter) = r_s(:, index);
	v_s_used_hist(:, controllerCounter) = v_s(:, index);
	a_s_used_hist(:, controllerCounter) = a_s(:, index);
	
	%% Differentially Flat Calculations
	
	% Calculate the rotation and angular velocity from the trajectory
	% inputs
	e1 = [cos(psi_s(index)); sin(psi_s(index)); 0];
	
	% Create the body frame axis from the trajectory
	b3 = (a_s(:, index) - g*zw)/(norm(a_s(:, index) - g*zw)); % check if we should be adding or subtracting that gravity term
	b2 = cross(b3, e1)/norm(cross(b3, e1));
	b1 = cross(b2, b3);
	b1_hist(:, controllerCounter) = b1;
	b2_hist(:, controllerCounter) = b2;
	b3_hist(:, controllerCounter) = b3;

	% Make the thrust from the trajectory
	u1_t = norm(-m*g*zw + m*a_s(:, index));

	% Calculate the omega_des from the trajectory
	h_omega = (m/u1_t) * (j_s(:, index) - dot(b3, j_s(:, index)) * b3);
	omega_des = [dot(-h_omega, b2); dot(h_omega, b1); dot(psi_dot_s(index) * zw, b3)];
	omega_des_hist(:, controllerCounter) = omega_des;
	
	r_des = r_s(:, index);
	vel_des = v_s(:, index);
	acc_des = a_s(:, index);
	
	b1_des = b1;
	
	% The controller takes: position, velocity, acceleration, rotation
	% (euler angles), angular velocity
	
	%right now we always look to the next point to calculate the error,
	%instead of using the closest point or something like that
	e_r = r_hat - r_des;
	e_v = linearVel - vel_des;
	e_r_hist(:, controllerCounter) = e_r;
	e_v_hist(:, controllerCounter) = e_v;
	
	b3_com =  (-kp_thrust*e_r - kd_thrust*e_v - m*g*zw + m*acc_des)/(norm(-kp_thrust*e_r - kd_thrust*e_v - m*g*zw + m*acc_des));
	b3_com_hist(:, controllerCounter) = b3_com;
	
	b2_com = cross(b3_com, b1_des)/(norm(cross(b3_com, b1_des)));
	b2_com_hist(:, controllerCounter) = b2_com;
	
	R_com = [cross(b2_com, b3_com), b2_com, b3_com];
	b1_com_hist(:, controllerCounter) = cross(b2_com, b3_com);
	R_com_hist(:, :, controllerCounter) = R_com;
	
	u1_com = dot(-kp_thrust*e_r -kd_thrust*e_v -m*g*zw + m*acc_des, R_hat*zw); % Can still add in the non-linear terms
	u1_com_hist(controllerCounter) = u1_com;
		
	e_R_temp = 0.5 * ((R_com' * R_hat) - (R_hat' * R_com));	% rotation matrix error (on SO(3))
	e_R = veemap(e_R_temp);	% apply the vee map to the rotation matrix error (map from SO(3) to R^3)
	e_R_hist(:, controllerCounter) = e_R;
	
	e_Omega = omega - R_hat' * R_com * omega_des;	% get the error on the rotation speed
	e_Omega_hist(:, controllerCounter) = e_Omega;
	
	% TODO: add the last three terms to the desired moments calc
	moments_com = (kp_attitude * e_R) - (kd_attitude * e_Omega); % get the desired moments. this does not have the full non-linear terms right now
	moments_com_hist(:, controllerCounter) = moments_com;

	u_des = [u1_com; moments_com]; % Thrust mag, moment around X, moment around Y, moment around Z (recently changed this order)

	omega_motor_des = gammainv * (u_des);
	omega_motor_des = sign(omega_motor_des) .* sqrt(abs(omega_motor_des));

 	omega_motor_des(1) = 1*(omega_motor_des(1));
 	omega_motor_des(2) = -1*(omega_motor_des(2));
 	omega_motor_des(3) = 1*(omega_motor_des(3));
 	omega_motor_des(4) = -1*(omega_motor_des(4));

	omega_motor_des_hist(:, simCounter) = omega_motor_des;
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
	thrust = u(1) * zw;
	thrust_hist(simCounter) = u(1);
	
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
	
	%% Position Dynamics
	[~, y] = ode45(@(t_s, y) sumOfForces(rotMat, g, m, thrust, F_ext), tspan, linearVel); 
	linearVel = (y(size(y, 1), :))';
	velHist(:, simCounter) = linearVel;

	[~, y] = ode45(@(t_s, y) simpleIntegral(linearVel, 1), tspan, position);
	position = (y(size(y, 1), :))';
	
	linearAcc = sumOfForces(rotMat, g, m, thrust, F_ext);
	acc_hist(:, simCounter) = linearAcc;
	
	if position(3) < 0 % put in the ground
		position(3) = 0;
		linearVel(3) = 0;
	end
	
	if(t ~= 0)
		in_bounds_hist(:, simCounter) = check_bounds(position, lab_x, lab_y, lab_z, t, in_bounds_hist(:, simCounter - 1));
	end
	positionHist(:, simCounter) = position; % make the history of all the angles for graphing
	
	%% Update the target craft
	obs_pos = obs_pos + target_velocity * dt;
	target_position_hist(:, simCounter) = obs_pos;
	
	simCounter = simCounter + 1;
	endtime_mod = t;
	end
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
plot(n_controller, r_s_used_hist(1,:));%k_simCounter));
hold off
legend('q pos', 't pos', 'traj x');
title('x of Craft vs. Time');

subplot(3,1,2);
plot(n_sim, positionHist(2, k_simCounter));
hold on
plot(n_sim, target_position_hist(2, k_simCounter));
plot(n_controller, r_s_used_hist(2,:));%k_simCounter));
hold off
legend('q pos', 't pos', 'traj y');
title('y of Craft vs Time');


subplot(3,1,3);
plot(n_sim, positionHist(3, k_simCounter));
hold on
plot(n_sim, target_position_hist(3, k_simCounter));
plot(n_controller, r_s_used_hist(3,:));
hold off
legend('q pos', 't pos', 'traj z');
title('z of Craft vs Time');

figure;
plot3(positionHist(1, :), positionHist(2, :), positionHist(3, :), '-o','Color', 'm', 'MarkerSize', 7);
hold on;
grid on;
title('position in 3d space');
plot3(r_s(1, :), r_s(2, :), r_s(3, :), '--'); 
[x_ell, y_ell, z_ell] = ellipsoid(obs_pos(1), obs_pos(2), obs_pos(3), obs_radi(1), obs_radi(2), obs_radi(3));
surf(x_ell, y_ell, z_ell);

axis([0 lab_x 0 lab_y 0 lab_z+5]); %equal; %([-30 30 -30 30 -30 30]);
xlabel('X Axis');
ylabel('Y Axis');
zlabel('Z Axis');
hold off;
nC = 1:controllerCounter -1;
figure; plot(nC, e_r_hist)
legend('x error', 'y error', 'z error')

figure;
n = 1:1:simCounter-1;
n_sim = 1:length(r_s);
plot(n_sim, r_s(1,:));%k_simCounter));
hold on;
plot(n_sim, v_s(1,:));%k_simCounter));
plot(n_sim, a_s(1,:));%k_simCounter));
plot(n_sim, j_s(1,:));%k_simCounter));
hold off;
legend('xr', 'xv', 'xa', 'xj')
%axis([0 2 -30 30]);

figure;
plot(n_sim, r_s(2,:));%k_simCounter));
hold on;
plot(n_sim, v_s(2,:));%k_simCounter));
plot(n_sim, a_s(2,:));%k_simCounter));
plot(n_sim, j_s(2,:));%k_simCounter));
hold off;
legend('yr', 'yv', 'ya', 'yj')
%axis([0 2 -30 30]);

figure;
plot(n_sim, r_s(3,:));%k_simCounter));
hold on;
plot(n_sim, v_s(3,:));%k_simCounter));
plot(n_sim, a_s(3,:));%k_simCounter));
plot(n_sim, j_s(3,:));%k_simCounter));
hold off;
legend('zr', 'zv', 'za', 'zj')
%axis([0 2 -30 30]);

figure; 
plot(n_controller, u1_com_hist)
title('Commanded Thrust and Actual Thrust');
hold on;
plot(0:dt:endtime_mod, thrust_hist)
hold off
axis([0 endtime -70 70]);

function zeta = veemap(matrix)
	zeta = [matrix(2, 3); -matrix(1, 3); matrix(1, 2)];
end

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