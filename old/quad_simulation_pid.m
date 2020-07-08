clear all;
close all;
%% Next: trajectory planner, then IMU. That way we can prove everything else before I add the IMU error to the model
%% Simulation Parameters
endtime = 10;
dt = 0.001; %0.001; % simulation time step
controllerRate = dt;%0.01; % interval at which the controller activates
gps_rate = 0.1; % 10Hz gps update
vicon_rate = 0.0025; %400hz update for the vicon system
motor_control_freq = dt; %1/500;
using_vicon = true;

% for repeatability:
rng('default');

%% State Machine Parameters
state = 0;
%trajectory_freq = 1; % update the path once/second when in trajectory generation mode
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
% TODO: Get more realistic motor params so the battery voltage is more
% normal
% motor time constant should be somewhere around 20. Which it is
b = 0.05; % with simplified motor model, I am not using these anymore
J = 0.01;
R = 5;
k_torque = 0.06; % still needs modification
V_batt = 12; % volts 
% max rotor speed ~2000.0 rad/s (at 12v) (which translates to about 87N of thrust at
% max)
%% Sensor Parameters
gps_sigma = 0.02; % 2cm standard deviation
vicon_sigma = 0.0001; % 0.1mm standard deviation

imu_rate =  controllerRate; %dt; %0.01; having the wrong imu rate can be terrible
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

ka = 1; % these need to be tuned
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

e_r = [0;0;0];

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

r_effort = zeros(3, 1);
v_effort = zeros(3, 1);
%% Gains
k_motor = 0.00001; %1	% probably need to retune after rotor flapping and drag are added in. actually it should only depend on the motor model which wont change unless I add in rotor-air drag at that level
k_motor_ff = 0.006; %4.1 see above - probably need to retune

kp_attitude = [.1 0 0;
			   0 .1 0;
			   0 0 .1];
kd_attitude = [6 0 0;
			   0 6 0;
			   0 0 6];

kp_thrust = [1 0 0;
			 0 1 0;
			 0 0 1];
kd_thrust = [75.5 0 0; % used to be 35.5 and that worked well (5/25)
			 0 75.5 0;
			 0 0 75.5];

%% End of Constants
%% Trajectory Planning
% The trajectory will give the x,y,z (and up to 4 derivatives) and yaw (up to 2 derivatives) of a dynamically feasible
% trajectory. Thus it will have a continuous 4th derivative for position.

j_max = 50;
a_max = 10;
v_max = 20;

start_point = [30; 30; 40];
end_point = [8; 8; 8]; % if the target location is too far away for the time given, the jerk will get very large and cause the aircraft to become unstable
% i wouldn't do anything with a jerk higher than 12.

jerk_t(:, 1) = [0; 0; 0];
acc_t(:, 1) = [0; 0; 0];
vel_t(:, 1) = [0; 0; 0];
r_t(:, 1) = [0; 0; 0];
psi_t(1) = 0;
psi_dot_t(1) = 0;
	
%% Simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simCounter = 1;
controllerCounter = 1;
thrust = 0; % need to move this
rotDot = [0 0 0; 0 0 0; 0 0 0]; % need to move this
D = diag([dx, dy, 0]); % move this. also need a better estimate for the drag constants
v_hat = [0; 0; 0]; % move this
a_hat = [0;0;0]; % move this


position = start_point;
for t = 0:dt:endtime
	tspan = [t, t+dt];
	%% State Machine %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% do a switch statement on the state variable. I will need to make the
	% trajectory planning slightly different so I can put it in a function,
	% and so I can specify the amount of time that I want it to take to
	% complete an action.
	% The trajectory planning will also need to handle starting with an
	% arbitrary position (and hopefully accel, vel, and jerk)
	% here is a time based state machine switcher:
	if (t < 1)
		state = 1;
	else
		state = 2;
		time_for_trajectory = 9;
		end_point = [0;0;5];
	end
% 		if (t < 3)
% 		state = 0;
% 	elseif (t < 8)
% 		state = 2;
% 		time_for_trajectory = 5; % the time allowed to track to the given trajectory
% 		end_point = [5, 5, 5];
% 	elseif (t < 10)
%  		state = 1;
% 	elseif (t < 13)
% 		state = 2;
% 		end_point = [10; 5; 7];
% 		time_for_trajectory = 3; % the time allowed to track to the given trajectory
% 	elseif (t < 15)
% 		state = 1;
% 	elseif (t < 19)
% 		state = 2;
% 		end_point = [10; 0; 3];
% 		time_for_trajectory = 4; % the time allowed to track to the given trajectory
% 	end
	ki = 0;
	if (state == 0) % NOTHING: Do nothing state
		r_t(:, simCounter) = position; % set the target to the current position
		vel_t(:, simCounter) = linearVel; % set the velocity target to the current target
		acc_t(:, simCounter) = linearAcc; % set the acceleration target to the current acceleration
		jerk_t(:, simCounter) = 0;
		psi_t(simCounter) = eulerAngles(3);
		psi_dot_t(simCounter) = 0;
		generated_trajectory = 0;
		in_hover = 0;
	elseif (state == 1) % HOVER: Hover in current location
		if (in_hover == 0)
			position_for_hover = position;
		end
		r_t(:, simCounter) = position_for_hover;
		vel_t(:, simCounter) = [0; 0; 0];
		acc_t(:, simCounter) = [0; 0; 0];
		jerk_t(:, simCounter) = [0; 0; 0];
		psi_t(simCounter) = eulerAngles(3);
		psi_dot_t(simCounter) = 0;
		generated_trajectory = 0;
		in_hover = 1;
		
	elseif (state == 2) % WAYPOINT: Follow a trajectory to a given location
		in_hover = 0;
		if (~generated_trajectory)
% A note on the trajectories:
% These trajectories are only bounded by start and end points (and their derivatives) so this means that the derivatives can become
% arbitrarily large depending on the time required to go a certain
% distance. This means that for the best response, the path should be
% constrained such that the commanded thrust is less than the maximum
% thrust. For acclerations of larger than ~15 all bets are off (in theory
% about 22 is the true cap), but jerks should not go above ~11. Anything
% above this and the ability to follow the path will degrade. That means
% that given this path generator requires time and distance, care should be
% taken to keep the derivatives withing their bounded zones.

			final_t = t + time_for_trajectory;
			
			trajX = genTrajectory(end_point(1), t, time_for_trajectory, position(1), linearVel(1), a_hat(1), 0, simCounter, dt);
			trajY = genTrajectory(end_point(2), t, time_for_trajectory, position(2), linearVel(2), a_hat(2), 0, simCounter, dt);
			trajZ = genTrajectory(end_point(3), t, time_for_trajectory, position(3), linearVel(3), a_hat(3) + g, 0, simCounter, dt);
			[psi_t, psi_dot_t] = genPsiZero(t, time_for_trajectory, simCounter, dt);
			counter_t = simCounter;
			for n = t:dt:final_t
				r_t(1, counter_t) = trajX(counter_t, 1);
				vel_t(1, counter_t) = trajX(counter_t, 2);
				acc_t(1, counter_t) = trajX(counter_t, 3);
				jerk_t(1, counter_t) = trajX(counter_t, 4);

				r_t(2, counter_t) = trajY(counter_t, 1);
				vel_t(2, counter_t) = trajY(counter_t, 2);
				acc_t(2, counter_t) = trajY(counter_t, 3);
				jerk_t(2, counter_t) = trajY(counter_t, 4);

				r_t(3, counter_t) = trajZ(counter_t, 1);
				vel_t(3, counter_t) = trajZ(counter_t, 2);
				acc_t(3, counter_t) = trajZ(counter_t, 3);
				jerk_t(3, counter_t) = trajZ(counter_t, 4);
				counter_t = counter_t + 1;
			end
			
			generated_trajectory = 1;
		end
		elseif (state == 3) % TRACK: Track a moving target
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
		%%%%%%% NEED to retune once linear acceleration is correct %%%%%%%%
		eta_a = normrnd(mu_a, sigma_a);
		% I never update linearAcc!!! this won't ever be right
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

		% can also improve the sensor estimates (potentially) by propogating the current states out in a linear fashion 

		% Get the horizontal velocity components
		% need a better velocity estimator
		
% 		P_h = [1 0 0; 0 1 0];
% 		
% 		Aah = P_h*R_hat*a_imu;
% 		
% 		v_h = -1/g * inv(P_h*R_hat*D*R_hat'*P_h')*(Aah + g*P_h*R_hat*zw);
% 		v_dot_hat_h = -g*P_h*(R_hat*zw + R_hat*D*R_hat'*(P_h'*v_hat_h)) - k_w*(v_hat_h - v_h);
% 		[~, y] = ode45(@(t_s, v_hat_h) horzVel(g, P_h, zw, R_hat, D, v_hat_h, k_w, v_h), tspan, v_hat_h);
% 		v_hat_h = y(size(y, 1), :)';

	end
	
	%% Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	if(mod(t, controllerRate) == 0)
	%% Differentially Flat Calculations
	
	% Calculate the rotation and angular velocity from the trajectory
	% inputs
	e1 = [cos(psi_t(simCounter)); sin(psi_t(simCounter)); 0];
	
	% Create the body frame axis from the trajectory
	b3 = (acc_t(:, simCounter) - g*zw)/(norm(acc_t(:, simCounter) - g*zw)); % check if we should be adding or subtracting that gravity term
	b2 = cross(b3, e1)/norm(cross(b3, e1));
	b1 = cross(b2, b3);
	b1_hist(:, controllerCounter) = b1;
	b2_hist(:, controllerCounter) = b2;
	b3_hist(:, controllerCounter) = b3;

	% Make the thrust from the trajectory
	u1_t = norm(-m*g*zw + m*acc_t(:, simCounter));

	% Calculate the omega_des from the trajectory
	h_omega = (m/u1_t) * (jerk_t(:, simCounter) - dot(b3, jerk_t(:, simCounter)) * b3);
	omega_des = [dot(-h_omega, b2); dot(h_omega, b1); dot(psi_dot_t(simCounter) * zw, b3)];
	omega_des_hist(:, controllerCounter) = omega_des;
	
	r_des = r_t(:, simCounter);
	vel_des = vel_t(:, simCounter);
	acc_des = acc_t(:, simCounter);
	
	b1_des = b1;
	
	% TODO: Make the controller have a different freq than the sim
	% The controller takes: position, velocity, acceleration, rotation
	% (euler angles), angular velocity
	
	%right now we always look to the next point to calculate the error,
	%instead of using the closest point or something like that
	e_r_prev = e_r;
	e_r = r_hat - end_point;%r_des;
	e_r_hist(:, controllerCounter) = e_r;
	
	e_r_d = (e_r - e_r_prev)/dt;
	if(t == 0)
		i_test = 0;
	else
		i_test = i_test + e_r * dt;
	end
	
	% might make more sense to switch these to "commanded"
	temp = -kp_thrust*e_r -.01*i_test -0.1*e_r_d - m*g*zw;
	b3_com =  (temp)/(norm(-kp_thrust*e_r - m*g*zw));
	b3_com_hist(:, controllerCounter) = b3_com;
	
	b2_com = cross(b3_com, b1_des)/(norm(cross(b3_com, b1_des))); % should I change out b1_des with a vector like e1?
	b2_com_hist(:, controllerCounter) = b2_com;
	
	R_com = [cross(b2_com, b3_com), b2_com, b3_com]; % does not seem to be matching b1,2,3 from above
	b1_com_hist(:, controllerCounter) = cross(b2_com, b3_com);
	R_com_hist(:, :, controllerCounter) = R_com;
	
	u1_com = dot(temp, R_hat*zw); % Can still add in the non-linear terms
	u1_com_hist(controllerCounter) = u1_com;
		
	e_R_temp = 0.5 * ((R_com' * R_hat) - (R_hat' * R_com));	% rotation matrix error (on SO(3))
	e_R = veemap(e_R_temp);	% apply the vee map to the rotation matrix error (map from SO(3) to R^3)
	e_R_hist(:, controllerCounter) = e_R;
	
% 	e_Omega = omega - R_hat' * R_com * omega_des;	% get the error on the rotation speed
% 	e_Omega_hist(:, controllerCounter) = e_Omega;
	
	% TODO: add the last three terms to the desired moments calc
	moments_com = (kp_attitude * e_R); % get the desired moments. this does not have the full non-linear terms right now
	moments_com_hist(:, controllerCounter) = moments_com;

	u_des = [u1_com; moments_com]; % Thrust mag, moment around X, moment around Y, moment around Z (recently changed this order)

	omega_motor_des = gammainv * (u_des); % i think I am getting a motor direction switch around t = 10
	omega_motor_des = sign(omega_motor_des) .* sqrt(abs(omega_motor_des)); % when taking this square root I think I get imaginary number for certain u_des to should take abs first

 	omega_motor_des(1) = 1*(omega_motor_des(1));
 	omega_motor_des(2) = -1*(omega_motor_des(2));
 	omega_motor_des(3) = 1*(omega_motor_des(3));
 	omega_motor_des(4) = -1*(omega_motor_des(4));

	omega_motor_des_hist(:, simCounter) = omega_motor_des;
	end
	if (mod(t, motor_control_freq) == 0)
		Vin = k_motor*(omega_motor_des - omega_motor_act) + k_motor_ff*omega_motor_des; % motor controller
		
		% saturate Vin to the motor battery
		for n = 1:4
			Vin(n) = min([Vin(n), V_batt]);
			Vin(n) = max([Vin(n), -V_batt]);
		end
		Vin_hist(:, simCounter) = Vin;
	end
	controllerCounter = controllerCounter + 1;
	
	
	%% Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Motor Dynamics
 	[~, y] = ode45(@(t_s, omega_motor_act) motorDynamics(b, J, R, k_torque, Vin, omega_motor_act), tspan, omega_motor_act);
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
	
% 	[~, y] = ode45(@(t_s, y) simpleIntegral(linearAcc, 1), tspan, linearVel);
%  	linearVel = (y(size(y, 1), :))';
% 	velHist(:, simCounter) = linearVel;
% 	
% 	[~, y] = ode45(@(t_s, y) simpleIntegral(linearVel, 1), tspan, position);
%  	position = (y(size(y, 1), :))';
	
	if position(3) < 0 % put in the ground
		position(3) = 0;
		linearVel(3) = 0;
	end
	positionHist(:, simCounter) = position; % make the history of all the angles for graphing
	
	simCounter = simCounter + 1;
end
%% Visualization

figure;
subplot(3, 1, 1);
plot(0:dt:endtime, anglesHist(1, :));
title('Yaw of Craft vs. Time');

subplot(3,1,2);
plot(0:dt:endtime, anglesHist(2, :));
title('Pitch of Craft vs Time');

subplot(3,1,3);
plot(0:dt:endtime, anglesHist(3, :));
title('Roll of Craft vs Time');

figure;
subplot(3, 1, 1);
plot(0:dt:endtime, positionHist(1, :));
title('x of Craft vs. Time');

subplot(3,1,2);
plot(0:dt:endtime, positionHist(2, :));
title('y of Craft vs Time');


subplot(3,1,3);
plot(0:dt:endtime, positionHist(3, :));
title('z of Craft vs Time');

figure;
plot3(positionHist(1, :), positionHist(2, :), positionHist(3, :), '-o','Color', 'm', 'MarkerSize', 7);
grid on;
title('position in 3d space');
hold on;
plot3(r_t(1, :), r_t(2, :), r_t(3, :), '--'); 
% for k = 1:simCounter-1
% 	% Generate the normalized unit vectors for the body coordinate system
% 	xb = (rotMatHist(:,:,k) * xw)/norm(rotMatHist(:,:,k) * xw);
% 	yb = (rotMatHist(:,:,k) * yw)/norm(rotMatHist(:,:,k) * yw);
% 	zb = (rotMatHist(:,:,k) * zw)/norm(rotMatHist(:,:,k) * zw);
% 	
% 	
% 	% xb: red
% 	% yb: green
% 	% zb: blue
% 	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), xb(1), xb(2), xb(3), 'AutoScale', 'off', 'Color', [1 0 0]);
% 	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), yb(1), yb(2), yb(3), 'AutoScale', 'off', 'Color', [0 1 0]);
% 	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), zb(1), zb(2), zb(3), 'AutoScale', 'off', 'Color', [0 0 1]);
% 
% 	% can only un-comment this if the controller rate = the simulation rate
% 	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), b1_com_hist(1, k), b1_com_hist(2, k), b1_com_hist(3, k), 'AutoScale', 'off', 'Color', [0 0 0]);
% 	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), b2_com_hist(1, k), b2_com_hist(2, k), b2_com_hist(3, k), 'AutoScale', 'off', 'Color', 'c');
% 	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), b3_com_hist(1, k), b3_com_hist(2, k), b3_com_hist(3, k), 'AutoScale', 'off', 'Color', 'm');
% end
axis equal; %([-30 30 -30 30 -30 30]);
xlabel('X Axis');
ylabel('Y Axis');
zlabel('Z Axis');
hold off;
nC = 1:controllerCounter - 1;
figure; plot(nC, e_r_hist)
legend('x error', 'y error', 'z error')

figure;
n = 1:1:simCounter-1;
plot(n, r_t(3,n));
hold on;
plot(n, vel_t(3,n));
plot(n, acc_t(3,n));
plot(n, jerk_t(3,n));
hold off;
legend('r', 'v', 'a', 'j')

figure; plot(nC, u1_com_hist)
title('Commanded Thrust and Actual Thrust');
%axis([10, 11, 0, 120])
hold on;
plot(n, thrust_hist)
legend('Commanded', 'Actual');
hold off

% figure;
% n = 1:simCounter-1;
% subplot(2,2,1)
% hold on;
% plot(0:dt:endtime, omega_motor_des_hist(1, nC))
% title('Comanded Motor Speed and Actual Motor Speed');
% plot(0:dt:endtime, omega_motor_hist(1,n));
% axis([0, endtime, 0, 2000]);
% hold off;
% 
% subplot(2,2,2)
% hold on;
% plot(0:dt:endtime, omega_motor_des_hist(2, n))
% plot(0:dt:endtime, omega_motor_hist(2,n));
% axis([0, endtime, 0, 2000]);
% hold off;
% 
% subplot(2,2,3)
% hold on;
% plot(0:dt:endtime, omega_motor_des_hist(3, n))
% plot(0:dt:endtime, omega_motor_hist(3,n));
% hold off;
% 
% subplot(2,2,4)
% hold on;
% plot(0:dt:endtime, omega_motor_des_hist(4, n))
% plot(0:dt:endtime, omega_motor_hist(4,n));
% hold off;
% 
% figure;
% subplot(2,2,1)
% plot(0:dt:endtime, Vin_hist(1,n));
% title('Voltage applied to the motor');
% 
% subplot(2,2,2)
% plot(0:dt:endtime, Vin_hist(2,n));
% 
% subplot(2,2,3)
% plot(0:dt:endtime, Vin_hist(3,n));
% 
% subplot(2,2,4)
% plot(0:dt:endtime, Vin_hist(4,n));

function zeta = veemap(matrix)
	zeta = [matrix(2, 3); -matrix(1, 3); matrix(1, 2)];
end

function M = hatmap(vector)
	M = [0 -vector(3) vector(2);
		 vector(3) 0 -vector(1);
		 -vector(2) vector(1) 0];
end

function [trajectory, psi_t] = genTrajectory(end_point, t, time_allowed, r_s, v_s, a_s, j_s, simCounter, dt)
final_t = t + time_allowed;

A_mat = [1*t^7 1*t^6 1*t^5 1*t^4 1*t^3 1*t^2 1*t 1;
			 7*(t^6) 6*(t^5) 5*(t^4) 4*(t^3) 3*(t^2) 2*t 1 0;
			 42*(t^5) 30*(t^4) 20*(t^3) 12*(t^2) 6*t 2 0 0;
			 7*6*5*(t^4) 6*5*4*(t^3) 5*4*3*(t^2) 4*3*2*t 6 0 0 0;
			 1*final_t^7 1*final_t^6 1*final_t^5 1*final_t^4 1*final_t^3 1*final_t^2 1*final_t 1;
			 7*(final_t^6) 6*(final_t^5) 5*(final_t^4) 4*(final_t^3) 3*(final_t^2) 2*final_t 1 0;
			 42*(final_t^5) 30*(final_t^4) 20*(final_t^3) 12*(final_t^2) 6*final_t 2 0 0;
			 7*6*5*(final_t^4) 6*5*4*(final_t^3) 5*4*3*(final_t^2) 4*3*2*final_t 6 0 0 0];
		 
		 
	UX = [r_s; v_s; a_s; j_s; end_point; 0; 0; 0]; % since I don't have a jerk state, right now assuming that jerk always starts and ends at 0
	% assuming that the derivatives at the target are still 0
	coefsX = inv(A_mat)*UX;
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