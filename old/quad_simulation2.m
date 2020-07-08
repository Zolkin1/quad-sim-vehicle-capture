clear all;
close all;
%% Next: trajectory planner, then IMU. That way we can prove everything else before I add the IMU error to the model
%% Simulation Parameters
endtime = 10;
dt = 0.05;

%% Quadrotor Parameters
I = [0.1 0 0;
	 0 0.1 0;
	 0 0 0.1]; % moment of inertia matrix
m = 4; % mass matrix
g = -9.8; % gravity

L = 0.25; % length of quad arm

c_T = .0005; % no idea if this is the correct ball park, check this
c_Q = 0.0008; %1.5; % also not sure if this is the correct order of magnitude
gamma = [c_T, c_T, c_T, c_T;
		 0, L*c_T, 0, -L*c_T;
		 -L*c_T, 0, L*c_T, 0;
		 -c_Q, c_Q, -c_Q, c_Q];  
gammainv = inv(gamma);

A1c = 1; % check order of magnitude
A1s = 1; % check order of magnitude
dx = 0.02; % this should be roughly correct. Note that we drag I believe I have a speed cap
dy = 0.02; % drag in y
%% Motor Parameters
% TODO: Get more realistic motor params so the battery voltage is more
% normal
b = 0.05;
J = 0.01;
R = 5;
k_torque = 0.06; % still needs modification
V_batt = 40000; % volts - this requires accurate motor model constants. So right now it is set very high until I get accurate motor model constants
% that will require re-tuning the gains
%% Initial Conditions
eulerAngles = zeros(3,1); % ZYX: yaw, roll, pitch
omega = zeros(3,1);
omegadot = zeros(3,1);

position = zeros(3,1); % this is position of the center of mass of the quad in the world frame
linearVel = zeros(3,1); % in the world frame
linearAcc = zeros(3,1); % in the world frame

omega_motor_act = zeros(4,1);

rotMat = eul2rotm(eulerAngles', 'XYZ');

xw = [1; 0; 0]; % world coordinate x axis
yw = [0; 1; 0]; % world coordinate y axis
zw = [0; 0; 1]; % world coordinate z axis

anglesHist = zeros(3,1);
positionHist = zeros(3,1);
rotMatHist = zeros(3, 3, 1); % holds the rotation matricies for each dt
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
k_motor = 1;	% probably need to retune after rotor flapping and drag are added in. actually it should only depend on the motor model which wont change unless I add in rotor-air drag at that level
k_motor_ff = 4.1; % see above - probably need to retune

kp_attitude = [19.25 0 0;
			   0 19.5 0;
			   0 0 19.5];
kd_attitude = [6 0 0;
			   0 6 0;
			   0 0 6];

kp_thrust = [12.5 0 0;
			 0 12.5 0;
			 0 0 12.5];
kd_thrust = [30.5 0 0;
			 0 30.5 0;
			 0 0 30.5];

%% Trajectory Planning
% The trajectory will give the x,y,z (and up to 4 derivatives) and yaw (up to 2 derivatives) of a dynamically feasible
% trajectory. Thus it will have a continuous 4th derivative for position.

% basic trapezoidal trajectories for tracking to a stationary target
j_max = 50;
a_max = 10;
v_max = 20;

start_point = [0; 0; 0];
end_point = [25; 6; 20];

jerk_t(:, 1) = [0; 0; 0];
acc_t(:, 1) = [0; 0; 0];
vel_t(:, 1) = [0; 0; 0];
r_t(:, 1) = [0; 0; 0];
psi_t(1) = 0;
psi_dot_t(1) = 0;

% do it with splines
% start at a (t = 0)
% end at b (t = 1)
% r_t(t) = A2t^7 + A3t^6 + A4t^5 + A5t^4 + A6t^3 + A7t^2 + A8t + A9

A_mat = [0 0 0 0 0 0 0 1;
		 0 0 0 0 0 0 1 0;
		 0 0 0 0 0 1 0 0;
		 0 0 0 0 1 0 0 0;
		 1*endtime^7 1*endtime^6 1*endtime^5 1*endtime^4 1*endtime^3 1*endtime^2 1*endtime 1;
		 7*(endtime^6) 6*(endtime^5) 5*(endtime^4) 4*(endtime^3) 3*(endtime^2) 2*endtime 1 0;
		 42*(endtime^5) 30*(endtime^4) 20*(endtime^3) 12*(endtime^2) 6*endtime 2 0 0;
		 7*6*5*(endtime^4) 6*5*4*(endtime^3) 5*4*3*(endtime^2) 4*3*2*endtime 0 0 0 0];
UX = [0; 0; 0; 0; end_point(1); 0; 0; 0];

coefsX = inv(A_mat)*UX;

counter_t = 1;
for t = 0:dt:endtime
	r_t(1, counter_t) = coefsX(1)*t^7 + coefsX(2)*t^6 + coefsX(3)*t^5 + coefsX(4)*t^4; 
	vel_t(1, counter_t) = 7*coefsX(1)*t^6 + 6*coefsX(2)*t^5 + 5*coefsX(3)*t^4 + 4*coefsX(4)*t^3;
	acc_t(1, counter_t) = 7*6*coefsX(1)*t^5 + 6*5*coefsX(2)*t^4 + 5*4*coefsX(3)*t^3 + 4*3*coefsX(4)*t^2;
	jerk_t(1, counter_t) = 7*6*5*coefsX(1)*t^4 + 6*5*4*coefsX(2)*t^3 + 5*4*3*coefsX(3)*t^2 + 4*3*2*coefsX(4)*t;

	psi_dot_t(counter_t) = 0;
 	psi_t(counter_t) = 0;
	counter_t = counter_t + 1;	
end

UY = [0; 0; 0; 0; end_point(2); 0; 0; 0];

coefsY = inv(A_mat)*UY;
counter_t = 1;
for t = 0:dt:endtime
	r_t(2, counter_t) = coefsY(1)*t^7 + coefsY(2)*t^6 + coefsY(3)*t^5 + coefsY(4)*t^4; 
	vel_t(2, counter_t) = 7*coefsY(1)*t^6 + 6*coefsY(2)*t^5 + 5*coefsY(3)*t^4 + 4*coefsY(4)*t^3;
	acc_t(2, counter_t) = 7*6*coefsY(1)*t^5 + 6*5*coefsY(2)*t^4 + 5*4*coefsY(3)*t^3 + 4*3*coefsY(4)*t^2;
	jerk_t(2, counter_t) = 7*6*5*coefsY(1)*t^4 + 6*5*4*coefsY(2)*t^3 + 5*4*3*coefsY(3)*t^2 + 4*3*2*coefsY(4)*t;
	
	counter_t = counter_t + 1;	
end

 UZ = [0; 0; 0; 0; end_point(3); 0; 0; 0];

coefsZ = inv(A_mat)*UZ;
counter_t = 1;
for t = 0:dt:endtime
	r_t(3, counter_t) = coefsZ(1)*t^7 + coefsZ(2)*t^6 + coefsZ(3)*t^5 + coefsZ(4)*t^4; 
	vel_t(3, counter_t) = 7*coefsZ(1)*t^6 + 6*coefsZ(2)*t^5 + 5*coefsZ(3)*t^4 + 4*coefsZ(4)*t^3;
	acc_t(3, counter_t) = 7*6*coefsZ(1)*t^5 + 6*5*coefsZ(2)*t^4 + 5*4*coefsZ(3)*t^3 + 4*3*coefsZ(4)*t^2;
	jerk_t(3, counter_t) = 7*6*5*coefsZ(1)*t^4 + 6*5*4*coefsZ(2)*t^3 + 5*4*3*coefsZ(3)*t^2 + 4*3*2*coefsZ(4)*t;
	
	psi_dot_t(counter_t) = 0;
 	psi_t(counter_t) = 0;
	counter_t = counter_t + 1;	
end


figure;
n = 1:1:counter_t - 1;
plot(n, r_t(1,n));
hold on;
plot(n, vel_t(1,n));
plot(n, acc_t(1,n));
plot(n, jerk_t(1,n));
hold off;


%a _t represents that it came from the trajectory planning
	
%% Simulation
simCounter = 1;
for t = 0:dt:endtime
	tspan = [t, t+dt];
	%% Differentially Flat Calculations
	
	% Calculate the rotation and angular velocity from the trajectory
	% inputs
	e1 = [cos(psi_t(simCounter)); sin(psi_t(simCounter)); 0];
	
	% Create the body frame axis from the trajectory
	b3 = (acc_t(:, simCounter) - g*zw)/(norm(acc_t(:, simCounter) - g*zw)); % check if we should be adding or subtracting that gravity term
	b2 = cross(b3, e1)/norm(cross(b3, e1));
	b1 = cross(b2, b3);
	b1_hist(:, simCounter) = b1;
	b2_hist(:, simCounter) = b2;
	b3_hist(:, simCounter) = b3;

	% Make the thrust from the trajectory
	u1_t = norm(-m*g*zw + m*acc_t(:, simCounter));

	% Calculate the omega_des from the trajectory
	h_omega = (m/u1_t) * (jerk_t(:, simCounter) - dot(b3, jerk_t(:, simCounter)) * b3);
	omega_des = [dot(-h_omega, b2); dot(h_omega, b1); dot(psi_dot_t(simCounter) * zw, b3)];
	omega_des_hist(:, simCounter) = omega_des;
	
	r_des = r_t(:, simCounter);
	vel_des = vel_t(:, simCounter);
	acc_des = acc_t(:, simCounter);
	
	b1_des = b1;
	
	%% Controller
	% TODO: Make the controller have a different freq than the sim
	% The controller takes: position, velocity, acceleration, rotation
	% (euler angles), angular velocity
	
	%right now we always look to the next point to calculate the error,
	%instead of using the closest point or something like that
	e_r = position - r_des; % something is giving me imaginary position
	e_v = linearVel - vel_des;
	e_r_hist(:, simCounter) = e_r;
	
	% might make more sense to switch these to "commanded"
	b3_com =  (-kp_thrust*e_r - kd_thrust*e_v - m*g*zw + m*acc_des)/(norm(-kp_thrust*e_r - kd_thrust*e_v - m*g*zw + m*acc_des));
	b3_com_hist(:, simCounter) = b3_com;
	
	b2_com = cross(b3_com, b1_des)/(norm(cross(b3_com, b1_des))); % should I change out b1_des with a vector like e1?
	b2_com_hist(:, simCounter) = b2_com;
	
	R_com = [cross(b2_com, b3_com), b2_com, b3_com]; % does not seem to be matching b1,2,3 from above
	b1_com_hist(:, simCounter) = cross(b2_com, b3_com);
	R_com_hist(:, :, simCounter) = R_com;
	
	
	u1_com = dot(-kp_thrust*e_r - kd_thrust*e_v -m*g*zw + m*acc_des, rotMat*zw); % Can still add in the non-linear terms
	u1_com_hist(simCounter) = u1_com;
	
	e_R_temp = 0.5 * ((R_com' * rotMat) - (rotMat' * R_com));	% rotation matrix error (on SO(3))
	e_R = veemap(e_R_temp);	% apply the vee map to the rotation matrix error (map from SO(3) to R^3)
	e_R_hist(:, simCounter) = e_R;
	
	e_Omega = omega - rotMat' * R_com * omega_des;	% get the error on the rotation speed
	e_Omega_hist(:, simCounter) = e_Omega;
	
	% TODO: add the last three terms to the desired moments calc
	moments_com = (kp_attitude * e_R) - (kd_attitude * e_Omega); % get the desired moments. this does not have the full non-linear terms right now
	moments_com_hist(:, simCounter) = moments_com;

	%TODO: check this dot product
	u_des = [u1_com; moments_com]; % Thrust mag, moment around X, moment around Y, moment around Z (recently changed this order)

	omega_motor_des = gammainv * (u_des);
	omega_motor_des = sign(omega_motor_des) .* sqrt(abs(omega_motor_des)); % when taking this square root I think I get imaginary number for certain u_des to should take abs first
 	omega_motor_des(1) = 1*(omega_motor_des(1));
 	omega_motor_des(2) = -1*(omega_motor_des(2));
 	omega_motor_des(3) = 1*(omega_motor_des(3));
 	omega_motor_des(4) = -1*(omega_motor_des(4));
	
	Vin = k_motor*(omega_motor_des - omega_motor_act) + k_motor_ff*omega_motor_des; % motor controller
	
	% saturate Vin to the motor battery
	for n = 1:4
		Vin(n) = min([Vin(n), V_batt]);
		Vin(n) = max([Vin(n), -V_batt]);
	end
	%% Motor Dynamics
 	[~, y] = ode45(@(t_s, omega_motor_act) motorDynamics(b, J, R, k_torque, Vin, omega_motor_act), tspan, omega_motor_act);
 	omega_motor_act = y(size(y, 1), :)';
 	omega_motor_hist(:, simCounter) = omega_motor_act;
	
	u = gamma * omega_motor_act.^2;
	moments = u(2:4);
	thrust = u(1) * zw;
	
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
	F_ext = u(1) * ((D_ind + A_flap) * (rotMat' * linearVel)); % i think this may be applying force in the wrong directions
	
	%% Position Dynamics
	[~, y] = ode45(@(t_s, y) sumOfForces(rotMat, g, m, thrust, F_ext), tspan, linearVel);
	linearVel = (y(size(y, 1), :))';
	velHist(:, simCounter) = linearVel;
	
	[~, y] = ode45(@(t_s, y) simpleIntegral(linearVel, 1), tspan, position);
	position = (y(size(y, 1), :))';
	if position(3) < 0 % add the ground
		position(3) = 0;
	end
	positionHist(:, simCounter) = position; % make the history of all the angles for graphing
	
	simCounter = simCounter + 1;
end
%% Visualization
% gonna need to somehow show all this data.
% Im thinking that generating an animations with some polygon for the
% quadcopter would be good. But I think on a more basic level, I will
% need a set of 6 graphs so show all the lowest derivative states
% It would also be nice to have the ability to step theough the data x
% number of dt's at a time. Do i need to make a GUI? Maybe
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
plot3(positionHist(1, :), positionHist(2, :), positionHist(3, :), '-o', 'Color', 'm', 'MarkerSize', 7);
grid on;
title('position in 3d space');
hold on;
plot3(r_t(1, :), r_t(2, :), r_t(3, :), '--'); 
for k = 1:simCounter-1
	% Generate the normalized unit vectors for the body coordinate system
	xb = (rotMatHist(:,:,k) * xw)/norm(rotMatHist(:,:,k) * xw);
	yb = (rotMatHist(:,:,k) * yw)/norm(rotMatHist(:,:,k) * yw);
	zb = (rotMatHist(:,:,k) * zw)/norm(rotMatHist(:,:,k) * zw);
	
	
	% xb: red
	% yb: green
	% zb: blue
	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), xb(1), xb(2), xb(3), 'AutoScale', 'off', 'Color', [1 0 0]);
	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), yb(1), yb(2), yb(3), 'AutoScale', 'off', 'Color', [0 1 0]);
	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), zb(1), zb(2), zb(3), 'AutoScale', 'off', 'Color', [0 0 1]);
% 	
% 	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), b1_hist(1, k), b1_hist(2, k), b1_hist(3, k), 'AutoScale', 'off', 'Color', [0 0 0]);
% 	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), b2_hist(1, k), b2_hist(2, k), b2_hist(3, k), 'AutoScale', 'off', 'Color', 'c');
% 	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), b3_hist(1, k), b3_hist(2, k), b3_hist(3, k), 'AutoScale', 'off', 'Color', 'm');
	
	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), b1_com_hist(1, k), b1_com_hist(2, k), b1_com_hist(3, k), 'AutoScale', 'off', 'Color', [0 0 0]);
	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), b2_com_hist(1, k), b2_com_hist(2, k), b2_com_hist(3, k), 'AutoScale', 'off', 'Color', 'c');
	quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), b3_com_hist(1, k), b3_com_hist(2, k), b3_com_hist(3, k), 'AutoScale', 'off', 'Color', 'm');
	
	% linear velocity in the world frame at each point
	%quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), velHist(1, k)/norm(velHist(:, k)), velHist(2, k)/norm(velHist(:, k)), velHist(3, k)/norm(velHist(:, k)), 'AutoScale', 'off', 'Color', [0 1 1]);
	
	% gravity at every point
	%quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), 0, 0, -9.8, 'AutoScale', 'off', 'Color', [0 0 0]);
	
	% applied thrust (due to the rotors) at each point
	%tw = rotMatHist(:,:,k)*thrust;
	%quiver3(positionHist(1, k), positionHist(2, k), positionHist(3, k), tw(1), tw(2), tw(3), 'AutoScale', 'off', 'Color', [0 0 0]);

end
axis equal; %([-30 30 -30 30 -30 30]);
xlabel('X Axis');
ylabel('Y Axis');
zlabel('Z Axis');
hold off;

figure; plot(0:dt:endtime, e_r_hist)
legend('x error', 'y error', 'z error')

function zeta = veemap(matrix)
	zeta = [matrix(2, 3); -matrix(1, 3); matrix(1, 2)];
end
