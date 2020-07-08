close all;
%%
end_time = .75;
dt = .25;
xw = [1; 0; 0];
yw = [0; 1; 0];
zw = [0; 0; 1];
bodyframe = zeros(3,3);

omega = zeros(3, 1);
position = zeros(3, 1);
velocity = zeros(3, 1);
acceleration = zeros(3, 1);
angles = zeros(3, 1);
velocity_body = 0;

u = [1; 1; 0; 0]; %thrust, roll, pitch, yaw moments
I = [0.1 0 0;
	 0 0.1 0;
	 0 0 0.1]; %The moment of inertia matrix. It is symmetric because I am using a simplified model of the quad
mass = 0.5; % Check the units on this
g = 9.8;
quiver3(0, 0, 0, 1, 0, 0); %x world
hold on;
quiver3(0, 0, 0, 0, 1, 0); %y world
quiver3(0, 0, 0, 0, 0, 1); %z world
%quiver3(0, 0, 0, xt(1), xt(2), xt(3)); %x body
%quiver3(0, 0, 0, yt(1), yt(2), yt(3)); %y body
%quiver3(0, 0, 0, zt(1), zt(2), zt(3)); %z body
hold off;
%%
for t = 0:dt:end_time
	%% Get the inputs from the controller
	%u = get_inputs(x); %this will ultimetely come from the
	%IK/differentially flat stuff
	%% Calculate the change in the body frame of the quad
	% This should involve numeric integration of the angular velocity
	% equations
	omegadot = getAngularAccel(omega, u([2 3 4]), I);
	omega = omega + omegadot * dt; % very simple integration could probably use something better (like trapezoidal) (might want to write a function for that if its used a lot)
	angles = angles + omega * dt; % each term of omega should be the time derivative of the Euler angles
	rotMat = eul2rotm(angles', 'XYZ'); % eul2rotm uses ZYX as default, so I changed it to xyz
	% make the rotation matrix from the world frame to the body frame with the given euler angles
	% should test this stuff visually and make sure it looks correct
	% can also consider making the intermideate axis based on the yaw
	% rotation and working with that instead of the default maltab one.
	xt = rotMat * xw; % need a good way to graph this
	yt = rotMat * yw;
	zt = rotMat * zw;
	
	bodyframe(:, 1) = rotMat * xw;
	bodyframe(:, 2) = rotMat * yw;
	bodyframe(:, 3) = rotMat * zw;
	
	%% Update the acceleration of the quad - might not need this if all the calculations are done in the body frame
	accelerationMag = u(1)/mass; % need to ADD IN gravity - so its probably best if i make this a vector
	velocity_body = velocity_body + accelerationMag * dt; % this is making
	%the velocity a scalar value because it is only ever in the direction
	%of the z_B (double check this)
	position = position + velocity_body * bodyframe(:, 3) * dt;
	hold on;
	quiver3(0, 0, 0, position(1), position(2), position(3));
	hold off;
end

function omegadot = getAngularAccel(omega, moments, I)
	omegadot = inv(I) * (moments - (cross(omega, I * omega)));
end