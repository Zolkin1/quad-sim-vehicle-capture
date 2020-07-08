% obstacle_pos: 3d point in space
% obstacle_rad: radius of the actual obstacle. Only support spheres right
% now
% goal: goal location in 3d space
% start: start location in 3d space
% obstacle_buffer: extra distance around the obstacle to stay out of

function [path_x, path_y, path_z] = genPath(obstacle_pos, obstacle_radi, goal, start, obstacle_buffer)
	eta = 1;
	Q_star = obstacle_buffer + obstacle_radi;
	claw_length = 0.25;
	
	% size of the field to inspect: xmin, xmax, ymin, ymax, zmin, zmax
	% field_size = [min([goal(1), start(1)])-1, max([goal(1), start(1)])+1, min([goal(2), start(2)])-1, max([goal(2), start(2)])+1, min([goal(3), start(3)])-1, max([goal(3), start(3)])+1];
	
% I create the geometric path starting at the goal and going towards the
% start location

% Need to constrain the max tilt angle
	path(:, 1) = goal;
	x = goal(1);
	y = goal(2);
	z = goal(3);
	
	counter = 2;
	while (counter < 1000000) && (abs(y - start(2)) > 0.001 || abs(x-start(1)) > 0.001 || abs(z-start(3)) > 0.001)
		x = path(1, counter-1);
		y = path(2, counter-1);
		z = path(3, counter-1);

		U_att_grad = eta * ([x, y, z] - start);

% 		if z <= obstacle_pos(3)+claw_length+obstacle_rad
% 			Q_star = obstacle_buffer + obstacle_rad;
% 		else
% 			Q_star = obstacle_rad;
% 		end - dont think this was helping

		diff = [x y z] - obstacle_pos;
		unit = diff/norm(diff);
		
		lambda = acos(unit(1));
		theta = acos(unit(2));
		
		x_obs = obstacle_pos(1) + obstacle_radi(1)*cos(theta)*cos(lambda);
		y_obs = obstacle_pos(2) + obstacle_radi(2)*cos(theta)*cos(lambda);
		z_obs = obstacle_pos(3) + obstacle_radi(3)*sin(theta);
		nearest = [x_obs, y_obs, z_obs];
		nearest_hist(:, counter) = nearest;
		
		dist_obstacle = getDist(nearest, x, y, z);
		dist_hist(counter) = dist_obstacle;
		%dist_obstacle = getDist(obstacle_pos, x, y, z);
		if(dist_obstacle <= obstacle_buffer) %Q_star)
			diff = [x y z] - obstacle_pos;
			unit = diff/norm(diff);
			%nearest = obstacle_pos + (unit * obstacle_radi(1)); 
			grad = getGrad(nearest, x, y, z);
			dist = getDist(nearest, x, y, z);
			if (dist < 0.01)
				dist = 0.01;
			end
			U_rep_grad = -40*eta*(1/dist^2)*grad; 
		else
			U_rep_grad = [0, 0, 0];
		end
		
		U_rep_grad_hist(:, counter) = U_rep_grad;
		dist_hist(counter) = dist_obstacle;

		U_grad = U_att_grad + U_rep_grad;

		path(1, counter) = x - 0.0001*U_grad(1);
		path(2, counter) = y - 0.0001*U_grad(2);
		path(3, counter) = z - 0.0001*U_grad(3);
		
		angle_hist(counter) = asin((path(3, counter) - path(3, counter-1))/norm(path(3, counter) - path(3, counter-1)));
		
		counter = counter + 1;
	end
	figure;
	plot(1:counter-1, dist_hist(1:counter-1));

	figure;
	plot(1:counter-1, angle_hist(1:counter-1))
	%legend('x', 'y', 'z');
	% TODO: Work on ways to make the path less wigly. Ideas below
	%
	% 1) TODO: Implement an automatic Hessian generator so I can change
	% polynomial order easier
	% 2) TODO: Enforce velocity, acceleration and jerk continuity
	%	- This will probably require going to a high order, so do the above
	%	step first
	% 3) TODO: Enforce velocity, accleration and jerk inequality laws
	%	- to make this a solvable optimization problem, this may require
	%	lower the waypoints to a certain number
	% TODO: Try to remove the discontinuities as much as possible - maybe
	% draw a straight line back to an earlier location, and it doesnt hit
	% an obstacle, use that instead. This may mitigate the above issue of
	% dropping waypoints to work better.
	%	- Maybe generate from the start to the goal, then generate from the
	%	goal to the start and see what info you can use from that. Maybe
	%	one path will be smoother or they can be merged. I wonder if with 1
	%	obstacle if 1 path will always be better. Maybe then if the
	%	obstacle is near the goal, the one from the goal will always be
	%	better. If the obstacle is above the line conecting the goal and
	%	start then going from the start will be better I think. if its
	%	below that line then generating from the goal will be better
	% TODO: Create a time optimizing program
	% TODO: Create a potnetial field such that the repulsion depends on the
	% side of the vehcicle - so the quad can't get close when its next to
	% it, but it can get close when the quad is above the target. Could
	% make the target and oval.
	
	waypoint_num = 20;
	total_arc_len = calcArcLen(path);
	arc_segment = total_arc_len/waypoint_num;
	temp_arc_len = 0;
	h = 1;
	
	% need to constrain so one waypoint is the end and one is the start
	% garunteed.
	for k = counter-1:-1:2
		temp_arc_len = temp_arc_len + sqrt((path(1,k-1)-path(1,k))^2 + (path(2,k-1)-path(2,k))^2 + (path(3,k-1)-path(3,k))^2);
		if temp_arc_len >= arc_segment
			temp_arc_len = 0;
			waypoints(:, h) = path(:, k);
			h = h + 1;
		end
	end
	
	waypoints(:, waypoint_num) = path(:, 1);
	poly_order = 3;
	num_terms = poly_order + 1;
	t_final = 5;

	x1 = atOnceOptimize(waypoint_num, waypoints, num_terms, t_final, 1);
	figure;
	plotPolys(x1, t_final, waypoint_num, num_terms);
	title('X At once');

	y1 = atOnceOptimize(waypoint_num, waypoints, num_terms, t_final, 2);
	figure;
	plotPolys(y1, t_final, waypoint_num, num_terms);
	title('Y At once');
	
	z1 = atOnceOptimize(waypoint_num, waypoints, num_terms, t_final, 3);
	figure;
	plotPolys(z1, t_final, waypoint_num, num_terms);
	title('Z At once');

	figure;
	plotPath(x1, y1, z1, t_final, waypoint_num, num_terms)
	title('Smooth Path');
	%[x_sphere, y_sphere, z_sphere] = sphere;
	[x_ell, y_ell, z_ell] = ellipsoid(obstacle_pos(1), obstacle_pos(2), obstacle_pos(3), obstacle_radi(1), obstacle_radi(2), obstacle_radi(3));
	hold on;
	surf(x_ell, y_ell, z_ell);
	%	surf(x_sphere*obstacle_radi + obstacle_pos(1), y_sphere*obstacle_radi + obstacle_pos(2), z_sphere*obstacle_radi + obstacle_pos(3));
	hold off;
	axis equal;
	view(45, 45);
	
	figure; plot3(path(1,:), path(2,:), path(3,:))
	title('Path');
	grid on
	axis equal;

end

function dist = getDist(goal, x, y, z)
	dist = sqrt((goal(1) - x)^2 + (goal(2) - y)^2 + (goal(3) - z)^2);
end

function grad = getGrad(obstacle, x, y, z)
	gradx = (((x-obstacle(1))^2 + (y-obstacle(2))^2 + (z-obstacle(3))^2)^-0.5)*(x-obstacle(1));
	grady = (((x-obstacle(1))^2 + (y-obstacle(2))^2 + (z-obstacle(3))^2)^-0.5)*(y-obstacle(2));
	gradz = (((x-obstacle(1))^2 + (y-obstacle(2))^2 + (z-obstacle(3))^2)^-0.5)*(z-obstacle(3));
	
	grad = [gradx, grady, gradz];
end

function arcLen = calcArcLen(path)
	arcLen = 0;
	for k = 1:length(path)-1
		arcLen = arcLen + sqrt((path(1,k+1)-path(1,k))^2 + (path(2,k+1)-path(2,k))^2 + (path(3,k+1)-path(3,k))^2);
	end
end

function x = atOnceOptimize(waypoint_num, waypoints, num_terms, t_final, dimension)
	% Right now I only enforce first derivative continuity at the waypoints
	% so if I am seeing poor performance, it would be best to add
	% acceleration and jerk continuity and add inequality constraints

	H = zeros((num_terms) * waypoint_num, (num_terms) * waypoint_num);
	% right now the Hessian is designed for a 3rd order poly. could write a
	% program to generate a hessian for an nth order poly to minimize jerk
	for i = 1:waypoint_num
		t_start = (i-1)*(t_final/waypoint_num); 
		t_end = t_start + t_final/waypoint_num;
		H(((i-1)*num_terms)+1, ((i-1)*num_terms)+1) = 12*t_end - 12*t_start + 24*t_end^3 - 24*t_start^3; %12;
	end

	A = [];
	B = [];
	end_cond = zeros(1, num_terms*waypoint_num);
	start_cond = zeros(1, num_terms*waypoint_num);

	start_deriv = zeros(1, num_terms*waypoint_num);
	end_deriv = zeros(1, num_terms*waypoint_num);
	start_deriv_val = (waypoints(dimension, 2) - waypoints(dimension, 1))/(t_final/waypoint_num); % this derivative approximation does not take into account acceleration and assumes an even time spline of the path
	end_deriv_val = (waypoints(dimension, 3) - waypoints(dimension, 2))/(t_final/waypoint_num);
	
	for i = 1:waypoint_num-1
		t_start = (i-1)*(t_final/waypoint_num); 
		t_end = t_start + t_final/waypoint_num;
		for k = 0:num_terms-1
			end_cond(((i-1)*num_terms) + k+1) = t_end^(-k+num_terms-1);
			start_cond(((i-1)*num_terms) + k+1) = t_start^(-k+num_terms-1);

				if(-k+num_terms-1 > 0)		
					start_deriv(((i-1)*num_terms) + k+1) = (-k+num_terms-1)*t_start^(-k+num_terms-2);
					end_deriv(((i-1)*num_terms) + k+1) = (-k+num_terms-1)*t_end^(-k+num_terms-2);
				else
					start_deriv(((i-1)*num_terms) + k+1) = 0;
					end_deriv(((i-1)*num_terms) + k+1) = 0;
				end
		end
		Aeq(1 + ((i-1)*num_terms), :) = start_cond;
		Aeq(2 + ((i-1)*num_terms), :) = end_cond;
		Aeq(3 + ((i-1)*num_terms), :) = start_deriv;
		Aeq(4 + ((i-1)*num_terms), :) = end_deriv;

		Beq(1 + ((i-1)*num_terms), 1) = waypoints(dimension, i);
		Beq(2 + ((i-1)*num_terms), 1) = waypoints(dimension, i+1);
		Beq(3 + ((i-1)*num_terms), 1) = start_deriv_val;
		Beq(4 + ((i-1)*num_terms), 1) = end_deriv_val;

		end_cond = zeros(1, num_terms*waypoint_num);
		start_cond = zeros(1, num_terms*waypoint_num);
		start_deriv = zeros(1, num_terms*waypoint_num);
		end_deriv = zeros(1, num_terms*waypoint_num);
		
		start_deriv_val = end_deriv_val;
		if (i+2 < waypoint_num)
				end_deriv_val = (waypoints(dimension, i+3) - waypoints(dimension, i+2))/(t_final/waypoint_num);
		else
				end_deriv_val = 0;
		end
	end

	f = zeros(num_terms*waypoint_num, 1);

	OPTIONS = optimoptions('quadprog', 'Algorithm', 'interior-point-convex', 'MaxIterations', 1000);

	x = quadprog(H, f, A, B, Aeq, Beq, [], [], [], OPTIONS);
end

function plotPolys(x, t_final, waypoint_num, num_terms)
	hold on;
	i = 0;
	
	for k = 1:num_terms:length(x)
		y = 0;
		t_start = (i)*(t_final/waypoint_num); 
		t_end = t_start + t_final/waypoint_num;
		t = t_start:0.001:t_end;
		for h = 0:num_terms-1
			y = y + x(k+h)*(t.^(num_terms-h-1));
		end
		plot(t, y);
		i = i+1;
	end
	hold off;
end

function plotPath(x, y, z, t_final, waypoint_num, num_terms)
	hold on;
	i = 0;
	for k = 1:num_terms:length(x)
		yg = 0;
		xg = 0;
		zg = 0;
		
		t_start = (i)*(t_final/waypoint_num); 
		t_end = t_start + t_final/waypoint_num;
		t = t_start:0.001:t_end;
		
		for h = 0:num_terms-1
			xg = xg + x(k+h)*(t.^(num_terms-h-1));
			yg = yg + y(k+h)*(t.^(num_terms-h-1));
			zg = zg + z(k+h)*(t.^(num_terms-h-1));
		end
		
		plot3(xg, yg, zg, 'Color', 'm', 'Marker', 'o', 'MarkerSize', 5); % need to use the rotate tool to get the 3rd dimension because hold on is wack
		i = i+1;
	end
	grid on;
	xlabel('x');
	ylabel('y');
	zlabel('z');
	hold off;
end