close all;
clear

obstacle_pos = [4, 4];
obstacle_rad = 1;

goal = [9, 9];
start = [1.5,.5];

eta = 1;
Q_star = 0.5 + obstacle_rad;

field_size = [0, 10, 0, 10];
dd = 0.1;

x_count = 1;
y_count = 1;

[X, Y] = meshgrid(field_size(1):dd:field_size(2), field_size(3):dd:field_size(4));
for x = field_size(1):dd:field_size(2)
	for y = field_size(3):dd:field_size(4)
		dist_goal = getDist(goal, x, y);
		dist_obstacle = getDist(obstacle_pos, x, y);
		
		U_att(x_count, y_count) = 0.5 * eta * dist_goal;
		U_att_grad(:, x_count, y_count) = eta * ([x, y] - goal);
		temp = [x, y] - goal;
		U_att_grad2x(x_count, y_count) = eta * temp(1);
		U_att_grad2y(x_count, y_count) = eta * temp(2);
		
		
		if (dist_obstacle <= Q_star)
			diff = [x y] - obstacle_pos;
			unit = diff/norm(diff);
			nearest = obstacle_pos + (unit * obstacle_rad); 
			dist = getDist(nearest, x, y);
			if (dist < 0.1)
				dist = 0.1;
			end
				
			if(dist_obstacle < obstacle_rad + 0.1)
				U_rep(x_count, y_count) = 1/dist;
			else	
				U_rep(x_count, y_count) = (1/dist);
				grad = getGrad(nearest, x, y);
				U_rep_grad2x(x_count, y_count) = -40*eta*(1/dist^2)*grad(1);
				U_rep_grad2y(x_count, y_count) = -40*eta*(1/dist^2)*grad(2);
			end
		else
			U_rep_grad(:, x_count, y_count) = [0, 0];
			U_rep_grad2x(x_count, y_count) = 0;
			U_rep_grad2y(x_count, y_count) = 0;
			U_rep(x_count, y_count) = 0;
		end
		y_count = y_count + 1;
	end
	y_count = 1;
	x_count = x_count + 1;
end

U = U_att+U_rep;
U_grad = U_att_grad + U_rep_grad;

figure;
surf(X, Y, U);

x_count = 1;
y_count = 1;

path = [start(1); start(2)];

path(:, 1) = start;
counter = 2;
while( (counter < 1000000) && (abs(y - goal(2)) > 0.1 || abs(x-goal(1)) > 0.1))
	x = path(1, counter-1);
	y = path(2, counter-1);
	U_att_grad = eta * ([x, y] - goal);
	U_att_hist(:, counter) = U_att_grad;
	
	dist_obstacle = getDist(obstacle_pos, x, y);
	if(dist_obstacle <= Q_star)
		diff = [x y] - obstacle_pos;
		unit = diff/norm(diff);
		nearest = obstacle_pos + (unit * obstacle_rad); 
		grad = getGrad(nearest, x, y);
		dist = getDist(nearest, x, y);
		if (dist < 0.1)
			dist = 0.1;
		end
		U_rep_grad = -40*eta*(1/dist^2)*grad; 
	else
		U_rep_grad = [0, 0];
	end
	U_rep_grad_hist(:, counter) = U_rep_grad;
	dist_hist(counter) = dist_obstacle;

	test(counter) = ((1/Q_star) - (1/dist_obstacle))*(1/dist_obstacle^2);

	U_grad = U_att_grad + U_rep_grad;

	path(1, counter) = x - 0.0001*U_grad(1);
	path(2, counter) = y - 0.0001*U_grad(2);

	counter = counter + 1;
end

hold on;
plot(path(1, :), path(2, :), 'Color', 'm', 'Marker', '+', 'MarkerSize', 7);
hold off;
figure;
plot(path(1, :), path(2, :));

figure;
plot(1:counter-1, U_rep_grad_hist);
hold on;
plot(1:counter-1, U_att_hist);
plot(1:counter-1, U_att_hist+U_rep_grad_hist);
hold off;
legend('repX', 'repY', 'attX', 'attY', 'totalX', 'totalY');

figure;
quiver(X, Y,  U_att_grad2x+U_rep_grad2x , U_att_grad2y+U_rep_grad2y, 'AutoScale', 'on', 'AutoScaleFactor', 20,'MarkerSize', 25);
hold on;
plot(path(1, :), path(2, :));
hold off;

figure;
plot(1:counter-1, U_rep_grad_hist);
title('repelant force');

figure;
plot(1:counter-1, dist_hist);
title('dist from obstacle');

%%

waypoint_num = 20;
total_arc_len = calcArcLen(path);
arc_segment = total_arc_len/waypoint_num;
temp_arc_len = 0;
h = 1;
for k = 1:counter-2
	temp_arc_len = temp_arc_len + sqrt((path(1,k+1)-path(1,k))^2 + (path(2,k+1)-path(2,k))^2);
	if temp_arc_len >= arc_segment
		temp_arc_len = 0;
		waypoints(:, h) = path(:, k);
		h = h + 1;
	end
end
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

figure;
plotPath(x1, y1, t_final, waypoint_num, num_terms)
title('Smooth Path');

figure; plot(path(1,:), path(2,:))
title('Path');
hold on;
scatter(waypoints(1,:), waypoints(2,:));
plotPath(x1, y1, t_final, waypoint_num, num_terms)
hold off;
legend('True Path', 'Waypoints');

figure;
plotVel(x1, t_final, waypoint_num, num_terms);
hold on
plotAcc(x1, t_final, waypoint_num, num_terms);
hold off;
title('x vel');

% deriv_val_start = (waypoints(1,2) - waypoints(1,1))/(t_final/waypoint_num);
% deriv_val_end = (waypoints(1,3) - waypoints(1,2))/(t_final/waypoint_num);
% x_total = 0;
% 
% for k = 1:waypoint_num-1
% 	t_start = (k-1)*(t_final/waypoint_num); 
% 	t_end = t_start + t_final/waypoint_num;
% 	
% 	x = optimizePoly(num_terms, waypoints(1, k), waypoints(1, k+1), t_end, t_start, deriv_val_start, deriv_val_end);
% 	deriv_val_start = deriv_val_end;
% 	if (k+3 > waypoint_num)
% 		deriv_val_end = 0;
% 	else
% 		deriv_val_end = (waypoints(1, k+3) - waypoints(1, k+2))/(t_final/waypoint_num);
% 	end
% 	
% 	if k == 1
% 		x_total = x;
% 	else
% 		x_total = [x_total; x];
% 	end
% end
% going to break this into 3 optimization problems - independent in each
% axis
% the size of the optimization vector is going to be poly_order+1 *
% waypoint_num per dimension

% plotPolys(x_total, t_final, waypoint_num, num_terms);
% title('Iterative');

function dist = getDist(goal, x, y)
	dist = sqrt((goal(1) - x)^2 + (goal(2) - y)^2);
end

function grad = getGrad(obstacle, x, y)
	gradx = (((x-obstacle(1))^2 + (y-obstacle(2))^2)^-0.5)*(x-obstacle(1));
	grady = (((x-obstacle(1))^2 + (y-obstacle(2))^2)^-0.5)*(y-obstacle(2));
	grad = [gradx, grady];
end

function x = optimizePoly(num_terms, start_loc, stop_loc, t_end, t_start, deriv_val_start, deriv_val_end)
	H = zeros(num_terms);
	H(1,1) = 12*t_end - 12*t_start + 24*t_end^3 - 24*t_start^3; % minimize jerk
	A = [];
	B = [];
	
	Aeq = zeros(num_terms);
	Beq = zeros(num_terms, 1);
	
	end_cond = zeros(1, num_terms);
	start_cond = zeros(1, num_terms);

	start_deriv = zeros(1, num_terms);
	end_deriv = zeros(1, num_terms);
	for k = 0:num_terms-1
		end_cond(k+1) = t_end^(-k+num_terms-1);
		start_cond(k+1) = t_start^(-k+num_terms-1);

		if(-k+num_terms-1 > 0)		
			start_deriv(k+1) = (-k+num_terms-1)*t_start^(-k+num_terms-2);
			end_deriv(k+1) = (-k+num_terms-1)*t_end^(-k+num_terms-2);
		else
			start_deriv(k+1) = 0;
			end_deriv(k+1) = 0;
		end
	end
	Aeq(1, :) = start_cond;
	Aeq(2, :) = end_cond;
	Aeq(3, :) = start_deriv;
	Aeq(4, :) = end_deriv;

	Beq(1, 1) = start_loc;
	Beq(2, 1) = stop_loc;
	Beq(3, 1) = deriv_val_start;
	Beq(4, 1) = deriv_val_end;

	f = zeros(num_terms, 1);
	
	OPTIONS = optimoptions('quadprog', 'Algorithm', 'interior-point-convex', 'MaxIterations', 1000);
	x = quadprog(H, f, A, B, Aeq, Beq, [], [], [], OPTIONS);
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
	
	for i = 1:waypoint_num-2
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
		if (i+3 < waypoint_num)
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

function plotPath(x, y, t_final, waypoint_num, num_terms)
	hold on;
	i = 0;
	for k = 1:num_terms:length(x)
		yg = 0;
		xg = 0;
		
		t_start = (i)*(t_final/waypoint_num); 
		t_end = t_start + t_final/waypoint_num;
		t = t_start:0.001:t_end;
		
		for h = 0:num_terms-1
			xg = xg + x(k+h)*(t.^(num_terms-h-1));
			yg = yg + y(k+h)*(t.^(num_terms-h-1));
		end
		
		plot(xg, yg);
		i = i+1;
	end
	hold off;
end

function arcLen = calcArcLen(path)
	arcLen = 0;
	for k = 1:length(path)-1
		arcLen = arcLen + sqrt((path(1,k+1)-path(1,k))^2 + (path(2,k+1)-path(2,k))^2);
	end
end

function plotVel(x, t_final, waypoint_num, num_terms)
	hold on;
	i = 0;
	
	for k = 1:num_terms:length(x)
		y = 0;
		t_start = (i)*(t_final/waypoint_num); 
		t_end = t_start + t_final/waypoint_num;
		t = t_start:0.001:t_end;
		for h = 0:num_terms-1
			y = y + (num_terms-h-1)*x(k+h)*(t.^(num_terms-h-2));
		end
		plot(t, y);
		i = i+1;
	end
	hold off;
end

function plotAcc(x, t_final, waypoint_num, num_terms)
	hold on;
	i = 0;
	
	for k = 1:num_terms:length(x)
		y = 0;
		t_start = (i)*(t_final/waypoint_num); 
		t_end = t_start + t_final/waypoint_num;
		t = t_start:0.001:t_end;
		for h = 0:num_terms-1
			y = y + (num_terms-h-1)*(num_terms-h-2)*x(k+h)*(t.^(num_terms-h-3));
		end
		plot(t, y);
		i = i+1;
	end
	hold off;
end