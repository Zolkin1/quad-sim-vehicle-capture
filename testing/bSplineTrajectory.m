close all;
clear;
%% TODO:
% Could eventually put in the receding waypoint version to potentially
% speed up the optimization. Just need to make the horizon far enough out.
%%
% B Spline Derivatives:
% Same knots
% Different Control Points (#control points = n+1-d where n+1 = original
% number of control points, d is the dth derivative)
% Polynomial degree: k-d where k is the original poly degree and d is the
% dth derivative

% To clarify:
% k = order of polynomial
% n+1 = number of control points
% d = the dth derivative to take
% j = knot index
% n-k = # of knot steps

%% Parameters
dx = 0.01;
dist = 16.9;

jmax = 10;
amax = 20;
s = (3*jmax^2)/(2*amax);
vi = (8*amax^2)/(9*jmax);
%% Generate Original Curve
% Pretty sensitive to the initial condition
knot_steps = [1 1 1 1 1 1 1];
% 8 8 8 8 8 8 8 8 8
control = [0 dist/2 dist/2 dist/2 dist/2 dist/2 dist/2 dist/2 dist/2 dist/2 dist];%dist/10 dist/5 3*dist/10 4*dist/10 5*dist/10 6*dist/10 7*dist/10 8*dist/10 9*dist/10 dist];
n = length(control)-1;
k = 4;
knot1 = 0;
fcn = @(x) sum(x(1:length(knot_steps))); % sum the knots steps
nonlcon = @velCon;
x0 = horzcat(knot_steps, control);
A = []; 
b = [];
Aeq = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1;
	   0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0];
beq = [dist; 0];
lb = [0; 0; 0; 0; 0; 0; 0; -200; -200; -200; -200; -200; -200; -200; -200; -200; -200; -200];
ub = [];
OPTIONS = optimoptions('fmincon', 'Algorithm', 'sqp', 'MaxIterations', 1000, 'MaxFunctionEvaluations', 2000);
tic
y = fmincon(fcn,x0,A,b,Aeq,beq,lb,ub,nonlcon, OPTIONS);
toc
control = y(length(knot_steps)+1:length(y));
knot_steps = y(1:length(knot_steps));
d = 1;
for i = 1:n+1-d
	q(i) = controlPointDeriv(i, d, k, n, knot1, knot_steps, control);
end

for temp = 1:k+2
	knots(temp) = knot1;
end
for temp = k+2:7+1+k
	knots(temp) = knots(temp-1) + knot_steps(temp-k-1);
end
for temp = 9+k:k+k+8
	knots(temp) = knots(temp-1);
end
x_final = knots(temp);
n = length(control)-1;
curve = makeCurve(k, length(control)-1, knots, control, dx, x_final);

figure;
plot(0:dx:x_final, curve);
grid on;
legend('BSpline');

%% Generate Derivatives

[curve, knots, q, order] = makeDerivCurve(k, n, knots(1), knot_steps, control, dx, x_final);

figure;
plot(0:dx:x_final, curve);
grid on;

knot_steps = makeKnotSteps(knots, order);

[curve, knots, q, order] = makeDerivCurve(order, n-1, knots(1), knot_steps, q, dx, x_final);

figure;
plot(0:dx:x_final, curve);
grid on;

knot_steps = makeKnotSteps(knots, order);

[curve, knots, q, order] = makeDerivCurve(order, n-2, knots(1), knot_steps, q, dx, x_final);
figure;
plot(0:dx:x_final, curve);
grid on;


%% Integration Functions
function q = controlPointInt(i, d, k, n, knot1, knot_steps, control)
if d == 0
	q = control';
else
	for i = 1:1
		for j = 1:n+d+1
			if j>=i
				sum = 0;
				for q = max([1, j-k-d+1], [], 'all'):min([n-k+1, j-1], [], 'all')
					sum = sum + knot_steps(q);
				end
				
% 				temp1 = getKnot(knot1, knot_steps, k, j-1);
% 				temp2 = getKnot(knot1, knot_steps, k, j-k-d);
				sigma = sum/(k+d);
				
				D(i,j) = sigma;
			else
				D(i,j) = 0;
			end
		end
	end

	q = controlPointInt(i-1, d-1, k, n, knot1, knot_steps, control) * D + 0;
end
end

%% Derivative Functions
function [curve, knots, q, order] = makeDerivCurve(k, n, knot1, knot_steps, control, dx, x_final)
d = 1;
	for i = 1:n+1-d
		q(i) = controlPointDeriv(i, d, k, n, knot1, knot_steps, control);
	end

	nd = length(q)-1;
	counter = 1;
	
	for eta = 1:k
		knots(eta) = knot1;
	end
	for eta = k+1:length(knot_steps)+k
		knots(eta) = knots(eta-1) + knot_steps(eta-k);
	end
	for eta = length(knot_steps)+k+1:length(knot_steps)+k+k
		knots(eta) = knots(eta-1);
	end
	
	for x = 0:dx:x_final
		for i = 1:nd+1
			bd(i, counter) = genBSpline(knots(1:length(knots)), k-1, x, i);
		end
		counter = counter + 1;
	end

	bderiv = 0;
	for i = 1:nd+1
		bderiv = bderiv + q(i)*bd(i,:);
	end
	
	order = k-1;
	curve = bderiv;
end

function q = controlPointDeriv(i, d, k, n, knot1, knot_steps, control)
if d == 0
	q = control(i);
else
	temp1 = getKnot(knot1(1), knot_steps, k, i+k+1);
	temp2 = getKnot(knot1(1), knot_steps, k, i+d);
	
	denom = temp1 - temp2;

	q = ((k-d+1)/denom)*(controlPointDeriv(i+1, d-1, k, n, knot1, knot_steps, control) - controlPointDeriv(i, d-1, k, n, knot1, knot_steps, control));
end
end

%% Normal Spline Functions
function curve = makeCurve(k, n, knots, control, dx, x_final)
	counter = 1;
	b = 0;
	for x = 0:dx:x_final
		for i = 1:n+1
			b(i, counter) = genBSpline(knots, k, x, i);
		end
		counter = counter + 1;
	end

	curve = 0;
	for i = 1:n+1
		curve = curve + control(i)*b(i,:);
	end
end

function b_spline = genBSpline(knots, k, x, i)
	if k == 0
		if (x < knots(i+1) && x >= knots(i))
			b_spline = 1;
		else
			b_spline = 0;
		end
	else
		w = makeW(i, k, x, knots);
		w1 = makeW(i+1, k, x, knots);
		b_spline = w*genBSpline(knots, k-1, x, i) + (1-w1)*genBSpline(knots, k-1, x, i+1);
	end
end

function w = makeW(i, k, x, knots)
	if knots(i+k) > knots(i)
		w = (x - knots(i))/(knots(i+k) - knots(i));
	else
		w = 0;
	end
end

%% Helper Functions
function knot_steps = makeKnotSteps(knots, order)
	counter = 1;
	for m = order+1:length(knots)-order-1
		knot_steps(counter) = knots(m+1)-knots(m);
		counter = counter + 1;
	end
end

function knot = getKnot(knot1, knot_steps, k, index)
	if index <= k
		knot = knot1;
	elseif index <= k+length(knot_steps)
		knot = knot1 + sum(knot_steps((1:index-k-1)));
	else
		knot = knot1 + sum(knot_steps);
	end
end

%% Constraint Functions
function [c ceq] = velCon(x)
% constrain the square of the control points to be less than the max vel
% structure of x:
% all the knot spaces then all the control points
% assuming a 4th order position poly which means n+1 = 7 and m+1 = 12
ceq = [];
d = 1;
k = 4;
n = 10;
knot1 = 0;
knot_steps = x(1:7);
control = x(8:18);

for i = 1:n+1-d
	q(i) = controlPointDeriv(i, d, k, n, knot1, knot_steps, control);
end
c(1:n) = q.^2 - 20^2;
ceq(1) = q(1);
ceq(2) = q(n);

d = 1;
for i = 1:n+1-d-1
	p(i) = controlPointDeriv(i, d, k-1, n-1, knot1, knot_steps, q);
end
c(n+1:n+n-1) = p.^2 - 15^2;
ceq(3) = p(1);
ceq(4) = p(n-1);

d = 1;
for i = 1:n+1-d-2
	r(i) = controlPointDeriv(i, d, k-2, n-2, knot1, knot_steps, p);
end
c(n+n+1:n+n+n-2) = r.^2 - 11^2;
% would need another degree on the polynomial to get these constraints
%ceq(5) = r(1);
%ceq(6) = r(n-2);

end