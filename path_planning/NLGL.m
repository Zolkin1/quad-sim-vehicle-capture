% take
% P = (x,y,z) of the quad.
% r(gamma) = path in terms of gamma where 0 <= gamma <= 1
% L = look ahead distance
% s = orientation of the quad
% maybe a(gamma) = accleration as a fcn of gamma

% consider how the path is accessed - is it in discrete chunks? (rn thats
% what I am doing)
function [b3_d, gamma_vtp, d, a] = NLGL(P, r, L, gamma_prev) % FIX NLGL
[d, gamma] = getMinDist(P, r, gamma_prev, 0);
if d >= L
	gamma_vtp = gamma;
else
	[d, gamma_vtp] = getMinDist(P, r, gamma_prev, L);
	d = d + L;
end

% now determine the desired orientation to get to the location
b3_d = (r(:, gamma_vtp) - P')/norm(r(:, gamma_vtp) - P');
%graph(P, r, b3_d, L, gamma_vtp, norm(r(:, gamma_vtp) - P'));
a = 2*d;
end

function [d, gamma] = getMinDist(P, r, gamma_prev, L)
gamma = gamma_prev+1;
d_prev_1 = abs(norm(P' - r(:, gamma_prev))-L);
d_prev_2 = 1; 
minNotFound = true;

while minNotFound && gamma < length(r)
	d = abs(norm(P' - r(:, gamma)) - L);
	if d_prev_1 < d_prev_2 && d_prev_1 < d && d_prev_1 <= abs(norm(P' - r(:, gamma_prev))-L)
		minNotFound = false;
	else
		gamma = gamma + 1;
	end
	d_prev_2 = d_prev_1;
	d_prev_1 = d;
end
d = d_prev_1;
if gamma ~= gamma_prev+1
gamma = gamma - 1;
end
if gamma > length(r)
	gamma = gamma - 1;
end
end

function graph(P, r, b3_d, L, gamma_vtp, mag)
figure;
%hold on;
plot3(r(1,:), r(2,:), r(3,:), 'MarkerSize', 5 ,'Marker', 'o');
hold on;
scatter3(P(1), P(2), P(3));
v1 = [P(1)+b3_d(1)*mag, P(2)+b3_d(2)*mag, P(3)+b3_d(3)*mag];
v2 = [P(1), P(2), P(3)];
v = [v2;v1];
plot3(v(:,1), v(:,2), v(:,3));

[x, y, z] = sphere;
h = surf(x*L+P(1), y*L+P(2), z*L+P(3));
set(h, 'FaceAlpha', 0.375, 'EdgeColor', 'none');

scatter3(r(1,gamma_vtp), r(2,gamma_vtp), r(3,gamma_vtp), 'r');
hold off;
grid on;
axis equal
xlabel('Position in X (m)')
ylabel('Position in Y (m)')
zlabel('Position in Z (m)')
end