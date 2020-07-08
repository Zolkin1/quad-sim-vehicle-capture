function [pathCoefs, path, arc_len, T, N, B, curvature, torsion, lookup_table] = makeGeoPath(obs_pos, obs_radi, obs_buffer, start, max_tilt, claw_length)
	max_tilt = pi/2 - max_tilt;
	
	diff = start - obs_pos;
	figure;
	subplot(1,2,2)
	[x_ell, y_ell, z_ell] = ellipsoid(obs_pos(1), obs_pos(2), obs_pos(3), obs_radi(1), obs_radi(2), obs_radi(3));
	surf(x_ell, y_ell, z_ell);
	hold on;

	if diff(3) < 0 % check if we are above or below the target
		if diff(1) == 0 && diff(2) == 0
			diff = [1, 0];
		end
		unit_xy = [diff(1) diff(2)]/norm([diff(1) diff(2)]);

		% calculate the location of the first waypoint
		targ_1 = [unit_xy(1)*(obs_radi(1)+(sign(unit_xy(1))*obs_buffer)) + obs_pos(1); unit_xy(2)*(obs_radi(2)+(sign(unit_xy(2))*obs_buffer)) + obs_pos(2); obs_pos(3)];
		scatter3(targ_1(1), targ_1(2), targ_1(3));
		
		% Second, find the point directly above the target
		x_2 = obs_pos(1) - targ_1(1);
		y_2 = obs_pos(2) - targ_1(2);

		max_z = tan(max_tilt)*(sqrt(x_2^2 + y_2^2));
		
		targ_2 = [x_2+targ_1(1); y_2+targ_1(2); max_z+obs_pos(3)];
		
		scatter3(targ_2(1), targ_2(2), targ_2(3));
	
	else
		targ_1 = [0 0 0];
		targ_2 = [obs_pos(1) obs_pos(2) obs_pos(3) + 2*obs_buffer]; % go to a set height above the target
		scatter3(targ_2(1), targ_2(2), targ_2(3));
	end
	scatter3(start(1), start(2), start(3));	
	
	% Third, lower onto the target
	targ_3 = [obs_pos(1); obs_pos(2); obs_pos(3)+claw_length];
	scatter3(targ_3(1), targ_3(2), targ_3(3));
	
	% Generate the curves
	stretch_1 = 1; % stretch factor on the control point
	s = 0:0.0001:1;

	p_01 = start';
	p_11 = targ_1;
	p_31 = targ_1;
	p_21 = [-(x_2/stretch_1)+targ_1(1); -(y_2/stretch_1)+targ_1(2); -(max_z/stretch_1)+obs_pos(3)];
	
	% Calculate the TNB information
	for k = 1:3
		B_1(:, k) = constructCurve(p_01(k), p_11(k), p_21(k), p_31(k), s);
		B_1T_old(:,k) = getTangent(p_01(k), p_11(k), p_21(k), p_31(k), s);
	end
	for k = 1:length(B_1T_old)
		B_1T(k,:) = B_1T_old(k,:)/norm(B_1T_old(k,:));	
	end
	
	for k = 2:length(B_1T)
		B_1N(k,:) = (B_1T(k,:)-B_1T(k-1,:))/0.0001;
		curvature1(k) = norm(B_1N(k,:))/norm(B_1T_old(k,:));
		B_1N(k,:) = B_1N(k,:)/norm(B_1N(k,:));
		B_1B(k,:) = cross(B_1T(k,:), B_1N(k,:))'/norm(cross(B_1T(k,:), B_1N(k,:))');
	end
	
	for k = 2:length(B_1T)
		torsion1(k) = -dot(B_1N(k,:),(B_1B(k,:)-B_1B(k-1,:))/0.0001);
	end
		
	plot3(B_1(:,1), B_1(:,2), B_1(:,3));
	
	stretch_2 = 6;
	p_02 = targ_1;
	p_12 = [(x_2/stretch_1)+targ_1(1); (y_2/stretch_1)+targ_1(2); (max_z/stretch_1)+obs_pos(3)];
	p_32 = targ_2;
	p_22 = [targ_2(1)+(targ_1(1)-targ_2(1))/stretch_2; targ_2(2)+(targ_1(2)-targ_2(2))/stretch_2; targ_2(3) + (targ_1(3)-targ_1(3))/stretch_2];%[targ_3(1); targ_3(2); (targ_3(3)/stretch_2)+targ_2(3)];
	
	% Calculate the TNB information for the second curve
	for k = 1:3
		B_2(:, k) = constructCurve(p_02(k), p_12(k), p_22(k), p_32(k), s);
	end
	for k = 1:3
		B_2(:, k) = constructCurve(p_02(k), p_12(k), p_22(k), p_32(k), s);
		B_2T_old(:,k) = getTangent(p_02(k), p_12(k), p_22(k), p_32(k), s);
	end
	for k = 1:length(B_2T_old)
		B_2T(k,:) = B_2T_old(k,:)/norm(B_2T_old(k,:));	
	end
	
	for k = 2:length(B_2T)
		B_2N(k,:) = (B_2T(k,:)-B_2T(k-1,:))/0.0001;
		curvature2(k) = norm(B_2N(k,:))/norm(B_2T_old(k,:));
		B_2N(k,:) = B_2N(k,:)/norm(B_2N(k,:));
		B_2B(k,:) = cross(B_2T(k,:), B_2N(k,:))'/norm(cross(B_2T(k,:), B_2N(k,:))');
	end
	
	for k = 2:length(B_2T)
		torsion2(k) = -dot(B_2N(k,:),(B_2B(k,:)-B_2B(k-1,:))/0.0001);
	end
	
	scatter3(p_12(1), p_12(2), p_12(3));
	
	plot3(B_2(:,1), B_2(:,2), B_2(:,3));
	
	stretch_3 = 1;
	p_03 = targ_2;
	p_13 = [targ_3(1); targ_3(2); -(targ_3(3)/stretch_2)+targ_2(3)];
	p_33 = targ_3;
	p_23 = [targ_3(1); targ_3(2); targ_3(3)*stretch_3];
	
	for k = 1:3
		B_3(:, k) = constructCurve(p_03(k), p_13(k), p_23(k), p_33(k), s);
	end

	plot3(B_3(:,1), B_3(:,2), B_3(:,3));
	
	arc_len = [calcArcLen(B_1(:,1), B_1(:,2), B_1(:,3)), calcArcLen(B_2(:,1), B_2(:,2), B_2(:,3)), calcArcLen(B_3(:,1), B_3(:,2), B_3(:,3))];
	pathCoefs = [p_01, p_11, p_21, p_31, p_02, p_12, p_22, p_32, p_03, p_13, p_23, p_33];
	path = [B_1; B_2; B_3];
	T = [B_1T; B_2T];
	N = [B_1N; B_2N];
	B = [B_1B; B_2B];
	curvature = [curvature1, curvature2];
	torsion = [torsion1, torsion2];
	
	lookup_table = makeArcLenLookUpTable(path(1:20002,:), 1);
	
	hold off;
	axis equal;
	%title('Geometric Path of the Quadrotor');
	hold on
	h = 2:300:20002;
	for w = 2:length(h)
		v1 = [path(h(w),1), path(h(w),2), path(h(w),3)];
		v2 = [path(h(w),1)+T(h(w),1), path(h(w),2)+T(h(w),2), path(h(w),3)+T(h(w),3)];
		v = [v1; v2];
		plot3(v(:,1), v(:,2), v(:,3), 'Color', 'r', 'MarkerSize', 3);

		v1 = [path(h(w),1), path(h(w),2), path(h(w),3)];
		v2 = [path(h(w),1)+N(h(w),1), path(h(w),2)+N(h(w),2), path(h(w),3)+N(h(w),3)];
		v = [v1; v2];
		plot3(v(:,1), v(:,2), v(:,3), 'Color', 'b', 'MarkerSize', 3);

		v1 = [path(h(w),1), path(h(w),2), path(h(w),3)];
		v2 = [path(h(w),1)+B(h(w),1), path(h(w),2)+B(h(w),2), path(h(w),3)+B(h(w),3)];
		v = [v1; v2];
		plot3(v(:,1), v(:,2), v(:,3), 'Color', 'k', 'MarkerSize', 3);
	end
	hold off
	xlabel('Position in X (m)')
	ylabel('Position in Y (m)')
	zlabel('Position in Z (m)')

	subplot(1,2,1)
	plot3(B_1(:,1), B_1(:,2), B_1(:,3), 'LineWidth', 2.5);
	hold on;
	plot3(B_2(:,1), B_2(:,2), B_2(:,3), 'LineWidth', 2.5);
	plot3(B_3(:,1), B_3(:,2), B_3(:,3), 'LineWidth', 2.5);
	surf(x_ell, y_ell, z_ell);
	hold off;
	axis equal
	grid on
	xlabel('Position in X (m)')
	ylabel('Position in Y (m)')
	zlabel('Position in Z (m)')
	
end

function b = constructCurve(p_0, p_1, p_2, p_3, s)
	b = (1-s).^3.*p_0 + 3.*(1-s).^2.*s.*p_1 + 3.*(1-s).*s.^2.*p_2 + s.^3.*p_3;
end

function arc_len = calcArcLen(x, y, z)
	c_f = length(x);
	arc_len = 0;
	for k = 2:c_f
		arc_len = arc_len + sqrt((x(k)-x(k-1))^2 + (y(k)-y(k-1))^2 + (z(k)-z(k-1))^2);
	end
end


function T = getTangent(p_0, p_1, p_2, p_3, s)
T = 3*(1-s).^2.*(p_1-p_0) + 6*(1-s).*s.*(p_2 - p_1) + 3.*s.^2*(p_3-p_2);
end

function table = makeArcLenLookUpTable(path, dt)
table(1) = 0;
for k = 2:length(path)
	table(k) = table(k-1) + sqrt(((path(k,1)-path(k-1,1))/dt)^2 + ((path(k,2)-path(k-1,2))/dt)^2 + ((path(k,3)-path(k-1,3))/dt)^2);
end
end
