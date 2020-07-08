function in_bounds = check_bounds(position, x_max, y_max, z_max, time, prev_check)
% Assume the room is a rectangular prism where the top corner (furthest
% from the origin) is at [x_max, y_max, z_max]. Assume the opposite corner
% is the origin at [0, 0, 0]
in_bounds = [true; true; true];

if (position(1) < 0 || position(1) > x_max)
	if (prev_check(1)) % first time going out of bounds, so print
		fprintf('Out of bounds error (x)! Time: %i \n', time);
	end
	in_bounds(1) = false;
elseif (~prev_check(1) && in_bounds(1))
	fprintf('No longer out of bounds (x). Time: %i \n', time);
end

if (position(2) < 0 || position(2) > y_max)
	if (prev_check(2))
		fprintf('Out of bounds error (y)! Time: %i \n', time);
	end
	in_bounds(2) = false;
elseif (~prev_check(2) && in_bounds(2))
	fprintf('No longer out of bounds (y). Time: %i \n', time);
end

if (position(3) < 0 || position(3) > z_max)
	if (prev_check(3))
		fprintf('Out of bounds error (z)! Time: %i \n', time);
	end
	in_bounds(3) = false;
elseif (~prev_check(3) && in_bounds(3))
	fprintf('No longer out of bounds (z). Time: %i \n', time);
end


end