function [s, len_traveled, dist] = getClosestPoint(pos, path, pathCoefs, arc_len, prev)
	% Find the location on the curve that is closest to the position point
	% Then determine the arc length at that point. Then divide by the total
	% arc length to get the index (need to convert to an int);
	s = prev; %cast(guess/length(path), 'double');

	gamma = 0.0001;
	d_prime = 1;
	
	while abs(d_prime) > 0.001
		% s is always between 0 and 1
		loc = evalPath(pathCoefs, s);
		slope = evalSlope(pathCoefs, s);
		
		d_prime = 2*(pos(1) - loc(1))*slope(1) + 2*(pos(2) - loc(2))*slope(2) + 2*(pos(3) - loc(3))*slope(3);
		s = s + gamma*d_prime;
	end
	if s < prev
		s = prev + 0.001;
	end
	len_traveled = s*arc_len; % this only holds for small deviations of the quad
	dist = sqrt((pos(1)-loc(1))^2 + (pos(2)-loc(2))^2 + (pos(3)-loc(3))^2);
end

function loc = evalPath(pathCoefs, s)
for k = 1:3
	loc(k) = (1-s)^3*pathCoefs(k, 1) + 3*(1-s)^2*s*pathCoefs(k, 2) + 3*(1-s)*s^2*pathCoefs(k, 3) + s^3*pathCoefs(k, 4);
end
end

function slope = evalSlope(pathCoefs, s)
for k = 1:3	
	slope(k) = 3*(1-s)^2*(pathCoefs(k, 2)-pathCoefs(k, 1)) + 6*(1-s)*s*(pathCoefs(k, 3)-pathCoefs(k, 2)) + 3*s^2*(pathCoefs(k, 4)-pathCoefs(k, 3));
end
end