function path_indexret = getClosestPoint2(rc, r, path_index)
gamma = path_index+1;
d_prev_1 = norm(rc - r(:, path_index));
d_prev_2 = 1000;
minNotFound = true;

while minNotFound && gamma < length(r)
	d = norm(rc - r(:, gamma));
	if d_prev_1 < d_prev_2 && d_prev_1 < d% && d_prev_1 <= abs(norm(rc - r(:, path_index)))
		minNotFound = false;
	else
		gamma = gamma + 1;
	end
	d_prev_2 = d_prev_1;
	d_prev_1 = d;
end
% d = d_prev_1;
if gamma ~= path_index+1
	gamma = gamma-1;
end
if gamma >= length(r)
	gamma = path_index+1;
end
if gamma > length(r)
	gamma = length(r);
end
path_indexret = gamma;
end