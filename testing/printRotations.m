k = 1;
for test = 0:dt:endtime
	[R_des_hist(:, :, k), b1_hist(:, k), b2_hist(:, k), b3_hist(:, k)]
	k = k + 1;
end