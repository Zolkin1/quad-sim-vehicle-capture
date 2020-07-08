r = [2;1;0];
v = [0;0;0];

for k = 2:length(acc_hist)
	v(:, k) = v(:, k-1) + acc_hist(:,k)*dt;
	r(:, k) = r(:,k-1) + v(:,k-1)*dt;
end