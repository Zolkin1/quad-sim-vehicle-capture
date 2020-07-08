function zeta = attitudeChange(omega, tau, I)
	zeta = [omega; inv(I) * (-cross(omega, I*omega) + tau)];
end