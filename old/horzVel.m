function v_dot_hat = horzVel(g, P_h, zw, R_hat, D, v_hat_h, k_w, v_h)
	v_dot_hat = -g*P_h*(R_hat*zw + R_hat*D*R_hat'*P_h'*v_hat_h) - k_w*(v_hat_h - v_h);
end