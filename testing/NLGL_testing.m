clear;
close all;

obs_pos = [1 1 8];
obs_radi = [0.5 0.5 0.25];
buffer = 0.75;
start_pos = [1 1 0];
tilt_angle = pi/6;
claw_length = 0.4;
ds = 0.001;


[pathCoefs, path, arc_len] = makeGeoPath(obs_pos, obs_radi, buffer, start_pos, tilt_angle, claw_length);


gamma_prev = 425;
L = 0.5;

[b3_d, gamma] = NLGL([1.3, 1, 1], path(1:10001, :)', L, gamma_prev)