clear
syms s1 s2 v1 v2 a j dt t;
v2 = v1 + a*dt + j*dt^2
eqn = int(v2,dt) + s1 - s2
