syms p0x p1x p2x p3x p0y p1y p2y p3y p0z p1z p2z p3z k
Bx = (1-k)^3*p0x + 3*(1-k)^2*k*p1x + 3*(1-k)*k*p2x + k^3*p3x;
Bxd = diff(Bx,k);

By = (1-k)^3*p0y + 3*(1-k)^2*k*p1y + 3*(1-k)*k*p2y + k^3*p3y;
Byd = diff(By,k);

Bz = (1-k)^3*p0z + 3*(1-k)^2*k*p1z + 3*(1-k)*k*p2z + k^3*p3z;
Bzd = diff(Bz,k);

v_mag = Bxd^2 + Byd^2 + Bzd^2;
arc_len = int(v_mag,k)