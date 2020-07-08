function y = motorDynamics(Vin, thetadot)
% not sure I implemented this correctly
%y =  (-b*thetadot + k*((Vin - k*thetadot)/R))/J; % this does not have accurate voltage inputs, but works for now
y = -20*thetadot + 3333*Vin; % this is currently 2000rad/s max speed
% max thurst at 12 volts is 87.2 N
%omega(t) = x*v(t)
% omega(s)/v(s) = 2000*(1/(s+20))
% omega(s) = 2000*v(s)/(s+20)
% omega(s)*(s+20) = 2000*v(s)
% omegad(t) + 20*omega(t) = 2000*v(t)
end