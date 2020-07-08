function zeta = sumOfForces(rotMat, g, mass, thrust, externalForce)
% Returns an aceleration vector in A of the acceleration of B wrt to A

zeta = 1/mass * (mass*g*[0; 0; 1] + rotMat*(thrust - externalForce));
end