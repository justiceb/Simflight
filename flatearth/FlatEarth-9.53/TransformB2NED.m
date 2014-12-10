function [TransMatrixB2NED]=TransformB2NED(phi,theta,psi)
% function [TransMatrixB2NED]=TransformB2NED(phi,theta,psi)
% Generate the transformation matrix to transform a vector from
% body axis coordinates to North-East-Down coordinates.
% V(NED)=TransMatrixB2NED*V(body)
cphi=cos(phi); 
sphi=sin(phi);
ctheta=cos(theta);
stheta=sin(theta);
cpsi=cos(psi);
spsi=sin(psi);
% See Stevens and Lewis, eqn. 2.4-5, p.81
TransMatrixB2NED=[ ctheta*cpsi,-cphi*spsi+sphi*stheta*cpsi, sphi*spsi+cphi*stheta*cpsi;
                   ctheta*spsi, cphi*cpsi+sphi*stheta*spsi,-sphi*cpsi+cphi*stheta*spsi;
				  -stheta,      sphi*ctheta,                cphi*ctheta];
