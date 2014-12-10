function [TransMatrixNED2B]=TransformNED2B(phi,theta,psi)
% function [TransMatrixNED2B]=TransformNED2B(phi,theta,psi)
% Generate the transformation matrix to transform a vector from
% North-East-Down coordinates to body axis coordinates.
% V(body)=TransMatrixNED2B*V(NED)
cphi=cos(phi);
sphi=sin(phi); 
ctheta=cos(theta);
stheta=sin(theta);
cpsi=cos(psi);
spsi=sin(psi);	 
% See Stevens and Lewis, eqn. 2.4-5, p.81
TransMatrixNED2B=[  ctheta*cpsi,                 ctheta*spsi,               -stheta;
                   -cphi*spsi+sphi*stheta*cpsi,  cphi*cpsi+sphi*stheta*spsi, sphi*ctheta;
				    sphi*spsi+cphi*stheta*cpsi, -sphi*cpsi+cphi*stheta*spsi, cphi*ctheta];
