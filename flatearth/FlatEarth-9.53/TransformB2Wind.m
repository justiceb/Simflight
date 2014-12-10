function [TransMatrixB2Wind]=TransformB2Wind(alpha,beta)
% function [TransMatrixB2Wind]=TransformB2Wind(alpha,beta)
% Generate the transformation matrix to transform a vector from
% body-axis coordinates to wind-axis coordinates.
% V(wind)=TransMatrixB2Wind*V(body)
calpha=cos(alpha); 
salpha=sin(alpha);
cbeta=cos(beta);
sbeta=sin(beta);
% See Stevens and Lewis, p.63, eqn 2.3-2b.
TransMatrixB2Wind=[ calpha*cbeta, sbeta, salpha*cbeta;
                   -calpha*sbeta, cbeta,-salpha*sbeta;
				   -salpha,       0,     calpha];
