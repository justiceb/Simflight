function [TransMatrixWind2B]=TransformWind2B(alpha,beta)
% function [TransMatrixWind2B]=TransformWind2B(alpha,beta)
% Generate the transformation matrix to transform a vector from
% wind-axis coordinates to body-axis coordinates.
% V(body)=TransMatrixWind2B*V(wind)
calpha=cos(alpha); 
salpha=sin(alpha);
cbeta=cos(beta);
sbeta=sin(beta);
% See Stevens and Lewis, p.63, eqn 2.3-2b transposed.
TransMatrixWind2B=[ calpha*cbeta, -calpha*sbeta, -salpha;
                    sbeta,         cbeta,         0;
				    salpha*cbeta, -salpha*sbeta,  calpha];
