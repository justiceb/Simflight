function [YI] = interlim1(X,Y,XI)
% function [YI] = interlim1(X,Y,XI)
% *********************************************
% interlim1 - 	1-D Interpolate with checking 
% 		(and modification) of limits
% *********************************************     
%
% Version 4.0 3/22/05. 
% This version corrects a mistake when XI and YI are vectors
% Dominick Andrisani
%
% A&AE 421 Fall 2001
% Matthew Wysel
% Jaret Matthews
%
%   INTERLIM1 1-D interpolation (table lookup).   
%   YI = INTERP1(X,Y,XI) interpolates to find YI, the values of
%   the underlying function Y at the points in the vector XI.
%   The vector X specifies the points at which the data Y is
%   given. If Y is a matrix, then the interpolation is performed
%   for each column of Y and YI will be length(XI)-by-size(Y,2).
%   Out of range values are returned as NaN. 
%    FOR MORE HELP SEE interp2.m
%
% In addition to performing 1-D interpolation, this function file 
% checks that each XI is within the range of X. If it is, then 
% the value is passed without alteration, if not then it sets the 
% outlying value to the maximum allowable value and returns a 
% warning to the screen.
%
% All arrays must be vectors (ie ndim(variable)=1)
%
% For example, if XI = 15 and length(X) = 10 then XI would be 
% set to 10 and a warning would be sent to the screen
%
% See also INTERP1 - MATLABs generic 1-D interpolation.

j=find(XI>max(X));
nj=length(j);
if nj>0
    for k=1:nj
        warning on
        warning([num2str(XI(j(k))),' exceeded interpolation limits'])
        warning backtrace
        warning([num2str(XI(j(k))),' was converted to ',num2str(max(X))])
        XI(j) = max(X);
        warning off  
    end
end

j=find(XI<min(X));
nj=length(j);
if nj>0
    for k=1:nj
        warning on
        warning([num2str(XI(j(k))),' exceeded interpolation limits'])
        warning backtrace
        warning([num2str(XI(j(k))),' was converted to ',num2str(min(X))])
        XI(j) = min(X);
        warning off  
    end
end

YI = interp1(X,Y,XI);

return
