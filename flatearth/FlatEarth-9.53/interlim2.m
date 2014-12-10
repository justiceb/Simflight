function [VI] = interlim3(X,Y,V,XI,YI)
% function [VI] = interlim3(X,Y,V,XI,YI)
% *********************************************
% interlim3 - 	3-D Interpolate with checking 
% 		(and modification) of limits
% *********************************************     
%
% Version 4.0 3/23/05. 
% Dominick Andrisani
% This version corrects an error when XI, YI are vectors
%
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%%
% A&AE 421 Fall 2001
% Matthew Wysel
%
%   INTERP2 2-D interpolation (table lookup).
%    ZI = INTERP2(X,Y,Z,XI,YI) interpolates to find ZI, the values of the
%    underlying 2-D function Z at the points in matrices XI and YI.
%    Matrices X and Y specify the points at which the data Z is given. 
%    FOR MORE HELP SEE interp3.m
%
% In addition to performing 2-D interpolation, this function file 
% checks that each XI and YI is within the range of X and Y
% respectively. If it is, then the value is passed without 
% alteration, if not then it sets the outlying value to the 
% maximum allowable value and returns a warning to the screen.
%
% For example, if XI = 15 and length(X) = 10 then XI would be 
% set to 10 and a warning would be sent to the screen
%
% See also INTERP2 - MATLABs generic 2-D interpolation.

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

j=find(YI>max(Y));
nj=length(j);
if nj>0
    for k=1:nj
        warning on
        warning([num2str(YI(j(k))),' exceeded interpolation limits'])
        warning backtrace
        warning([num2str(YI(j(k))),' was converted to ',num2str(max(Y))])
        YI(j) = max(Y);
        warning off  
    end
end

j=find(YI<min(Y));
nj=length(j);
if nj>0
    for k=1:nj
        warning on
        warning([num2str(YI(j(k))),' exceeded interpolation limits'])
        warning backtrace
        warning([num2str(YI(j(k))),' was converted to ',num2str(min(Y))])
        YI(j) = min(Y);
        warning off  
    end
end


VI = interp2(X,Y,V,XI,YI);

return
