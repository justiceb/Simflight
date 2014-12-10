function [VI] = interlim3(X,Y,Z,V,XI,YI,ZI)

% *********************************************
% interlim3 - 	3-D Interpolate with checking 
% 		(and modification) of limits
% *********************************************     
%
% Version 3.0 2/15/02. 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%%
% A&AE 421 Fall 2001
% Matthew Wysel
%
%  VI = INTERP3(X,Y,Z,V,XI,YI,ZI) interpolates to find VI, the values
%   of the underlying 3-D function V at the points in arrays XI,YI
%   and ZI. XI,YI,ZI must be arrays of the same size or vectors.
%   Vector arguments that are not the same size are passed through
%   MESHGRID.  Arrays X,Y and Z specify the points at which the data
%   V is given. FOR MORE HELP SEE interp3.m
%
% In addition to performing 3-D interpolation, this function file 
% checks that each XI, YI, and ZI  is within the range of X, Y, Z 
% respectively. If it is, then the value is passed without 
% alteration, if not then it sets the outlying value to the 
% maximum allowable value and returns a warning to the screen.
%
% For example, if XI = 15 and length(X) = 10 then XI would be 
% set to 10 and a warning would be sent to the screen
%
% See also INTERP3 - MATLABs generic 3-D interpolation.

% Start of an attempt at making the variables matrix compatable
%[m,n] = size(XI)
%for i:M for j:n
%	if XI(i,j) > X(i,j)
%   	XI(i,j) = X(i,j);
%   	warning([num2str(XI(i,j)),' converted to ',num2str(X)
%   end
%end end

if XI > max(X)
 	warning([num2str(XI),' exceeded interpolation limits'])
 	warning([num2str(XI),' was converted to ',num2str(max(X))])
  	XI = max(X);
 elseif XI < min(X)   
 	warning([num2str(XI),' exceeded interpolation limits'])
 	warning([num2str(XI),' was converted to ',num2str(min(X))])
   XI = min(X);
end

if YI > max(Y)
 	warning([num2str(YI),' exceeded interpolation limits'])
 	warning([num2str(YI),' was converted to ',num2str(max(Y))])
  	YI = max(Y);
 elseif YI < min(Y)   
 	warning([num2str(YI),' exceeded interpolation limits'])
 	warning([num2str(YI),' was converted to ',num2str(min(Y))])
   YI = min(Y);
end

if ZI > max(Z)
 	warning([num2str(ZI),' exceeded interpolation limits'])
 	warning([num2str(ZI),' was converted to ',num2str(max(Z))])
  	ZI = max(Z);
 elseif ZI < min(Z)   
 	warning([num2str(ZI),' exceeded interpolation limits'])
 	warning([num2str(ZI),' was converted to ',num2str(min(Z))])
   ZI = min(Z);
end

VI = interp3(X,Y,Z,V,XI,YI,ZI);

return
