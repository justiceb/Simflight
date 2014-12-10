function [rho]=rhofun(h)
%  function [rho]=rhofun(h)
%  Standard Atmosphere Computations in English Units
%  for the 1976 standard atmosphere up to 230,000 ft.
%  Author: Ilan Kroo (kroo@leland.stanford.edu)  31 Dec 95
%  Converted to MATLAB by D. Andrisani, 2 Nov 99
%  Scalar input h is geometric altitude in feet
%
%  Output   Units
%  rho      slug/ft^3     (density)
%
%  Because h must be a scalar, the arithmetic operations
%  used below (e.g., ^)are the normal arithmetic operations.


RHOSL = 0.00237689;   % slug/ft^3
saSigma = 1.0;

   if  h<232940 & h>=167323
      saSigma = ( 0.79899-h/606330)^11.20114;
   end

  
   if h<167323 & h>=154199
      saSigma = 0.00116533*exp((h-154200)/-25992);
   end
  
   if h<154199 & h>=104987
      saSigma = (0.857003+h/190115)^-13.20114;
   end
  
   if h<104987 & h>=65617
      saSigma = (0.978261+h/659515)^-35.16319;
   end
  
  if h<65617 & h>=36089
      % This is the stratosphere.
      saSigma = 0.297076 *exp((36089-h)/20806);
   end
  
  if h<36089
      % This is the troposphere.
      saSigma = (1.0-h/145442)^4.255876;
   end

rho = RHOSL * saSigma;   % slug/ft^3
