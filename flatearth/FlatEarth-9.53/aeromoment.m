function [La,Ma,Na,Croll,Cpitch,Cyaw]=aeromoment(x,u,constant,alpha,beta,Vt,qbar,alphadot,CX,CY,CZprime);
%  function [La,Ma,Na,Croll,Cpitch,Cyaw]=aeromoment(x,u,constant,alpha,beta,Vt,qbar,alphadot,CX,CY,CZprime);
%  Version 1.0, 2/14/01
%   Compute aircraft aerodynamic moments in the body axis system about the c.g.
%   x=[U,V,W,P,Q,R,Phi,Theta,Psi,Xprime,Yprime,Hprime]
%   Units are [ft/sec, ft/sec, ft/sec, rad/sec, rad/sec, rad/sec, rad, rad, rad, ft, ft, ft]
%   u=[deltaE,deltaA,deltaR,deltaT]
%   Units are [rad,rad,rad,bhp]
%   constant(20)= S, wing area, ft^2
%   constant(21)= cbar, mean geometric chord, ft
%   constant(22)= b, wing span, ft
% Rolling Moment
%   constant(39)= Cl0
%   constant(40)= Clbeta
%   constant(41)= CldeltaA
%   constant(42)= CldeltaR
%   constant(43)= ClP
%   constant(44)= ClR
% Pitching Moment
%   constant(45)= CM0
%   constant(46)= Cmalpha
%   constant(47)= CMdeltaE
%   constant(48)= CMalphadot
%   constant(49)= CMQ
% Yawing Moment
%   constant(50)= CN0
%   constant(51)= CNbeta
%   constant(52)= CNdeltaA
%   constant(53)= CNdeltaR
%   constant(54)= CNP
%   constant(55)= CNR
% Reference positions
%   constant(56)= XbarRef, nondimensional
%   constant(57)= XbarCG,  nondimensional
%   constant(31)= CLalphadot

Croll=constant(39)+constant(40)*beta+constant(41)*u(2)+constant(42)*u(3)+...
      (constant(22)/(2*Vt))*(constant(43)*x(4)+constant(44)*x(6));
% Croll=constant(41)*u(2);
% [Croll constant(39) constant(40)*beta constant(41)*u(2) constant(42)*u(3) (constant(22)/(2*Vt))*(constant(43)*x(4)+constant(44)*x(6))]
% The moments below are about arbitrary reference point given by XbarRef.
CpitchRef=constant(45)+constant(46)*alpha+constant(47)*u(1)+...
          (constant(21)/(2*Vt))*(constant(48)*alphadot+constant(49)*x(5));
CyawRef=constant(50)+constant(51)*beta+constant(52)*u(2)+constant(53)*u(3)+...
          (constant(22)/(2*Vt))*(constant(54)*x(4)+constant(55)*x(6));
% Correct pitch and yaw moments to the c.g. position given by XbarCG.
CZ=CZprime-constant(31)*alphadot*constant(21)/(2*Vt); % First, add in the CLalphadot effect
Cpitch=CpitchRef+CZ*(constant(56)-constant(57));
Cyaw=  CyawRef-(constant(21)/constant(22))*CY*(constant(56)-constant(57));

% Compute the dimensional moments.
La=qbar*constant(20)*constant(22)*Croll;
Ma=qbar*constant(20)*constant(21)*Cpitch;
Na=qbar*constant(20)*constant(22)*Cyaw;
