function [Fax,Fay,Fazprime,CX,CY,CZprime]=aeroforce(x,u,constant,alpha,beta,Vt,qbar)
%   function [Fax,Fay,Fazprime,CX,CY,CZprime]=aeroforce(x,u,constant,alpha,beta,Vt,qbar)
%   Version 1.0, 2/14/01
%   Compute aircraft aerodynamic forces in the body axis system.
%   FAvector=Fax*i+Fay*j+Faz*k
%   CX, CY, CZprime are nondimensional versions of Fax, Fay, and Fazprime.
%   x=[U,V,W,P,Q,R,Phi,Theta,Psi,Xprime,Yprime,Hprime]
%   Units are [ft/sec, ft/sec, ft/sec, rad/sec, rad/sec, rad/sec, rad, rad, rad, ft, ft, ft]
%   u=[deltaE,deltaA,deltaR,deltaT]
%   Units are [rad,rad,rad,bhp]
%   Fazprime does not contain the CLalphadot term.
%   constant(20)= S, wing area, ft^2
%   constant(22)= b, wing span, ft
%   constant(28)= CL0
%   constant(29)= CLalpha
%   constant(30)= CLdeltaE
%   constant(31)= CLalphadot
%   constant(32)= CLQ
%   constant(33)= CY0
%   constant(34)= CYbeta
%   constant(35)= CYdeltaA
%   constant(36)= CydeltaR
%   constant(37)= CYP
%   constant(38)= CYR
CLstatic=constant(28)+constant(29)*alpha+constant(30)*u(1);
CD=constant(26)*(CLstatic-constant(27))^2+constant(25);
CL=CLstatic+constant(32)*x(5)*constant(21)/(2*Vt); % Add in the damping in pitch term
CYstatic=constant(33)+constant(34)*beta+constant(35)*u(2)+constant(36)*u(3);
CY=CYstatic+constant(37)*x(4)*constant(22)/(2*Vt)+constant(38)*x(6)*constant(22)/(2*Vt);
[TransMatrixWind2B]=TransformWind2B(alpha,beta);
CF=TransMatrixWind2B*[-CD;CY;-CL];
CX=CF(1);
CY=CF(2);
CZprime=CF(3); % CZzprime does not contain the CLalphadot term.
F=CF*qbar*constant(20);
Fax=F(1);
Fay=F(2);
Fazprime=F(3); % Fazprime does not contain the CLalphadot term.
