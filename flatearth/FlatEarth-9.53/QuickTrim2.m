function [x,u,CL,CD,CM,alphadeg]=QuickTrim2(Vt,Hp,constant,Vwx,Vwy,Vwz)
% function [x,u,CL,CD,CT,CM,alphadeg]=QuickTrim(Vt,Hp,constant)
% Quick and dirty trim routine.
% Version 2.0 4/5/02 (includes steady wids)
%   Vwx, Vwy, Vwz are components of the wind vector in NED frame
% constant is an array of aircraft specific sconstants.
% Vt and Hp are trim true airspeed (ft/sec) and pressure altitude (ft).
% x is the 12x1 state vector which is initialized in this routine.
% u is the 4x1 control vector which is also initialized by this routine.
% CL and CD are trim lift and drag coefficient. 
% alphadeg is the trim angle of attack in degrees.
% The trim condition is assumed to be straight line flight with
% wings level, constant attitude, 
% constant altitude, and nonaccelerating.
% It assumes CY0, Cl0, CN0 are all zero and an initial northerly airspeed.
% No sideslip is allowed for this trim routine.
% Flight in the troposphere is assumed.
% This is not the most general trim routine. It is a quick-trim routine.
% It also assumes that
% Xcg=xref and that dT=0 (thrust inclination angle) and that
% alpha is very small.

rho=0.00237691267925741*(1-6.87558563248308e-06*Hp)^(4.25591641274834); %slug/ft3, troposphere
g= 1.4076431e16/(2.092565e7+Hp)^2; 
mass=constant(3);
Weight=mass*g;
S=constant(20); cbar=constant(21);
rad2deg=180/pi;

qbar=.5*rho*Vt*Vt;     %lbf/ft^2
CL=Weight/(qbar*S);
CL0=     constant(28);
CLalpha= constant(29);
CLdeltaE=constant(30);
CM0=     constant(45);
CMalphaREF= constant(46);
CMalpha=CMalphaREF-CLalpha*(constant(56)-constant(57));
CMdeltaE=constant(47);
eta_p=constant(8);
T=[CLalpha,CLdeltaE;
   CMalpha,CMdeltaE];
Y=[CL-CL0;-CM0];
X=inv(T)*Y;
alpha=X(1);
alphadeg=alpha*rad2deg;
deltaE=X(2);
deltaEdeg=deltaE*rad2deg;
CDm= constant(25);
k=   constant(26);
CLdm=constant(27);
CD=CDm+k*(CL-CLdm)^2;
D=qbar*S*CD;
bhp=Vt*D/(550*eta_p);

% compute components of airspeed in NED frame
Vxas=Vt;
Vyas=0;
Vzas=0;
% compute components of inertial velocity in NED frame
Xdot=Vxas+Vwx;
Ydot=Vyas+Vwy;
Zdot=Vzas+Vwz;
phi=0 ;
theta=alpha;
psi=0;
% get body axis components of inertial velocity
[TransMatrixNED2B]=TransformNED2B(phi,theta,psi);
ViVector=TransMatrixNED2B*[Xdot,Ydot,Zdot]';
U=ViVector(1); V=ViVector(2); W=ViVector(3);

x=[U,V,W,0,0,0,phi,theta,psi,0,0,Hp]';
u=[deltaE,0,0,bhp,Vwx,Vwy,Vwz]';
%CM=0;  % There is no moment due to thrust to balance.
CL=CL0+CLalpha*alpha+CLdeltaE*deltaE
Werror=qbar*S*CL-Weight
CM=CM0+CMalpha*alpha+CMdeltaE*deltaE
Thrust=550*bhp*eta_p/Vt
ThrustError=D-Thrust