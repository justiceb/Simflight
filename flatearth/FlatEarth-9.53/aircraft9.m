function  [sys,x0,str,ts] = aircraft9(t,x,u,flag,constant,xIC,Xa)
%   function [sys,x0,str,ts] = aircraft9(t,x,u,flag,constant,xIC,Xa)
%   
%   Version 9.3 4/8/07 (Includes steady winds and three accelerometers
%      located away from the c.g. by position vector Xa(3))
%   Xa is a 3x1 vector containing the boby axis location of the
%     three accelerometers wrt the c.g.
%   S-file for six degree of freedom aircraft simulation.
%   constant is a vector of constants that describe the aircraft.
%   xIC is the initial condition on the state vector.
% States
%   x=[U,V,W,P,Q,R,Phi,Theta,Psi,Xprime,Yprime,Hprime]
%   Units are [ft/sec, ft/sec, ft/sec, rad/sec, rad/sec, rad/sec, rad, rad, rad, ft, ft, ft]
% Inputs
%   u=[deltaE,deltaA,deltaR,bhp, Vwx, Vwy, Vwz]
%   Units are [rad,rad,rad,bhp, ft/sec, ft/sec, ft/sec]
%   Vwx, Vwy, Vwz are components of the wind vector in NED frame
% Outputs (22)
%   y=[U,V,W,P,Q,R,Phi,Theta,Psi,Xprime,Yprime,Hprime,Vt,alpha,beta,gamma,Xdot,Ydot,Hdot,Xacc,Yacc,Zacc]
%   Xdot,Ydot,Hdot are components of the inertial velocity vector in the NED frame.
%   Xacc, Yacc, Zacc are acceleromerer signals (ft/sec^2)
%   Units are [ft/sec, ft/sec, ft/sec, rad/sec, rad/sec, rad/sec, 
%   rad, rad, rad, ft, ft, ft, ft/sec, rad, rad, rad, ft/sec, ft/sec,
%   ft/sec, ft/sec^2, ft/sec^2, ft/sec^2]

switch flag,      
  case 0,         % Initialization
    [sys,x0,str,ts]=ACInitializeSizes(xIC);
  case 1,         % Compute derivatives of continuous states
    sys=ACDerivatives(t,x,u,constant) ;
  case 2,
    sys=ACUpdate(t,x,u);
  case 3,
      sys=ACOutputs(t,x,u,constant,Xa);  % Compute output vector
  case 4,                   % Compute time of next sample
    sys=ACGetTimeOfNextVarHit(t,x,u);
  case 9,                   % Finished. Do any needed 
    sys=ACTerminate(t,x,u);
  otherwise                 % Invalid input
    error(['Unhandled flag = ',num2str(flag)]);
end
 
%***********************************************************
%*                    ACInitializeSizes                   *
%***********************************************************
function [sys,x0,str,ts]=ACInitializeSizes(xIC)
% Return the sizes of the system vectors, initial 
% conditions, and the sample times and offets.
sizes = simsizes;   % Create the sizes structure
sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 22;
sizes.NumInputs      = 7;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);      % load sys with the sizes structure
x0  = xIC;    % Specify initial conditions for all states 
str = [];    % str is always an empty matrix
ts  = [0 0]; %initialize the array of sample times


%***********************************************************
%*                    ACDerivatives                       *
%***********************************************************
function sys=ACDerivatives(t,x,u,constant)
% Compute derivatives of continuous states
%   Version 2.0, 4/5/02 (Includes steady winds)
% Get elements of the state vector
U=x(1);   V=x(2);     W=x(3);   P=x(4);  Q=x(5);  R=x(6);
Phi=x(7); Theta=x(8); Psi=x(9); Xprime=x(10); Yprime=x(11); Hprime=x(12);
%   u=[deltaE,deltaA,deltaR,bhp, Vwx, Wwy, Vwz]]
%   Units are [rad,rad,rad,bhp, ft/sec, ft/sec, ft/sec]
%   Vwx, Vwy, Vwz are components of the wind vector in NED frame
% Compute air data
Vwx=u(5); Vwy=u(6); Vwz=u(7); % get NED components of steady wind
% get body axis components of steady wind
[TransMatrixNED2B]=TransformNED2B(Phi,Theta,Psi);
VwVector=TransMatrixNED2B*[Vwx,Vwy,Vwz]';
Uw=VwVector(1); Vw=VwVector(2); Ww=VwVector(3);
% compute components of airspeed
Uas=U-Uw; Vas=V-Vw; Was=W-Ww;
% compute wind related quantities
Vt=sqrt(Uas*Uas+Vas*Vas+Was*Was);  % ft/sec
alpha=atan2(Was,Uas);      % rad
beta=asin(Vas/Vt);         % rad
rho=0.00237691267925741*(1-6.87558563248308e-06*Hprime)^(4.25591641274834); %slug/ft3, troposphere
qbar=.5*rho*Vt*Vt;     %lbf/ft^2
% Get necessary constants
% Gravity varies with altitude
g= 1.4076431e16/(2.092565e7+Hprime)^2; 
mass=constant(3);
%W=mass*g;

S=constant(20); cbar=constant(21);
CLalphadot=constant(31);
c1=constant(11); c2=constant(12); c3=constant(13); c4=constant(14); 
c5=constant(15); c6=constant(16); c7=constant(17); c8=constant(18); 
c9=constant(19);

sphi=sin(Phi);
cphi=cos(Phi);
stheta=sin(Theta);
ctheta=cos(Theta);
ttheta=stheta/ctheta;

% Force equations of motion
% Get aerodynamic forces Fa
[Fax,Fay,Fazprime,CX,CY,CZprime]=aeroforce(x,u,constant,alpha,beta,Vt,qbar);
% Get thrust forces and Moments
[Ftx,Fty,Ftz,Lt,Mt,Nt]=thrust(u,constant,Vt);

Udot= R*V-Q*W-g*stheta     +(Fax+Ftx)/mass; % Stevens and Lewis, p.81
Vdot=-R*U+P*W+g*sphi*ctheta+(Fay+Fty)/mass;
aa=qbar*S*cos(alpha)*CLalphadot*cbar/(2*Vt*U);
alphadotfactor=(1/(1+aa/mass));
Wdot= (Q*U-P*V+g*cphi*ctheta+(Fazprime+Ftz)/mass)*alphadotfactor;

% Moment equations of motion
% Get aerodynamic moments
alphadot=(Uas*Wdot-Was*Udot)/(Uas*Uas+Was*Was);  % << change made here for steady winds
[La,Ma,Na,Croll,Cpitch,Cyaw]=aeromoment(x,u,constant,alpha,beta,Vt,qbar,alphadot,CX,CY,CZprime);

Pdot=(c1*R+c2*P)*Q      +c3*(La+Lt)+c4*(Na+Nt); 
Qdot=c5*P*R-c6*(P*P-R*R)+c7*(Ma+Mt);            
Rdot=(c8*P-c2*R)*Q      +c4*(La+Lt)+c9*(Na+Nt);

% Kinematic eqations
Phidot=  P+ttheta*(Q*sphi+R*cphi);
Thetadot=Q*cphi-R*sphi;
Psidot=  (Q*sphi+R*cphi)/ctheta;

% Navigation equations
[TransMatrixB2NED]=TransformB2NED(Phi,Theta,Psi);
xyzdot=TransMatrixB2NED*[U,V,W]';
Xdot= xyzdot(1);
Ydot= xyzdot(2);
Hdot=-xyzdot(3); % Note the negative sign. h is up, z is down.

% Assemble the entire derivative vector.
sys = [Udot,Vdot,Wdot,Pdot,Qdot,Rdot,Phidot,Thetadot,Psidot,Xdot,Ydot,Hdot];

%***********************************************************
%*                    ACUpdate                            *
%***********************************************************
function sys=ACUpdate(t,x,u)
% Compute update for discrete states. If necessary, check for
% sample time hits.
sys = [];    % Empty since this model has no discrete states.

%***********************************************************
%*                    ACOutputs                           *
%***********************************************************
function sys=ACOutputs(t,x,u,constant,Xa)
% function sys=ACOutputs(t,x,u)
% Compute output vector given current state, time, and input
%   Version 9.2.1 1/8/07 (Includes steady winds and lateral accelerometer)
% Outputs (22)
%   y=[U,V,W,P,Q,R,Phi,Theta,Psi,Xprime,Yprime,Hprime,Vt,alpha,beta,gamma,Xdot,Ydot,Hdot,Xacc,Yacc,Zacc]
%   Xdot,Ydot,Hdot are components of the inertial velocity vector in the NED frame.
%   Xacc, Yacc, Zacc are acceleromerer signals (ft/sec^2)
%   Units are [ft/sec, ft/sec, ft/sec, rad/sec, rad/sec, rad/sec, 
%   rad, rad, rad, ft, ft, ft, ft/sec, rad, rad, rad, ft/sec, ft/sec, ft/sec, ft/sec^2, ft/sec^2, ft/sec^2]
% States (12)
%   x=[U,V,W,P,Q,R,Phi,Theta,Psi,Xprime,Yprime,Hprime]
%   Units are [ft/sec, ft/sec, ft/sec, rad/sec, rad/sec, rad/sec, rad, rad, rad, ft, ft, ft]
% Inputs (7)
%   u=[deltaE,deltaA,deltaR,bhp, Vwx, Vwy, Vwz]]
%   Units are [rad,rad,rad,bhp, ft/sec, ft/sec, ft/sec]
%   Vwx, Vwy, Vwz are components of the wind vector in NED frame
U=x(1); V=x(2); W=x(3); phi=x(7); theta=x(8); psi=x(9);
P=x(4); Q=x(5); R=x(6);
% Compute air data
Vwx=u(5); Vwy=u(6); Vwz=u(7); % get NED components of steady wind
% get body axis components of steady wind
[TransMatrixNED2B]=TransformNED2B(phi,theta,psi);
VwVector=TransMatrixNED2B*[Vwx,Vwy,Vwz]';
Uw=VwVector(1); Vw=VwVector(2); Ww=VwVector(3);
% compute components of airspeed
Uas=U-Uw; Vas=V-Vw; Was=W-Ww;
% compute wind related quantities
Vt=sqrt(Uas*Uas+Vas*Vas+Was*Was);  % ft/sec
alpha=atan2(Was,Uas);      % rad
beta=asin(Vas/Vt);         % rad
a=cos(alpha)*cos(beta);
b=sin(phi)*sin(beta)+cos(phi)*sin(alpha)*cos(beta);
gamma=asin(a*sin(theta)-b*cos(theta)); %Stevens and Lewis p131, eqn 3.4-2
% Add the navigation equations to the outputs
[TransMatrixB2NED]=TransformB2NED(phi,theta,psi);
xyzdot=TransMatrixB2NED*[U,V,W]';
Xdot= xyzdot(1);
Ydot= xyzdot(2);
Hdot=-xyzdot(3); % Note the negative sign. h is up, z is down.
% Force equations of motion
mass=constant(3); Hprime=x(12);
rho=0.00237691267925741*(1-6.87558563248308e-06*Hprime)^(4.25591641274834); %slug/ft3, troposphere
qbar=.5*rho*Vt*Vt;     %lbf/ft^2
% Gravity varies with altitude
g= 1.4076431e16/(2.092565e7+Hprime)^2; 
mass=constant(3);
%W=mass*g;


mass=constant(3);  S=constant(20); cbar=constant(21);
CLalphadot=constant(31);
c1=constant(11); c2=constant(12); c3=constant(13); c4=constant(14); 
c5=constant(15); c6=constant(16); c7=constant(17); c8=constant(18); 
c9=constant(19);

sphi=sin(phi);
cphi=cos(phi);
stheta=sin(theta);
ctheta=cos(theta);
ttheta=stheta/ctheta;

% Get aerodynamic forces Fa
[Fax,Fay,Fazprime,CX,CY,CZprime]=aeroforce(x,u,constant,alpha,beta,Vt,qbar);
% Get thrust forces and Moments
[Ftx,Fty,Ftz,Lt,Mt,Nt]=thrust(u,constant,Vt);

% Get accelerometer at c.g.
% First some preliminaries
S=constant(20); cbar=constant(21); CLalphadot=constant(31);
aa=qbar*S*cos(alpha)*CLalphadot*cbar/(2*Vt*U);
alphadotfactor=(1/(1+aa/mass));
Udot= R*V-Q*W-g*stheta     +(Fax+Ftx)/mass; % Stevens and Lewis, p.81
Vdot=-R*U+P*W+g*sphi*ctheta+(Fay+Fty)/mass;
Wdot= (Q*U-P*V+g*cphi*ctheta+(Fazprime+Ftz)/mass)*alphadotfactor;
% Now find what accelerometers at c.g. measure
Xacc=(Fax+Ftx)/mass; % Longitudinal accelerometer at c.g.(ft/sec^2)
Yacc=(Fay+Fty)/mass; % Lateral accelerometer  at c.g.(ft/sec^2)
Zacc=(Fazprime-Wdot*aa+Ftz)/mass; % Z-accelerometer  at c.g.(ft/sec^2)

% Now correct for off c.g. location (Use position vector Xa[3])
%***
% First we must get Moment equations of motion
% Get aerodynamic moments
alphadot=(Uas*Wdot-Was*Udot)/(Uas*Uas+Was*Was);  % << change made here for steady winds
[La,Ma,Na,Croll,Cpitch,Cyaw]=aeromoment(x,u,constant,alpha,beta,Vt,qbar,alphadot,CX,CY,CZprime);
% Get angular acceleration
Pdot=(c1*R+c2*P)*Q      +c3*(La+Lt)+c4*(Na+Nt);
Qdot=c5*P*R-c6*(P*P-R*R)+c7*(Ma+Mt);
Rdot=(c8*P-c2*R)*Q      +c4*(La+Lt)+c9*(Na+Nt);
%****
% Second derivative of X vector from c.g to accelerometer location
ddXx=-(Q^2+R^2)*Xa(1)+(Q*P-Rdot)*Xa(2)+(R*P+Qdot)*Xa(3);
ddXy=-(P^2+R^2)*Xa(2)+(Q*P+Rdot)*Xa(1)+(Q*R-Pdot)*Xa(3);
ddXz=-(P^2+Q^2)*Xa(3)+(P*R-Qdot)*Xa(1)+(Q*R+Pdot)*Xa(2);
% Add in ddX to correct for off c.g. sensor location
Xacc=Xacc+ddXx;
Yacc=Yacc+ddXy;
Zacc=Zacc+ddXz;
sys = [U,V,W,x(4),x(5),x(6),phi,theta,psi,x(10),...
       x(11),x(12),Vt,alpha,beta,gamma,Xdot,Ydot,Hdot,Xacc,Yacc,Zacc];

%***********************************************************
%*                    ACGetTimeOfNextVarHit               *
%***********************************************************
function sys=ACGetTimeOfNextVarHit(t,x,u)
% Return the time of the next hit for this block.  Note that 
% the result is absolute time.  Note that this function is 
% only used when you specify a variable discrete-time sample
% time [-2 0] in the sample time array in sampleTime = 1;    
sys = [] ;

%************************************************************
%*                    ACTerminate                          *
%************************************************************
function sys=ACTerminate(t,x,u)
% Perform any necessary tasks at the end of the simulation
sys = [];

