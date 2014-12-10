%************************************************************************
% Step2_Trim.m      Version 9.51 1/25/2010
%
%************************************************************************
% OBJECTIVE: Trim the aircraft and develop initial conditions for the
%            SIMULINK nonlinear simulation.
%
% INPUTS: the array called constant
% OUTPUTS: In the Matlab workspace will be defined initial conditions
% on the state and controls (xIC and uIC). The array constant, xIC and uIC
% are all required by the SIMULINK nonlinear simulation.
%
%

% Quick trim the aircraft
disp(' '); disp(' ')
disp('Step2_Trim <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<')
disp(' ')

disp(aircraft); % Display aircraft identification
% Establish a trim condition (initial condition).
Vt=constant(58); % Trim airspeed (ft/sec)
Hp=constant(59); % Trim altitude (ft)
% Alternately you can make changes to the trim airspeed and 
% altitude here if the nondimensional stability and control
% derivatives in the array called constant are not strongly
% dependent on trim airspeed ar alitude, as is often the case.

% At this point in the script if you want you can change the c.g. location as follows.
% constant(57)=.25;  		
% XbarCG,  nondimensional, measured aft from leading edge of wing mean aerodynamic chord.

s1=['Trim airspeed=',num2str(Vt),' ft/sec, '];
s2=['Trim altitude=',num2str(Hp),' ft, '];
s3=['c.g.=',num2str(constant(57)),' cbar.'];
disp(' '); disp([s1 s2 s3]); disp(' ');

% Define the steady winds
disp('NED omponents of the steady wind (ft/sec)')
VwindKnots=0;
Vwx=-VwindKnots*1.687808 %ft/sec, - sign means headwind
Vwy=0
Vwz=0 

% Location of accelerometer package
disp('Location of accelerometer package (ft)')
Xa(1)=0; % x-location of accelerometer wrt c.g. (ft)
Xa(2)=0; % y-location of accelerometer wrt c.g. (ft)
Xa(3)=0  % z-location of accelerometer wrt c.g. (ft)

% Trim the aircraft
disp('Trim conditions')
echo on
% States x
%   x=[U,V,W,P,Q,R,Phi,Theta,Psi,Xprime,Yprime,Hprime]
%   Units are [ft/sec, ft/sec, ft/sec, rad/sec, rad/sec, rad/sec, rad, rad, rad, ft, ft, ft]
% Inputs u
%   u=[deltaE,deltaA,deltaR,bhp, Vwx, Vwy, Vwz]]
%   Units are [rad,rad,rad,bhp, ft/sec, ft/sec, ft/sec]
echo off
[xIC,uIC,CL,CD,CM,alphadeg]=QuickTrim2(Vt,Hp,constant,Vwx,Vwy,Vwz)

% xIC(1)=63.6243997464708
% xIC(3)=4.90459243193238
% xIC(8)=0.0775829791165969
% Check to insure that the derivatives of the state vector (sys) are small 
% for this trim condition.
flag=1;;t=0;
[Xdot] = aircraft9(t,xIC,uIC,flag,constant,xIC,Xa);
disp('Trim Check: The first 9 derivatives (Xdot) should be small at trim')
Xdot
disp('  Now you are ready to run the nonlinear simulation using')
disp('  SIMULINK with the model called FlatEarth_MATLAB7.mdl from MATLAB 7')
disp('  or to execute Step3_Simulate_Trim.m.')

% Defalt perturbed controls below, just in case they are not defined elsewhere.
deltaEpertRAD=0.;
deltaApertRAD=0.;
deltaRpertRAD=0.;
deltabhppert=0.;
Tstop=8;    % Default stop time for subsequent simulations

% PLOT POWER REQUIRED CURVE
disp(' ')
disp(' You should know if you are flying on the frontside or the backside of the power required curve.')
disp(' This is because the response to elevator and power are significantly different on the backside.')
disp(' You should verify that the trimmed lift coefficient is less than CLmax.')
disp(' This code does not model stall so it will let you fly at unrealistic values of CL.')
disp(' ')
w=constant(1);
rho=rhofun(constant(59));
S=constant(20);
Cdo=constant(25);
k=constant(26);
v=20:.1:240;
CLp=w./(.5*rho*v.*v*S);
CDp=Cdo+k*CLp.*CLp;
Power=.5*rho*v.^3.*S.*CDp;
figure(1)
plot(v,Power)
title('Power Required Curve'); xlabel('trim speed (f/s)');ylabel('Power (ft-lbf/s)')
text2(.02,.5,'The Backside of the Power Required Curve is where the slope is negative.')
text2(.02,.45,'When flying slowly you can get on the backside (high CL).')
text2(.02,.40,'Trimmed flight is possible only when CL<CLmax.')
text2(.02,.35,'Check CL curve to insure CL<CLmax.')
hold on;
CLt=w/(.5*rho*Vt*Vt*S);
CDt=Cdo+k*CLt*CLt;
Powert=.5*rho*Vt^3*S*CDt;
plot(Vt,Powert,'rx'); hold off
figure(2)
plot(v,CLp)
hold on; plot(Vt,CLt,'rx'); 
xlabel('Trim speed (ft/sec)')
ylabel('CL')
str4=['CLtrim=',num2str(CLt),' at trim speed=',num2str(Vt),' ft/sec.'];
text2(.5,.1,str4)
hold off


