%************************************************************************
% Step7_Linearize_and_Overplot.m    Version 9.51 1/25/2010
% Removed extra call for Quicktrim2
% Change x1, u1 to xIC, uIC
%************************************************************************
% OBJECTIVES: 1. Script to linearize the aircraft system assuming the nonlinear
%                aircraft model in SIMULINK model FlatEarth_MATLAB7.mdl.
%             2. Perform a linear simulation for the same conditions as
%                used in the SIMULINK nonlinear simulation.
%             3. Overplot the nonlinear simulation results and the linear simulation 
%                results in order to verify the accuracy of the linearization.
%             4. Split the 12th order system into the smaller longitudinal
%                and lateral-directional subsystems.
%             5. Save to binary files the longitudinal and lateral-directional
%                state space subsystems (modelLong.mat, modelLat.mat).
%             6. Determine if we are on the frontside or the backside of 
%                the power required curve.
% 
%INPUTS: 1. In the Matlab workspace must be an array of simulation times (taircraft)
%           and a matrix of simulation outputs (yaircraft). These are generally
%           created by the SIMULINK simulations FlatEarth_MATLAB7.mdl.
%        2. In the Matlab workspace will be defined initial conditions
%           on the state and controls (xIC and uIC) andthe  array constant.
%
% OUTPUTS: 3 figures of overplots,
%          plot of the power required curve,
%          2 output files (modelLong.mat, modelLat.mat),
%          transfer functions, poles, natural frequencis and damping ratios in the command window.

disp(' ');disp('Step7_Linearize_and_Overplot <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<');disp(' ');
% disp('Compute a quick trim')
% 
Vt=constant(58)  % trim speed (ft/sec) 
Vtknots=U % trim speed (knots)
Hp=constant(59) % altitude (ft)

% Get outputs at the trim condition
flag=3; t=0;
[y1] = aircraft9(t,xIC,uIC,flag,constant,xIC,Xa);
% OBJECTIVE: 1. Script to linearize the aircraft system assuming the nonlinear
%                aircraft model in SIMULINK model FlatEarth_MATLABx.mdl.
disp('Linear system matrices, linearized agout trim')
echo on
% Outputs (19)
%   y=[U,V,W,P,Q,R,Phi,Theta,Psi,Xprime,Yprime,Hprime,Vt,alpha,beta,gamma,Xdot,Ydot,Hdot
%      Xacc, Yacc,Zacc]
%   Xdot,Ydot,Hdot are components of the inertial velocity vector in the NED frame.
%   Xacc, Yacc,Zacc are 3 perpendicular accelerometers.
%   Units are [ft/sec, ft/sec, ft/sec, rad/sec, rad/sec, rad/sec, 
%   rad, rad, rad, ft, ft, ft, ft/sec, rad, rad, rad, ft/sec, ft/sec, ft/sec, 
%   ft/(sec*sec),ft/(sec*sec),ft/(sec*sec)]
% States (12)
%   x=[U,V,W,P,Q,R,Phi,Theta,Psi,Xprime,Yprime,Hprime]
%   Units are [ft/sec, ft/sec, ft/sec, rad/sec, rad/sec, rad/sec, rad, rad, rad, ft, ft, ft]
% Inputs (7)
%   u=[deltaE,deltaA,deltaR,bhp, Vwx, Vwy, Vwz]]
%   Vwx, Vwy, Vwz are 3 components of the wind in the NED frame.
%   Units are [rad,rad,rad,bhp, ft/sec, ft/sec, ft/sec]
echo off
[a,b,c,d]=linmod('FlatEarth_ports_MATLAB7',xIC,uIC) %Find linear model of nonlinear pendulum
disp('Natural frequencies, damping ratios, and poles of system matrix a')
[Wn,Z]=damp(a) % Find properties of the poles of linearized system
poles=eig(a)
PSYS=ss(a,b,c,d); %Create a linear state space system
disp(' ')
disp('Small Perturbation Initial condition for linear simulation.')
disp('Note that this is X(0)-XR, where X(0) is the same I.C. as')
disp('used in the nonlinear simulation.')
X0=xIC;
XR=xIC;
YR=y1;
dx0=X0-XR; %Use this small perturbation initial condition for linear simulation
input='Time response comparison. Flat Earth Model';
timevector=taircraft;
tstart=timevector(1)
tstop=timevector(length(timevector))
t=tstart:.02:tstop;
u=zeros(length(t),4);
%YI = INTERP1(X,Y,XI)
u(:,1)=interp1(timevector,deltaE,t');
u(:,2)=interp1(timevector,deltaA,t');
u(:,3)=interp1(timevector,deltaR,t');
u(:,4)=interp1(timevector,deltabhp,t');
u(:,5)=zeros(size(u(:,4)));
u(:,6)=u(:,5);
u(:,7)=u(:,5);

% form small perturbation controls
du=zeros(size(u));
du(:,1)=u(:,1)-uIC(1);
du(:,2)=u(:,2)-uIC(2);
du(:,3)=u(:,3)-uIC(3);
du(:,4)=u(:,4)-uIC(4);
du(:,5)=u(:,5)-uIC(5);

% OBJECTIVE:  2. Perform a linear simulation for the same conditions as
%                used in the SIMULINK nonlinear simulation.
[y,t]=lsim(PSYS,du,t,dx0); %Do linear simulation
%Plot the linear results and the nonlinear results
%  which have been stored in tnlsim and ynlsim.

% OBJECTIVE: 3. Overplot the nonlinear simulation results and the linear simulation 
%                results in order to verify the accuracy of the linearization.
figure(1) 
disp('Take care to add the reference state xR to the small perturbation')
disp('solution in order to get the total approximate solution.')
disp('Only the total approximate solution should be compared to the ')
disp('results from the nonlinear simulation.')

figure(1)
subplot(311)
plot(t,y(:,5)+YR(5),'-',timevector,yaircraft(:,5),':')
xlabel('time (sec)')
ylabel('Q (rad/sec)')
legend('linear sim','nonlinear sim')
title(input)

subplot(312)
plot(t,y(:,8)+YR(8),'-',timevector,yaircraft(:,8),':')
xlabel('time (sec)')
ylabel('theta (rad)')
legend('linear sim','nonlinear sim')

subplot(313)
plot(t,y(:,14)+YR(14),'-',timevector,yaircraft(:,14),':')
xlabel('time (sec)')
ylabel('alpha (rad)')
legend('linear sim','nonlinear sim')

figure(2)
subplot(311)
plot(t,y(:,6)+YR(6),'-',timevector,yaircraft(:,6),':')
xlabel('time (sec)')
ylabel('R (rad/sec)')
legend('linear sim','nonlinear sim')
title(input)

subplot(312)
plot(t,y(:,9)+YR(9),'-',timevector,yaircraft(:,9),':')
xlabel('time (sec)')
ylabel('psi (rad)')
legend('linear sim','nonlinear sim')

subplot(313)
plot(t,y(:,15)+YR(15),'-',timevector,yaircraft(:,15),':')
xlabel('time (sec)')
ylabel('beta (rad)')
legend('linear sim','nonlinear sim')

figure(3)
subplot(311)
plot(t,y(:,4)+YR(4),'-',timevector,yaircraft(:,4),':')
xlabel('time (sec)')
ylabel('P (rad/sec)')
legend('linear sim','nonlinear sim')
title(input)

subplot(312)
plot(t,y(:,7)+YR(7),'-',timevector,yaircraft(:,7),':')
xlabel('time (sec)')
ylabel('phi (rad)')
legend('linear sim','nonlinear sim')

subplot(313)
plot(t,y(:,11)+YR(11),'-',timevector,yaircraft(:,11),':')
xlabel('time (sec)')
ylabel('Y (ft)')
legend('linear sim','nonlinear sim')



%%%%%%
% OBJECTIVE: 4. Split the 12th order system into the smaller lateral-directional subsystem.
% [a,b,c,d]=selector(A,B,C,D,xselect,uselect,yselect)
% Select lateral-directional subsystem
disp(' '); disp('Lateral-Directional Subsystem'); disp(' ')
xselect=[2,4,6,7,9]; uselect=[2,3]; yselect=[2,4,6,7,9,15];
[aLD,bLD,cLD,dLD]=selector(a,b,c,d,xselect,uselect,yselect);
%
% OBJECTIVE: 5. Save to binary files the lateral-directional
%                state space subsystem (modelLat.mat).
save modelLat aLD bLD cLD dLD Vt Hp aircraft % Save subsystem linear model
LDsys=ss(aLD,bLD,cLD,dLD);
set(LDsys,'statename',{'v(f/s)' 'p(r/s)' 'r(r/s)' 'phi(r)' 'psi(r)'})
set(LDsys,'inputname',{'deltaA(r)' 'deltaR(r)'})
set(LDsys,'outputname',{'v(f/s)' 'p(r/s)' 'r(r/s)' 'phi(r)' 'psi(r)' 'beta(r)'})
LDsys
disp('Natural frequencies, damping ratios, and poles of system matrix aLD')
[Wn,Z,Poles]=damp(LDsys)
LDtfsys=tf(LDsys)
LDzpksys=zpk(LDsys)


%%%%%%
% OBJECTIVE: 4. Split the 12th order system into the smaller longitudinal subsystem.
% [a,b,c,d]=selector(A,B,C,D,xselect,uselect,yselect)
% Select Longitudinal subsystem
disp(' '); disp('Longitudinal Subsystem'); disp(' ')
xselect=[1,3,5,8,12]; uselect=[1,4]; yselect=[1,3,5,8,12,13,14,16];
[aL,bL,cL,dL]=selector(a,b,c,d,xselect,uselect,yselect);
%    SAVE fname X Y Z  -ASCII  uses 8-digit ASCII form instead of binary.
%
% OBJECTIVE: 5. Save to binary files the longitudinal
%                state space subsystem (modelLong.mat).
save modelLong aL bL cL dL Vt Hp aircraft % Save subsystem linear model
Lsys=ss(aL,bL,cL,dL);
set(Lsys,'statename',{'u(f/s)' 'w(f/s)' 'q(r/s)' 'thet(r)' 'h(f)'})
set(Lsys,'inputname',{'deltaE(r)' 'Bhp(hp)'})
set(Lsys,'outputname',{'u(f/s)' 'w(f/s)' 'q(r/s)' 'thet(r)' 'h(f)' 'Vt(f/s)' 'alf(r)' 'gam(r)'})
Lsys
disp('Natural frequencies, damping ratios, and poles of system matrix aL')
[Wn,Z,Poles]=damp(Lsys)
Ltfsys=tf(Lsys)
Lzpksys=zpk(Lsys)


% OBJECTIVE: 6. Determine if we are on the frontside or the backside of the power required curve.
% Plot the power required curve
Vknots=6:2:80;
Vfps=Vknots*1.688;
qbarS=.5*rho*Vfps.^2*S_w;
CLxx=W./qbarS;
CDxx=Cd_0+k*CLxx.^2;
DVxx=qbarS.*CDxx.*Vfps;
figure(4); clf
plot(Vknots,DVxx)
xlabel('V(knots)')
ylabel('Power Req (ft-#/s)')
hold on
%YI = INTERP1(X,Y,XI)
% Add the trim point on the power required curve
DVi=interp1(Vknots,DVxx,Vtknots);
plot(Vtknots,DVi,'x')
% Indicate where the backside and frontside are.
text(Vtknots,DVi+2000,'Trim point')
dmin=min(DVxx); jd=find(DVxx==dmin);
text(Vknots(jd+1),dmin,'Frontside')
text(Vknots(jd-3),dmin,'Backside')
title('Power Required Curve')
hold off


