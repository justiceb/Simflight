% Linearize_MATLAB7_fq.m    Version 1.0 3/22/09
% OBJECTIVES: 1. Script to linearize the aircraft system assuming the nonlinear
%                aircraft model in SIMULINK model FlatEarth_MATLAB7.mdl.
%             2. Split the 12th order system into the smaller longitudinal
%                and lateral-directional subsystems.
%             3. Save to binary files the longitudinal and lateral-directional
%                state space subsystems (modelLong.mat, modelLat.mat).
% 
%INPUTS: 1. In the Matlab workspace must be an array of simulation times (taircraft)
%           and a matrix of simulation outputs (yaircraft). These are generally
%           created by the SIMULINK simulations FlatEarth_MATLAB7.mdl.
%        2. In the Matlab workspace will be defined initial conditions
%           on the state and controls (xIC and uIC) andthe  array constant.
%
% OUTPUTS: 
%          2 output files (modelLong.mat, modelLat.mat),
%          transfer functions, poles, natural frequencis and damping ratios in the command window.

%disp(' ');disp('>>>>>>>Start here, Step5_Linearize_MATLAB7<<<<<<<');disp(' ');
%disp('Compute a quick trim')
Vtknots=U; % trim speed (knots)

%disp('Trim conditions')
flag=3; t=0;
[y1] = aircraft9(t,xIC,uIC,flag,constant,xIC,Xa);

% OBJECTIVE: 1. Script to linearize the aircraft system assuming the nonlinear
%                aircraft model in SIMULINK model FlatEarth_MATLABx.mdl.
%disp('Linear system matrices, linearized agout trim')
echo off
% Outputs (19)
%   y=[U,V,W,P,Q,R,Phi,Theta,Psi,Xprime,Yprime,Hprime,Vt,alpha,beta,gamma,Xdot,Ydot,Hdot]
%   Xdot,Ydot,Hdot are components of the inertial velocity vector in the NED frame.
%   Units are [ft/sec, ft/sec, ft/sec, rad/sec, rad/sec, rad/sec, 
%   rad, rad, rad, ft, ft, ft, ft/sec, rad, rad, rad, ft/sec, ft/sec, ft/sec]
% States (12)
%   x=[U,V,W,P,Q,R,Phi,Theta,Psi,Xprime,Yprime,Hprime]
%   Units are [ft/sec, ft/sec, ft/sec, rad/sec, rad/sec, rad/sec, rad, rad, rad, ft, ft, ft]
% Inputs (7)
%   u=[deltaE,deltaA,deltaR,bhp, Vwx, Vwy, Vwz]]
%   Units are [rad,rad,rad,bhp, ft/sec, ft/sec, ft/sec]
echo off
[a,b,c,d]=linmod('FlatEarth_ports_MATLAB7',xIC,uIC); %Find linear model 
%disp('Natural frequencies, damping ratios, and poles of system matrix a')
%[Wn,Z]=damp(a); % Find properties of the poles of linearized system
%poles=eig(a);
PSYS=ss(a,b,c,d); %Create a linear state space system

%%%%%%
% OBJECTIVE: 2. Split the 12th order system into the smaller lateral-directional subsystem.
% [a,b,c,d]=selector(A,B,C,D,xselect,uselect,yselect)
% Select lateral-directional subsystem
%disp(' '); disp('Lateral-Directional Subsystem'); disp(' ')
xselect=[2,4,6,7,9]; uselect=[2,3]; yselect=[2,4,6,7,9,15];
[aLD,bLD,cLD,dLD]=selector(a,b,c,d,xselect,uselect,yselect);
%
% OBJECTIVE: 3. Save to binary files the lateral-directional
%                state space subsystem (modelLat.mat).
%save modelLat aLD bLD cLD dLD Vt Hp aircraft % Save subsystem linear model
LDsys=ss(aLD,bLD,cLD,dLD);
set(LDsys,'statename',{'v(f/s)' 'p(r/s)' 'r(r/s)' 'phi(r)' 'psi(r)'});
set(LDsys,'inputname',{'deltaA(r)' 'deltaR(r)'});
set(LDsys,'outputname',{'v(f/s)' 'p(r/s)' 'r(r/s)' 'phi(r)' 'psi(r)' 'beta(r)'});

%%%%%%
% OBJECTIVE: 2. Split the 12th order system into the smaller longitudinal subsystem.
% [a,b,c,d]=selector(A,B,C,D,xselect,uselect,yselect)
% Select Longitudinal subsystem
%disp(' '); disp('Longitudinal Subsystem'); disp(' ')
xselect=[1,3,5,8,12]; uselect=[1,4]; yselect=[1,3,5,8,12,13,14,16];
[aL,bL,cL,dL]=selector(a,b,c,d,xselect,uselect,yselect);
%    SAVE fname X Y Z  -ASCII  uses 8-digit ASCII form instead of binary.
%
% OBJECTIVE: 3. Save to binary files the longitudinal
%                state space subsystem (modelLong.mat).
%%save modelLong aL bL cL dL Vt Hp aircraft % Save subsystem linear model
Lsys=ss(aL,bL,cL,dL);
set(Lsys,'statename',{'u(f/s)' 'w(f/s)' 'q(r/s)' 'thet(r)' 'h(f)'});
set(Lsys,'inputname',{'deltaE(r)' 'Bhp(hp)'});
set(Lsys,'outputname',{'u(f/s)' 'w(f/s)' 'q(r/s)' 'thet(r)' 'h(f)' 'Vt(f/s)' 'alf(r)' 'gam(r)'});
%Lsys;
