%************************************************************************
% Step9_Lateral_Design.m    Version 9.51 1/25/2010
%
%************************************************************************
% OBJECTIVE: Set up linear models for use in linear control system design
%            for the lateral-directional subsystem
% INPUTS: The file modelLat.mat created by Step7_Linearize_and_Overplot.m
% OUTPUTS: Many transfer functions, poles, natural frequencies and 
%          damping ratios in the MATLAB command window. Also several linear
%          time invariant systems are created in the MATLAB workspace f
%          or subsequent analysis (LDsys, LDtfsys, LDzpksys).
% NOTES: 1. This code can be run after Step7_Linearize_and_Overplot.m has been run.
%        2. This code clears memory at the start if execution


clear all
close all
disp(' '); disp('Step9_Lateral_Design<<<<<<<'); disp(' ');
disp(' Note: memory is cleared at the start of this script'); disp(' ')
load modelLat aLD bLD cLD dLD Vt Hp aircraft % Load lateral directional subsystem linear model
%whos
disp(aircraft)
Vt
Hp
LDsys=ss(aLD,bLD,cLD,dLD);
set(LDsys,'statename',{'v(f/s)' 'p(r/s)' 'r(r/s)' 'phi(r)' 'psi(r)'})
set(LDsys,'inputname',{'deltaA(r)' 'deltaR(r)'})
set(LDsys,'outputname',{'v(f/s)' 'p(r/s)' 'r(r/s)' 'phi(r)' 'psi(r)' 'beta(r)'})
LDsys
[Wn,Z,Poles]=damp(LDsys)
LDtfsys=tf(LDsys)
LDzpksys=zpk(LDsys)

% select r/Dr transfer function (3 output and 2 input)
%io=3
%ic=2
%LDsys2=ss(aLD,bLD(:,ic),cLD(io,:),dLD(io,ic));
%set(LDsys2,'statename',{'v(f/s)' 'p(r/s)' 'r(r/s)' 'phi(r)' 'psi(r)'})
%set(LDsys2,'inputname','deltaR(r)')
%set(LDsys2,'outputname','r(r/s)')
%LDsys2
%[Wn2,Z2,Poles2]=damp(LDsys2)
%LDtfsys2=tf(LDsys2)
%LDzpksys2=zpk(LDsys2)

% To set up for r feedback to the rudder in order to 
% increase the damping ratio of the dutch roll mode
RperDr=LDsys(3,2)
RperDrZPK=zpk(RperDr)
sisotool
echo on
% Import RperDrZPK into G of sisotool
% Examine the root locus and make sure you have the sign for stabilizing
% feedback. If the locus goes the wrong way, click the +- sign in the
% lower left corner of the block diagram in sisotool. The locus goes the
% right way is the dutch roll complex pair increases its damping ratio.
% Increased damping is noted in the root locus if the dutch roll complex poles
% move towards the real axis. On the root locus, a line of constant damping is a radial line.
% On the root locus, a line of constant natural frequency is a circle.
% If you click on the root locus plot you can bring up the property editor.
% In the property editor, open the Options tab. Select Show grid.
% This will bring up lines of constant damping ratio on the root locus.
% In the root locus the purple squares are the closed loop pole locations
% for the nominal gain. Click on any square and move it until you get the
% damping ratio that you want. The nominal gain will automatically be
% adjusted to acheive that result. This should complete your design.
echo off




