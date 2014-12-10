%************************************************************************
% Step8_Longitudinal_Design.m    Version 9.51 1/25/2010
%
%************************************************************************
% 
% OBJECTIVE: Set up linear models for use in linear control system design
%            for the longitudinal subsystem
% INPUTS: The file modelLong.mat created by Step7_Linearize_and_Overplot.m
% OUTPUTS: Many transfer functions, poles, natural frequencies and 
%          damping ratios in the MATLAB command window. Also several linear
%          time invariant systems are created in the MATLAB workspace f
%          or subsequent analysis (Lsys, Ltfsys, Lzpksys).
% NOTES: 1. This code can be run after Step7_Linearize_and_Overplot.m has been run.
%        2. This code clears memory at the start if execution

clear all
close all
disp(' '); disp('Step8_Longitudinal_Design<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<'); disp(' ');
disp(' Note: memory is cleared at the start of this script'); disp(' ')
load modelLong aL bL cL dL Vt Hp aircraft % Load lateral directional subsystem linear model
disp(aircraft)
Vt
Hp
Lsys=ss(aL,bL,cL,dL);
set(Lsys,'statename',{'u(f/s)' 'w(f/s)' 'q(r/s)' 'thet(r)' 'h(f)'})
set(Lsys,'inputname',{'deltaE(r)' 'Bhp(hp)'})
set(Lsys,'outputname',{'u(f/s)' 'w(f/s)' 'q(r/s)' 'thet(r)' 'h(f)' 'Vt(f/s)' 'alf(r)' 'gam(r)'})
Lsys
[Wn,Z,Poles]=damp(Lsys)
Ltfsys=tf(Lsys)
Lzpksys=zpk(Lsys)
% To set up for q feedback to the elevator in order to 
% increase the damping ratio of the short period mode
QperDe=Lsys(3,1)
QperDeZPK=zpk(QperDe)
sisotool 
echo on
% Import QperDeZPK into G of sisotool
% Examine the root locus and make sure you have the sign for stabilizing
% feedback. If the locus goes the wrong way, click the +- sign in the
% lower left corner of the block diagram in sisotool. The locus goes the
% right way is the short period complex pair increases its damping ratio.
% Increased damping is noted in the root locus if the short period complex poles
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