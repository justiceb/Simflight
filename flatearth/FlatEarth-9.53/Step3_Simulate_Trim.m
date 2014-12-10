%************************************************************************
% Step3_Simulate_Trim.m    Version 9.51 1/25/2010
%
%************************************************************************
%
disp(' ')
echo on
%Step3_Simulate_Trim <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
% OBJECTIVE: Simulate trim flight of an aircraft in 6 degree of freedom motion
%            with nonlinear equations of motion and nonlinear
%            aerodynamic and thrust models. A flat earth and
%            rigid body dynamics are assumed. If the aircraft is truly
%            trimmed the trajectory should remain trimmed (constant) for several
%            minutes.
% INPUT: In the Matlab workspace will be defined initial conditions
%        on the state and controls (xIC and uIC). The array constant, xIC and uIC
%        are all required by this SIMULINK nonlinear simulation.
% OUTPUT: Arrays in the MATLAB workspace called taircraft and yaircraft 
%         that contain an aray of time values and a matrix of output state 
%         time histories.
%
% To bring up Simulink, type simulink from the Matlab command window.
%
% From the Simulink window, use the File Open menu
%   to open the Simulink model  FlatEarth_MATLAB7.mdl.
%   Make sure the perturbation controls are defined in the workspace
%   or in the perturbation blocks.
%   To run the Simulink simulation click on the run button (>).
%
% Alternately, you can execute the following code and the 
% SIMULINK nonlinear simulation will run using default times and options.
%
deltaEpertRAD=0.;
deltaApertRAD=0.;
daltaRpertRAD=0.;
deltabhppert=0.;
Tstop=1000 % Stop time for simulation (sec)
sim('FlatEarth_MATLAB7')
echo off
