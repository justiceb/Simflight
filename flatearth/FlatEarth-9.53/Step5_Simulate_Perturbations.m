%************************************************************************
% Step5_Simulate_Perturbations.m    Version 9.51 1/25/2010
%
%************************************************************************
% OBJECTIVE: Simulate an aircraft in 6 degree of freedom motion
%            with nonlinear equations of motion and nonlinear
%            aerodynamic and thrust models. A flat earth and
%            rigid body dynamics are assumed. The motion can involve
%            perturbations from trim as defined below.
% INPUT: In the Matlab workspace will be defined initial conditions
%        on the state and controls (xIC and uIC). The array constant, xIC and uIC
%        are all required by this SIMULINK nonlinear simulation.
% OUTPUT: Arrays in the MATLAB workspace called taircraft and yaircraft 
%         that contain an aray of time values and a matrix of output state 
%         time histories.
echo on
%Step5_Simulate_Perturbations  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
% MATLAB 7 users will run
% the nonlinear simulation in SIMULINK using FlatEarth_MATLAB7.mdl.
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
%
%Perturbations to the controls.
%
deltaEpertRAD=1/57.29577 % Use this for elevator step
%deltaEpertRAD=0          % Use this for step in other controller
deltaApertRAD=1/57.29577 % Use this for aileron step
%deltaApertRAD=0;         % Use this for step in other controller
deltaRpertRAD=1/57.29577 % Use this for rudder step
%deltaRpertRAD=0;        % Use this for step in other controller
deltabhppert=0.05        % Use this for horsepower step
%deltabhppert=0;        % Use this for step in other controller
Tstop=100          % Stop time for simulation (sec)
sim('FlatEarth_MATLAB7')
echo off
