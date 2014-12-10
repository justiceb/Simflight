% Step0_ReadMe.m    Version 9.51 1/25/2010
%
echo on
disp(' ')
% Step0_ReadMe <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
% These MALAB scripts are designed to analyze and simulate arbitrary 
% rigid aircraft over the flat earth. A vehicle description file 
% called Basic_Constants is included allowing analysis 
% of a particular UAV.
%  
% It is important that these MATLAB scripts (.m files) be run in order.
% 
echo off
%Step1_Abbreviated
Step1_Math_Model
Step2_Trim
% Step3_Simulate_Trim
% Step4_Plot_Trim  %(optional step)
% Step5_Simulate_Perturbations
% Step6_Plot_Perturbations  %(optional step)
% Step7_Linearize_and_Overplot
% Step8_Longitudinal_Design % (memory is cleared in this step)
%Step9_Lateral_Design %(can be run after Step7), (memory is cleared in this step)
%
%
echo off
