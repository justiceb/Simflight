addpath FlatEarth-9.53

clc; clear all; close all; format short g
Basic_Constants_CDR
Make_Constants  % Creates the array called constant used to define the aerodynamics and mass properties of the aircraft.
Check_Constants % Check the constants for believability.
Step2_Trim
Step3_Simulate_Trim
Step4_Plot_Trim  %(optional step)
Step5_Simulate_Perturbations
Step6_Plot_Perturbations  %(optional step)
Step7_Linearize_and_Overplot

close all
clc

parent = 'C:\GitHub\classwork\AAE 451\Code\flatearth\test';
save(fullfile(parent,'a.txt'),'a', '-ascii');
save(fullfile(parent,'b.txt'),'b', '-ascii');
save(fullfile(parent,'c.txt'),'c', '-ascii');
save(fullfile(parent,'d.txt'),'d', '-ascii');
save(fullfile(parent,'uIC.txt'),'uIC', '-ascii');
save(fullfile(parent,'xIC.txt'),'xIC', '-ascii');
du_out = du(1:2,:);
save(fullfile(parent,'du.txt'),'du_out', '-ascii');




