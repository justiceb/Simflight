%% Run Flatearth-9.53
addpath FlatEarth-9.53
clc; clear all; close all; format short g
Basic_Constants_MPX5                          %This is where you specify your input constants file
Make_Constants                                % Creates the array called constant used to define the aerodynamics and mass properties of the aircraft.
Check_Constants                               % Check the constants for believability.
Step2_Trim
Step3_Simulate_Trim
Step4_Plot_Trim                               %(optional step)
Step5_Simulate_Perturbations
Step6_Plot_Perturbations                      %(optional step)
Step7_Linearize_and_Overplot
close all
clc

%% Export state space to config files within "labview code" directory
config_filename = 'MPX5';  %This will be the folder name containing the generated config files
root_folder =  fileparts(pwd);
parent = fullfile(root_folder,'labview_code/config_files',config_filename);
save(fullfile(parent,'a.txt'),'a', '-ascii');
save(fullfile(parent,'b.txt'),'b', '-ascii');
save(fullfile(parent,'c.txt'),'c', '-ascii');
save(fullfile(parent,'d.txt'),'d', '-ascii');
save(fullfile(parent,'uIC.txt'),'uIC', '-ascii');
save(fullfile(parent,'xIC.txt'),'xIC', '-ascii');
du_out = du(1:2,:);
save(fullfile(parent,'du.txt'),'du_out', '-ascii');

%% Initialize default SIM_controls config file (if it already exists, do nothing)
if exist(fullfile(parent,'SIM_Controls.ini')) == 0
   default_file = copyfile('default_SIM_Controls.ini',fullfile(parent,'SIM_Controls.ini'));
end


