%FQRoll1 Doc
disp(' ')
disp(' ')
disp('<<<<<<<<<<<Executing FQRoll1 - Roll Rate Requirements MIL-F-8785C Sec 3.3.4')
disp(' ')
disp(' ')
echo on
%FQroll1(da,roll_angle,LDsys,class,phase,sim_run_time,plot_flag,spd_range) 
%   Finds the flying qualites level for roll effectiveness
%   requirements.
%   For Flight Phase Category A use phase = 1
%                             B use phase = 2
%                             C use phase = 3
%   For Airplane Class  I use class=1;
%                       II-L use class = 21;
%                       II-C use class = 22;
%                       III  use class = 3;
%                       IV   use class = 4;
%   For Speed Range Very Low use spd_range = 1
%                        Low use spd_range = 2
%                     Medium use spd_range = 3
%                       High use spd_range = 4
% References
% [1] Roskam, Jan. "Airplane Flight Dynamics and Automatic Flight Controls", 
%     DARcorporation Lawrence, KS Second Printing, 1998.
% 
% [2] Roskam, Jan. “Airplane Design: Part VII Determination of Stability, 
%                  Control, and Performance Characteristics: 
%                  FAR and Military Requirements. DARcorporation. Lawrence, KS, 2002.
% 
% [3] Raymer, Daniel P. “Aircraft Design: A Conceptual Approach” AIAA 
%                       Education Series. Reston, VA Fourth Edition, 2006.
% 
% [4] Anon.; MIL-F-8785c, Military Specification Flying Qualities of 
%            Piloted Airplanes; November 5, 1980; Air Force Flight 
%            Dynamics Laboratory, WPAFB, Dayton, OH.
% 
% [5] Anon.; MIL-STD-1797A, Flying Qualities of Piloted Aircraft; 
%            January 30, 1990; Air Force Flight Dynamics 
%            Laboratory, WPAFB, Dayton, OH.
%
%Input Variables:
%LDsys - Lateral directional linear system
%class - Aircraft class ID
%phase - Aircraft phase ID
%spd_range - Aircraft speed range ID
%da - aileron Deflection angle (rad)
%LDsys - Lateral Directional Linear System
%sim_run_time - Length of time for simulation
%plot_flag - Flag to turn plots on/off
%
%Output Variables:
%level - Flying qualities level
%time - Time to roll through defined roll angle with aileron deflection da
%phi_sim - Simulated bank angle
%t_sim - Simulation times
%level1_time - Time required to achieve level 1 FQ
%roll_angle - Angle to roll aircraft throuh (rad)
echo off
format compact
disp('User Inputs')
class
phase
spd_range
da_deg
eta_ia
ca_cw
format loose
