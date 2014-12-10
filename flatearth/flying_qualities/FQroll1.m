function [level time phi_sim t_sim level1_time roll_angle]=FQroll1(da,LDsys,class,phase,sim_run_time,plot_flag,spd_range)
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
%roll_angle - Angle to roll aircraft throuh (rad)
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
level1_time=0;
%Run Linear Simulation
if class == 1
    if phase == 1 || phase == 2
        roll_angle=deg2rad(60);
    elseif phase == 3
        roll_angle=deg2rad(30);
    end
end
if class == 21
    if phase == 1 || phase == 2
        roll_angle=deg2rad(45);
    elseif phase == 3
        roll_angle=deg2rad(30);
    end
end

if class == 22
    if phase == 1 || phase == 2
        roll_angle=deg2rad(45);
    elseif phase == 3
        roll_angle=deg2rad(25);
    end
end 
if class == 3
    roll_angle=deg2rad(30);
end
if class == 4
    if phase==1
        if (spd_range==1 || spd_range==2)
            roll_angle=deg2rad(30);
        elseif (spd_range==3)
            roll_angle=deg2rad(90);
        elseif (spd_range==4)
            roll_angle=deg2rad(50);
        end
    elseif phase==2
        roll_angle=deg2rad(90);
    elseif phase==3
         roll_angle=deg2rad(30);
    end
end

[phi_sim,t_sim]=step(da*LDsys(4,1),sim_run_time);
time=interlim1(phi_sim,t_sim,roll_angle);

%Define times for flying qualities levels
%Data from MIL-F-8785C

class1_table=[1.3 1.7 1.3;
              1.7 2.5 1.8;
              2.6 3.4 2.6];
class2_table.L=[1.4 1.9 1.8;
                1.9 2.8 2.5;
                2.8 3.8 3.6];            
class2_table.C=[1.4 1.9 1.0;
                1.9 2.8 1.5;
                2.8 3.8 2.0];     
class3_table.low=[1.8 2.3 2.5;
                  2.4 3.9 4.0;
                  3.0 5.0 6.0];
class3_table.med=[1.5 2.0 2.5;
                  2.0 3.3 4.0;
                  3.0 5.0 6.0];
class3_table.high=[2.0 2.3 2.5;
                   2.5 3.9 4.0;
                   3.0 5.0 6.0];
class4_table.vlow=[1.1 2.0 1.1;
                   1.6 2.8 1.3;
                   2.6 3.7 2.0];
class4_table.low=[1.1 2.7 1.1;
                  1.5 2.5 1.3;
                  2.0 3.4 2.0];
class4_table.med=[1.3 1.7 1.1;
                  1.7 2.5 1.3;
                  2.6 3.4 2.0];
class4_table.high=[1.1 1.7 1.1;
                  1.3 2.5 1.3;
                  2.6 3.4 2.0];

%Initialize counting and while loop variables                   
count=0;
test=0;

if class == 1
    while test==0;
        count=count+1;
        if phase == 1 || phase == 2
            roll_angle=deg2rad(60);
            if class1_table(count,phase) > time;
                level1_time=class1_table(1,phase);
                level=count;
                test=1;
            elseif time > class1_table(3,phase)
                test=1;
                level=0;
            end
        elseif phase == 3
            roll_angle=deg2rad(30);
            if class1_table(count,phase) > time;
                level1_time=class1_table(1,phase);
                level=count;
                test=1;
            elseif time > class1_table(3,phase)
                test=1;
                level=0;
            end
        end
    end
end

if class == 21
    while test==0;
        count=count+1;
        if phase == 1 || phase == 2
            roll_angle=deg2rad(45);
            if class2_table.L(count,phase) > time;
                level1_time=class2_table.L(1,phase);
                level=count;
                test=1;
            elseif time > class2_table.L(3,phase)

                test=1;
                level=0;
            end
        elseif phase == 3
            roll_angle=deg2rad(30);
            if class2_table.L(count,phase) > time;
                level1_time=class2_table.L(1,phase);
                level=count;
                test=1;
            elseif time > class2_table.L(3,phase)
                test=1;
                level=0;
            end
        end
    end
end

if class == 22
    while test==0;
        count=count+1;
        if phase == 1 || phase == 2
            roll_angle=deg2rad(45);
            if class2_table.C(count,phase) > time;
                level1_time=class2_table.C(1,phase);
                level=count;
                test=1;
            elseif time > class2_table.C(3,phase)
                test=1;
                level=0;
            end
        elseif phase == 3
            roll_angle=deg2rad(25);
             if class2_table.C(count,phase) > time;
                level1_time=class2_table.C(1,phase);
                level=count;
                test=1;
            elseif time > class2_table.C(3,phase)
                test=1;
                level=0;
            end
        end
    end
end 

if class == 3
    if spd_range==2
        class3_table_red=class3_table.low;
    elseif spd_range==3
            class3_table_red=class3_table.med;
    elseif spd_range==4
        class3_table_red=class3_table.high;
    else
        level=0;
        test=1;
    end
    while test==0
     count=count+1;
         if class3_table_red(count,phase) > time;
            level1_time=class3_table_red(1,phase);
            level=count;
            test=1;
        elseif time > class3_table_red(3,phase)
            test=1;
            level=0;
        end
    end
end

if class == 4
    if spd_range==1
        class4_table_red=class4_table.vlow;
    elseif spd_range==2
            class4_table_red=class4_table.low;
    elseif spd_range==3
        class4_table_red=class4_table.med;
    elseif spd_range==4
        class4_table_red=class4_table.high;
    else
        level=0;
        test=1;
    end
    
    while test==0
        count=count+1;
        if phase==1
            if (spd_range==1 || spd_range==2)
                roll_angle=deg2rad(30);
                if class4_table_red(count,phase) > time;
                    level1_time=class4_table_red(1,phase);
                    level=count;
                    test=1;
                elseif time > class4_table_red(3,phase)
                    test=1;
                    level=0;
                end
            elseif (spd_range==3)
                roll_angle=deg2rad(90);
                if class4_table_red(count,phase) > time;
                    level1_time=class4_table_red(1,phase);
                    level=count;
                    test=1;
                elseif time > class4_table_red(3,phase)
                    test=1;
                    level=0;
                end
            elseif (spd_range==4)
                roll_angle=deg2rad(50);
                if class4_table_red(count,phase) > time;
                    level1_time=class4_table_red(1,phase);
                    level=count;
                    test=1;
                elseif time > class4_table_red(3,phase)
                    test=1;
                    level=0;
                end
            end
        elseif phase==2
            roll_angle=deg2rad(90);
            if class4_table_red(count,phase) > time;
                level1_time=class4_table_red(1,phase);
                level=count;
                test=1;
            elseif time > class4_table_red(3,phase)
                test=1;
                level=0;
            end
         elseif phase==3
             roll_angle=deg2rad(30);
            if class4_table_red(count,phase) > time;
                level1_time=class4_table_red(1,phase);
                level=count;
                test=1;
            elseif time > class4_table_red(3,phase)
                test=1;
                level=0;
            end
        else 
            test=1;
            level=0;
        end
    end
end
if plot_flag==1;
    phi_level1=interp1(t_sim,phi_sim,level1_time);
    time_to_roll_angle=interlim1(phi_sim,t_sim,roll_angle);
    plot(t_sim,rad2deg(phi_sim));
    xlabel('Time (sec)')
    ylabel('\phi (deg)')
    
    hold on;
    plot([time_to_roll_angle time_to_roll_angle],[0 rad2deg(roll_angle)],'--r')
    plot([0 time_to_roll_angle],[rad2deg(roll_angle) rad2deg(roll_angle)],'--r')
    hold off
    title('Phi vs Time')
    string1=['Level 1 flying qualities requires'];
    string2=['0 to ',num2str(rad2deg(roll_angle)), ...
        '^o in ', num2str(level1_time), ' sec.'];

    string3=['This aircraft rolls ',num2str(rad2deg(roll_angle)),'^0 in ',...
        num2str(time_to_roll_angle,3),' seconds.'];
    string4=['FQ Level = ', num2str(level)];

    text2(.1,.9,string1)
    text2(.1,.85,string2)
    text2(.1,.80,string3)
    text2(.1,.75,string4)
end

return