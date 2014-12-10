%FQRoll1 example usage
clear all;clc;close all
addpath FlatEarth-9.53
addpath flying_qualities
Basic_Constants_MPX5;
Make_Constants;
%Initialize accelerometer locations to zero for linearization
Xa(1)=0; % x-location of accelerometer wrt c.g. (ft)
Xa(2)=0; % y-location of accelerometer wrt c.g. (ft)
Xa(3)=0; % z-location of accelerometer wrt c.g. (ft)
%% User Defined Values
class=1;  %Airplane Class
phase=1;  %Flight Phase
spd_range=2; %Speed range (arbitrary for Class I, II-L, & II-C)
da_deg=25; %maximum aileron deflection angle (deg)
eta_ia=.72; % Percent semi-span position of inboard edge of aileron
%eta_oa = 43/45; % Percent semi-span position of outboard edge of aileron
%NOTE: Choose ca_cw > .05 (limitation of Figure 10.6 Ref 2)
ca_cw=0.02:.02:.4; % Ratio of aileron chord to wing chord
%ca_cw=.25;
sim_run_time=3; %Set time to run linear simulation
FQRoll1_doc;
%% Calculations
da=da_deg*0.0174532925; %maximum aileron deflection angle (rad)

%Set up Loop Length
plot_flag=1; %Initialize plot flag to plots off
if length(eta_ia)>1
    loop_length=length(eta_ia);
elseif length(eta_oa)>1
    loop_length=length(eta_oa);
elseif length(ca_cw)>1
    loop_length=length(ca_cw);
else
    loop_length=1;
    plot_flag=1;
end


%Set initial conditions for linearization about trim
[xIC,uIC,CL,CD,CM,alphadeg]=QuickTrim2(constant(58),constant(59),constant,0,0,0);
for j=1:loop_length
    %Compute Cl_da for each aileron design value
    constant(41)=Cl_da(S_w,AR_w,ca_cw(j),eta_ia,eta_oa,beta,kappa,Lambda_c4,lambda_w,tc_w,Cl_alpha_w);
    %Compute Cn_da for each aileron design value
    constant(52)=Cn_da(S_w,AR_w,ca_cw(j),eta_ia,eta_oa,beta,kappa,Lambda_c4,lambda_w,tc_w,Cl_alpha_w,CL);
    %Linearize system to obtain A,B,C,D for lateral system
    Linearize_MATLAB7_min;
    %Determine flying qualities level
    [level(j) time(j) phi_history(:,j) t_history(:,j) level1_time roll_angle]=FQroll1(da,LDsys,class,phase,sim_run_time,plot_flag,spd_range);
end

%% Plot Results fo Multiple values of design variables
if loop_length>1
    figure
    plot(ca_cw,time)
    hold on
    plot([0 ca_cw(length(ca_cw))],[level1_time level1_time],'r--')
    text(.01,level1_time+.05,'Level 2')
    text(.01,level1_time-.05,'Level 1')
    title('Time to Bank vs Aileron to Wing Chord Ratio')
    xlabel('Aileron Chord to Wing Chord ratio')
    ylabel(['Time to Bank ', num2str(rad2deg(roll_angle)),'^0'])
end


