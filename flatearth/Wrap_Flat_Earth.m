
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

%get xfer functions we want
xfer_el = Ltfsys(3,1)
xfer_al = LDtfsys(2,1)
xfer_rud = LDtfsys(3,2)
xfer_rud_roll = LDtfsys(2,2)

%get poles
[R_long,~] = rlocus(xfer_el,0)
[R_lat,~] = rlocus(xfer_al,0)

%get damping ratios and natural frequencies
[Wn_long,zeta_long] = damp(xfer_el)
[Wn_lat,zeta_lat] = damp(xfer_al)

%plot characteristic poles
figure(50)
plot(real(R_long),imag(R_long),'x','MarkerSize', 12)
temp = xlim;
hold on;plot([0 0],ylim,':k');hold on;plot([temp(1) temp(2)+1],[0 0],':k')
xlabel('real axis')
ylabel('imaginary axis')
title('longitudinal motion characteristic roots')
figure(51)
plot(real(R_lat),imag(R_lat),'x','MarkerSize', 12)
temp = xlim;
hold on;plot([0 0],ylim,':k');hold on;plot([temp(1) temp(2)],[0 0],':k')
xlabel('real axis')
ylabel('imaginary axis')
title('lateral motion characteristic roots')

%find spiral mode time to double
K = 0;
rlocus(xfer_al,K)
[NUM,DEN] = tfdata(xfer_al) 
NUM = NUM{1}; DEN = DEN{1};
sim('Lateral_Stability_Model')
t_sim = simout.Time;
q = simout.Data;
deflection = simout_cmd.Data;
deflection_feedback = simout_cmd_feedback.Data;
figure(24)
plot(t_sim,q)
xlabel('time (s)')
ylabel('roll Rate Q (deg/s)')
grid on
hold all
figure(25)
plot(t_sim,deflection,t_sim,deflection_feedback)
xlabel('time (sec)')
ylabel('rudder deflection (deg)')
legend('INPUT --> elevator control','feedback driven input')





%{
%pull out Xfer function we want
[z,p,k] = zpkdata(Lsys(3,1));  % From input "deltaE(r)" to output "q(r/s)"
NUM = poly(z{1})*k;
DEN = poly(p{1});
xfer = tf(NUM,DEN)

figure(9)
K = -0.5;
[R,~] = rlocus(-xfer,-K)
rlocus(-xfer,-K)
sim('Longitudinal_Stability_Model')
t_sim = simout.Time;
q = simout.Data;
deflection = simout_cmd.Data;
figure(10)
plot(t_sim,q*57.2957795)
xlabel('time (s)')
ylabel('Pitch Rate Q (deg/s)')
grid on
hold all
figure(11)
plot(t_sim,deflection)
xlabel('time (sec)')
ylabel('elevator deflection (deg)')
legend('INPUT --> elevator control')


%pull out Xfer function we want
close all
[z,p,k] = zpkdata(LDsys(3,2));  % From input "deltaE(r)" to output "q(r/s)"
NUM = poly(z{1})*k;
DEN = poly(p{1});
xfer = tf(NUM,DEN)
figure(23)
K = -0.01;
[R,~] = rlocus(-xfer,-K)
rlocus(-xfer)
sim('Lateral_Stability_Model')
t_sim = simout.Time;
q = simout.Data;
deflection = simout_cmd.Data;
deflection_feedback = simout_cmd_feedback.Data;
figure(24)
plot(t_sim,q)
xlabel('time (s)')
ylabel('Yaw Rate Q (deg/s)')
grid on
hold all
figure(25)
plot(t_sim,deflection*57.2957795,t_sim,deflection_feedback*57.2957795)
xlabel('time (sec)')
ylabel('rudder deflection (deg)')
legend('INPUT --> elevator control','feedback driven input')


%pull out Xfer function we want
close all
[z,p,k] = zpkdata(LDzpksys(2,1));  % From input "deltaE(r)" to output "q(r/s)"
NUM = poly(z{1})*k;
DEN = poly(p{1});
xfer = tf(NUM,DEN)
figure(23)
K = 0;
[R,~] = rlocus(xfer,K)
rlocus(xfer)
sim('Lateral_Stability_Model')
t_sim = simout.Time;
q = simout.Data;
deflection = simout_cmd.Data;
deflection_feedback = simout_cmd_feedback.Data;
figure(24)
plot(t_sim,q)
xlabel('time (s)')
ylabel('roll Rate Q (deg/s)')
grid on
hold all
figure(25)
plot(t_sim,deflection,t_sim,deflection_feedback)
xlabel('time (sec)')
ylabel('rudder deflection (deg)')
legend('INPUT --> elevator control','feedback driven input')

%}
