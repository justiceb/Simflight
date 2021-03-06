%************************************************************************
% Step4_Plot_Trim.m    Version 9.51 1/25/2010
%
%************************************************************************
% 
% OBJECTIVE: Plot results from the nonlinear flat earth simulation.
%
% INPUTS: In the Matlab workspace must be an array of simulation times (taircraft)
%         and a matrix of simulation outputs (yaircraft). These are generally
%         created by the SIMULINK simulations FlatEarth_MATLABx.mdl.
% OUTPUTS: Four figures, each with three subplots.

disp(' ');disp('Step4_Plot_Trim <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<');
disp(' ');
disp(' Trim time histories should remain at trim (constant)'); disp(' ')
input='Time histories that should remain at trim values (initial conditions)';
figure(1)
subplot(311)
plot(taircraft,yaircraft(:,5))
v=axis
axis([v(1),v(2),-.1,.1]) % Manually set plot scale to prevent tiny wiggles
xlabel('time (sec)')
ylabel('Q (rad/sec)')
title(input)

subplot(312)
plot(taircraft,yaircraft(:,8))
v=axis
axis([v(1),v(2),-.1,.1]) % Manually set plot scale to prevent tiny wiggles
xlabel('time (sec)')
ylabel('theta (rad)')

subplot(313)
plot(taircraft,yaircraft(:,14))
v=axis
axis([v(1),v(2),0,.2]) % Manually set plot scale to prevent tiny wiggles
xlabel('time (sec)')
ylabel('alpha (rad)')

figure(2)
subplot(311)
plot(taircraft,yaircraft(:,6))
xlabel('time (sec)')
ylabel('R (rad/sec)')
title(input)

subplot(312)
plot(taircraft,yaircraft(:,9))
xlabel('time (sec)')
ylabel('psi (rad)')

subplot(313)
plot(taircraft,yaircraft(:,15))
xlabel('time (sec)')
ylabel('beta (rad)')

figure(3)
subplot(311)
plot(taircraft,yaircraft(:,4))
xlabel('time (sec)')
ylabel('P (rad/sec)')
title(input)

subplot(312)
plot(taircraft,yaircraft(:,7))
xlabel('time (sec)')
ylabel('phi (rad)')

subplot(313)
plot(taircraft,yaircraft(:,11))
xlabel('time (sec)')
ylabel('Y (ft)')

figure(4)
subplot(311)
plot(taircraft,deltaE)
xlabel('time (sec)')
ylabel('Elevator (rad)')
title(input)

subplot(312)
plot(taircraft,deltaA)
xlabel('time (sec)')
ylabel('Aileron (rad)')

subplot(313)
plot(taircraft,deltaR)
xlabel('time (sec)')
ylabel('Rudder (rad)')

figure(5)
subplot(311)
plot(taircraft,yaircraft(:,13))
v=axis
axis([v(1),v(2),round(yaircraft(1,13)-5),round(yaircraft(1,13)+5)]) % Manually set plot scale to prevent tiny wiggles
xlabel('time (sec)')
ylabel('Speed (f/s)')
title(input)

subplot(312)
plot(taircraft,yaircraft(:,16))
v=axis
axis([v(1),v(2),-.1,.1]) % Manually set plot scale to prevent tiny wiggles
xlabel('time (sec)')
ylabel('Flight Path Angle (gamma) (rad)')

subplot(313)
plot(taircraft,deltabhp)
xlabel('time (sec)')
ylabel('bhp')
