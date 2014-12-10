
str=['b-'];
% subplot(321)
% plot(taircraft,uaircraft(:,2),str)
% hold on
% ylabel('Aileron, \delta_A (rad)')
% grid
% subplot(322)
% plot(taircraft,uaircraft(:,3),str)
% hold on
% ylabel('Rudder, \delta_R (rad)')
% grid
subplot(221)
plot(taircraft,uaircraft(:,1),str)
hold on
ylabel('Elevator, \delta_E (rad)')
grid
subplot(222)
plot(taircraft,uaircraft(:,4),str)
hold on
ylabel('Throttle, Bhp (Hp) ')
grid
subplot(223)
plot(taircraft,yaircraft(:,12),str)
hold on
ylabel('Altitude, h (ft)')
xlabel('Time (s)')
grid
subplot(224)
plot(taircraft,yaircraft(:,13),str)
hold on
ylabel('Speed, v_t (ft/sec)')
xlabel('Time (s)')
grid