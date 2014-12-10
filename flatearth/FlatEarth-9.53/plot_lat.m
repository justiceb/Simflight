
figure
% subplot(411)
% plot(taircraft,yaircraft(:,4))
% ylabel('p (rad/sec)')
% grid
% subplot(412)
% plot(taircraft,yaircraft(:,6))
% ylabel('r (rad/sec)')
% grid
subplot(411)
plot(taircraft,yaircraft(:,7))
ylabel('phi (rad)')
grid
subplot(412)
plot(taircraft,yaircraft(:,9))
ylabel('psi (rad)')
grid
subplot(413)
plot(taircraft,yaircraft(:,15))
ylabel('beta (rad)')
xlabel('time (s)')
grid
subplot(414)
plot(taircraft,yaircraft(:,20))
ylabel('Yacc (ft/s^2)')
xlabel('time (s)')
grid

figure
subplot(211)
plot(taircraft,uaircraft(:,2))
ylabel('deltaAil')
grid
subplot(212)
plot(taircraft,uaircraft(:,3))
ylabel('deltaR')
xlabel('time (s)')
grid

% load run3
% plot(taircraft,yaircraft(:,20),'b-')
% hold on
% load run3_1
% plot(taircraft,yaircraft(:,20),'r--')
% load run3_2
% plot(taircraft,yaircraft(:,20),'g:')