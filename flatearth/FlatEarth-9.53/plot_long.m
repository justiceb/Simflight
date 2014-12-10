altcomm=raircraft(:,1)+xIC(12);
velcomm=raircraft(:,2)+sqrt(xIC(1)^2+xIC(3)^2);

figure
subplot(511)
plot(taircraft,yaircraft(:,8))
ylabel('pitch, theta (rad)')
grid
subplot(512)
plot(taircraft,yaircraft(:,5))
ylabel('pitch rate, q (rad/sec)')
grid
subplot(513)
plot(taircraft,yaircraft(:,12))
hold on
plot(taircraft,altcomm,'g:')
ylabel('altitude, h (ft)')
grid
subplot(514)
plot(taircraft,yaircraft(:,14))
ylabel('alpha (rad)')
grid
subplot(515)
plot(taircraft,yaircraft(:,13))
hold on
plot(taircraft,velcomm,'g:')
ylabel('speed, v_t (ft/sec)')
xlabel('time (s)')
grid

figure
subplot(211)
plot(taircraft,uaircraft(:,1))
ylabel('deltaE (rad)')
grid
subplot(212)
plot(taircraft,uaircraft(:,4))
ylabel('Bhp')
xlabel('time (s)')
grid
