
CD0=.02668
%k=.068
k=.0885-CD0
A=6
e=1/(pi*A*k)
CL=-.2:.01:1.05;
CD=CD0+k*CL.*CL;
plot(CL,CD)
title('Drag polar of MPX5')
v=axis
axis([-.4,1.2,.02,.1])
xlabel('CL')
ylabel('CD')
grid on
k=1/(pi*A*.858)