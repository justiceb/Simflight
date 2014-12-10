function [Ftx,Fty,Ftz,Lt,Mt,Nt]=thrust(u,constant,Vt)
% [Ftx,Fty,Ftz,Lt,Mt,Nt]=thrust(u,constant,Vt)
% u(4)=bhp (hp)
% Vt=airspeed in ft/sec
% constant(23)=phiT, thrust inclination angle, RADIANS
% constant(24)=dT, thrust offset distance, ft
% constant(8)= eta, propeller efficiency
% Propeller efficiency is assumed constant.

T=550*u(4)*constant(8)/Vt;
Ftx=T*cos(constant(23));
Fty=0;
Ftz=-T*sin(constant(23));
Lt=0;
Mt=-T*constant(24);
Nt=0;
