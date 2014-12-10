function [nu]=nufun(h)
% function [nu]=nufun(h)
% Compute kinematic viscosity nu (ft*ft/sec)
% h can be a vector (ft)
% Ref Barnes W. McCormick, Aerodynamics, Aeronautics, and Flight Mechanics
% 	Second Edition, John Wiley and Sons,  pp 24-25.
% These results are accurate to within 0.1% at an altitude of 70,000 ft.
A0=1.5723;
A1=8.73065e-2;
A2=-1.18412e-2;
A3=1.16978e-3;
A4=-5.27207e-5;
A5=1.22466e-6;
A6=-1.369780e-8;
A7=5.94238e-11;
Vx104=A0+A1*(h/1000)+A2*(h/1000).^2+A3*(h/1000).^3+A4*(h/1000).^4+A5*(h/1000).^5+A6*(h/1000).^6+A7*(h/1000).^7;
nu=Vx104/10000;
