function [Cm_q]=Cm_q(Xw,b_w,c_w,c_h,AR_w,Lambda_c4,Lambda_c2,Lambda_c2_h,Xh,S_h,S_w,eta_h,AR_h,...
   beta, V_h,b_h, Cl_alpha,Cl_alpha_h,B)% *********************************************
% Cm_q - Variation of pitching moment coefficient
%        with pitch rate.
% *********************************************     
%
% Version 3.0 2/15/02. 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%
%
% Assumptions:
% Cm_q is considered to be the sum of the wing and tail contribution,
% the effect of the fuselage being usually small.
%
% Required Inputs:
% AR_w             Aspect ratio of the wing
% b_w              span of the wing [ft]
% b_h              span of the horizontal tail [ft]
% Xw               (positive reaward) distance from the arbitrary reference point to wing aerodynamic center [ft]
% eta_h            Ratio of dynamic pressure at the tail to that of the free stream
% c_w              Mean aerodynamic chord of the wing [ft]
% c_h              Mean aerodynamic chord of the horizontal tail [ft]
% V_h              Horizontal Tail Volume Coefficient
% Xh               Distance from arbitrary refernce point to the horizontal tail aerodynamic center [ft]
% Lambda_c4        Quater Chord Sweep angle [rad]
% Lambda_c2        Mid Chord Sweep angle [rad]
% B                Compressibility correction factor
% beta             Compressibility correction factor 
% M                Mach number
% Lambda           Mid chord sweep angle
% l_h              Defined in Fig 3.7 of Ref(2) Geometric parameter for horizontal tail location [ft]
% h_h              Defined in Fig 3.7 of Ref(2) Geometric parameter for horizontal tail location [ft]
% S_w              surface area of the wing [ft^2]
% S_h              surface area of horizontal tail [ft^2]
% AR_h             Aspect ratio of the horizontal tail
% CL_alpha         slope of the wing section lift curve
%
% A&AE 421 Fall 2001
% Jaret Matthews
% 
% Equations/Figures can be found in :
% Ref. (2) Roskam, Jan. "Methods for Estimating Satbility and
%         Control Derivatives of Conventional Subsonic Airplanes"
%         

kappa=Cl_alpha/(2*pi); %from equation 3.8 on page 3.2
kappa_h=Cl_alpha_h/(2*pi);

CLa_w= 2*pi*(AR_w)/ (2+sqrt((AR_w*beta/kappa)^2*(1+(tan(Lambda_c2))^2/beta^2)+4 )); %from equation 3.7 on page 3.2

CLa_h=2*pi*AR_h/(2+sqrt((AR_h*beta/kappa_h)^2*(1+(tan(Lambda_c2_h))^2/beta^2)+4 )); %From equation 3.8 on page 3.2

fig5_1 =[0	0.7; 6.47	0.7; 7.03	0.72; 7.5	0.744; 7.82	0.784; 8.13	0.824; 8.45	0.856; 8.76	0.872; 9.16	0.888; 10 0.9; 12 0.9; 20 0.9; 25 0.9]; %table of data from Ref. (2), fig 5.1
AR_tab = fig5_1(:,1); % Aspect Ratio values from above table
K_tab =fig5_1(:,2); % Correction constant values from above table
K=interlim1(AR_tab, K_tab, AR_w); % correction constant for wing contribution to Cm_q, Ref. (2), Fig 5.1
num1=AR_w*(2*(Xw/c_w)^2+.5*(Xw/c_w) ); %part of Ref. (2), eqn 5.8
den1=AR_w+2*cos(Lambda_c4); %part of Ref. (2), eqn 5.8
num2=AR_w^3*(tan(Lambda_c4))^2; %part of Ref. (2), eqn 5.8
den2=24*(AR_w+6*cos(Lambda_c4)); %part of Ref. (2), eqn 5.8
num3=((AR_w^3*(tan(Lambda_c4))^2)/(AR_w*B+6*cos(Lambda_c4))) + (3/B); %part of Ref. (2), eqn 5.7
den3=((AR_w^3*(tan(Lambda_c4))^2)/(AR_w+6*cos(Lambda_c4))) +3; %part of Ref. (2), eqn 5.7
Cm_q_w_M0=-K*CLa_w*cos(Lambda_c4)*((num1/den1)+(num2/den2)+1/8); %[rad^-1], Ref. (2), eqn 5.8
Cm_q_w=Cm_q_w_M0*(num3/den3); %[rad^-1], Ref. (2), eqn 5.7 - Wing Contribution to Variation of Pitching Moment Coefficient with Pitch Rate
Cm_q_h=-2*CLa_h*eta_h*V_h*(Xh/c_w); %[rad^-1], Ref. (2), eqn 5.9 - Horizontal Tail Contribution to Variation 
%                                   of Pitching Moment Coefficient with Pitch Rate
Cm_q = Cm_q_w+Cm_q_h; %[rad^-1], Ref. (2), eqn 5.6 - Variation of Pitching Moment Coefficient with Pitch Rate
return
