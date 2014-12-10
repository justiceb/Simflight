function [CL_q]=CL_q(Xw,b_w,c_w,c_h,AR_w,Lambda_c4,Lambda_c2,Lambda_c2_h,Xh,S_h,S_w,eta_h,AR_h,...
   beta, V_h, b_h, kappa, kappa_h)%
% *********************************************
% CL_q - Variation of airplane lift coefficient
%        with dimensionless pitch rate.
% *********************************************     
%
% Version 3.0 2/15/02. 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%
%
% Inputs:
% Xw = distance from the reference point to the wing aero center (ft.) positive aft
% Xh = distance from ref pt. to horiz tail aerodynamic center (ft).
% b_w = span of the winf [ft]
% c_w = mean aerodynamic chord of wing [ft]
% c_h = mean aero chord of horizontal tail [ft]
% AR_w = aspect ratio of the wing
% AR_h = aspect ratio of horizontal tail
% Lambda_c4 = quarter-chord sweep of wing [rad]
% S_w = area of wing [ft^2]
% S_h = area of horizontal tail [ft^2]
% eta_h = dynamic pressure ratio at horizontal tail 
% Lambda_w = taper ratio of the wing
% Lambda_h = taper ratio of the horizontal tail
% beta = sqrt (1-M^2)
% 
% 
%
% Assumptions:
% Cl_q is considered to be the sum of the wing and tail contribution,
% the effect of the fuselage being usually small.
%
% A&AE 421 Fall 2001
% Jaret Matthews
% 
% 

%CLa_w= 2*pi*(AR_w)/ (2+sqrt((AR_w*beta/kappa)^2*(1+(tan(Lambda_c2))^2/beta^2)+4 )); %from equation 3.8 on page 3.2
%since this should be evaluated at M=0, beta should be 1 (eq. 5.3) - Murthy
CLa_w= 2*pi*(AR_w)/ (2+sqrt((AR_w/kappa)^2*(1+(tan(Lambda_c2))^2)+4 )); %from equation 3.8 on page 3.2

CLa_h=2*pi*AR_h/(2+sqrt((AR_h*beta/kappa_h)^2*(1+(tan(Lambda_c2_h))^2/beta^2)+4 )); %From equation 3.8 on page 3.2
 
CL_q_w_M0 =(0.5+2*Xw/c_w)*CLa_w;	%[rad^-1] Ref. 2, eqn 5.3
CL_q_w_M =(AR_w+2*cos(Lambda_c4))/(AR_w*b_h+2*cos(Lambda_c4)) * CL_q_w_M0; % [rad^-1] Ref. 2, eqn 5.2 
CL_q_w = CL_q_w_M; %[rad^-1] Wing Contribution to Variation of Lift Coefficient with Pitch Rate

CL_q_h = 2*CLa_h*eta_h*V_h; %[rad^-1] Ref. 2, eqn 5.4 - Horrizontal Tail Contribution to Variation of Lift Coefficient with Pitch Rate
CL_q = CL_q_w + CL_q_h; %[rad^-1] Ref.2, eqn 5.1 - Variation of Lift Coefficient with Pitch Rate
