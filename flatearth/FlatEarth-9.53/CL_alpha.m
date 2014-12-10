%****************************************************************
% Function File for CLalpha
%****************************************************************
%
% Version 3.0 2/15/02. 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%
% Angie Dickinson
% Reference:Roskam, J., Methods for Estimating Stability and Control Derivatives of Conventional Subsonic Airplanes, 1977,Published by Jan Roskam, Kansas.
% INPUTS:
% AR_w          - Aspect ratio of the wing
% AR_h          - Aspect Ratio of the horizontal tail
% Lambda_c2     - Half chord sweep angle (rad)
% lambda_w      - Taper ratio
% kappa_h		 - Cl_alpha to 2pi ratio of the horizontal tail
% l_h           - the horizontal distance from the wing mean quarter chord to the horizontal tail mean quarter chord (ft)
% h_h           - the vertical distance from the wing root trailing edge to the chord line of the horizontal tail (ft)
% b_w           - span of the wing (ft)                         
% d                     - Fuselage diametre (ft)
% eta_h         - Ratio of dynamic pressure at the tail to that of the free stream
% S_h           - Area of the horizontal tail (ft^2)
% S_w           - Area of the wing (ft^2)
% beta          - Compressiblity correction factor
% kappa         - Ratio of Cl_alpha to 2pi

function [CL_alpha]=CL_alpha(AR_w,AR_h,Lambda_c2,lambda_w,l_h,h_h,b_w,d,eta_h,S_h,S_w,kappa_h,Lambda_c2_h,beta,kappa)

%To determine CLalpha, CLalpha of the wing is combined with CLalpha of the horiztonal tail

%************************************************************
%To determine CLalpha of the wing:CLalpha_wing
%************************************************************

%from equation 3.8 on page 3.2
CLalpha_wing= 2*pi*(AR_w)/ (2+sqrt((AR_w*beta/kappa)^2*(1+(tan(Lambda_c2))^2/beta^2)+4 ));
% No changes made, again reminding that TAN takes radians as input arguments - Murthy
%from equation 3.7 on page 3.2
Kwb= (1-.25*(d/b_w)^2+.025*(d/b_w));
%from equation 3.6 from page 3.2
CLalpha_wing_b=Kwb*CLalpha_wing;

%************************************************************
%To determine CLalpha of the horizontal tail:CLalpha_horizontal
%************************************************************
%From equation 3.8 on page 3.2
CLalpha_horizontal=2*pi*AR_h/(2+sqrt((AR_h*beta/kappa_h)^2*(1+(tan(Lambda_c2_h))^2/beta^2)+4 )); 
% No changes made, again reminding that TAN takes radians as input arguments - Murthy

%*************************************************************
%To determine CLalpha
%*************************************************************

%K_AR=wing aspect ratio factor
%from equation 3.13 on page 3.3
K_AR=(1./AR_w)-(1./(1+(AR_w)^1.7));
CLalpha_wing_M_is_zero=2*pi*(AR_w)/(2+sqrt((AR_w*1/kappa)^2*(1+(tan(Lambda_c2))^2/1^2)+4 ));
%No changes made, reminding that Lambda_c2 has to be in rad. - Murthy
%K_H=horizontal tail location factor
%from equation 3.15 on page 3.3 and figure 3.6 on page 3.10 
K_H=(1-(h_h./b_w))/(((2.*l_h)/b_w)^(1./3));
%K_lambda=wing taper ratio factor
%from equation 3.14
K_lambda=(10-(3*lambda_w))./7;

%from equation 3.12
d_epsilon_over_d_alpha_M_is_zero=4.44*(K_AR*K_lambda*K_H*sqrt(cos(Lambda_c2)))^1.19;
% No changes made, again reminding that COS takes radians as input arguments - Murthy
%from equation 3.11
d_epsilon_over_d_alpha=d_epsilon_over_d_alpha_M_is_zero*CLalpha_wing./CLalpha_wing_M_is_zero;
%from equation 3.5 on page 3.3
CL_alpha = CLalpha_wing_b + CLalpha_horizontal*eta_h*(S_h/S_w)*(1-d_epsilon_over_d_alpha); 
