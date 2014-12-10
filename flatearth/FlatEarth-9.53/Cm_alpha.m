%****************************************************************
% Function File for Cm_alpha
%****************************************************************
%
% Version 3.0 2/15/02. 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%
%Reference:Roskam, J., Methods for Estimating Stability and Control Derivatives of Conventional Subsonic Airplanes, 1977,Published by Jan Roskam, Kansas.
%To determine Cm_alpha, CLalpha and d_C_m_over_d_C_L need to be calculated.
%To determine CLalpha, CLalpha of the wing is combined with CLalpha of the horiztonal tail.  

%Inputs:		AR_w        	aspect ratio of the wing
% 				AR_h  			aspect ratio of horizontal tail
% 				Lambda			wing sweep angle
%				lambda_w		taper ratio 
%				l_h				the horizontal distance from the wing mean quarter chord 
%								to the horizontal tail mean quarter chord (ft)
%				h_h				the vertical distance from the wing root trailing edge to
%								the chord line of the horizontal tail (ft)
%				b_w				wing span (ft)
%				M				flight Mach number
%				d				fuselage diametre (ft)
%				eta_h			dynamic pressure ratio at the horizontal tail 
%								usually assumed to be in the range of .90<eta_h<1.0
%				S_h				surface area of the horizontal tail (ft^2)
%				S_w				surface area of the wing (ft^2)
%				Cl_alpha 		two dimensional coefficient of lift
%				Xref			Distance from the leading edge of the wing mean aerodynamic chord
%                               to the arbitrary reference point. The equivalent force system
% 								for the aerodynamic force system is given about this point.
%						        Measured as positive aft, starting from the leading edge of the mean aero. chord. [ft]
%               Xacwb			Distance from the leading edge of the wing mean aerodynamic chord
%                               to the aerodynamic center of the wing and body. 
%						        Measured as positive aft, starting from the leading edge of the mean aero. chord. [ft]


function [Cm_alpha]=Cm_alpha(AR_w,AR_h,Lambda_c2,lambda_w,l_h,h_h,b_w,c_w,d,eta_h,S_h,S_w,kappa_h,Lambda_c2_h,beta,kappa,Xref,Xacwb)


%To determine Cm_alpha, CLalpha is multiplied by d_C_m_over_d_C_L.  Therefore CLalpha needs to be determined first.


%************************************************************

%To determine CLalpha of the wing:CLalpha_wing

%************************************************************



%from equation 3.8 on page 3.2

CLalpha_wing= 2*pi*(AR_w)/ (2+sqrt((AR_w*beta/kappa)^2*(1+(tan(Lambda_c2))^2/beta^2)+4 ));

%from equation 3.7 on page 3.2

Kwb= (1-.25*(d/b_w)^2+.025*(d/b_w));

%from equation 3.6 from page 3.2

CLalpha_wing_b=Kwb*CLalpha_wing;



%************************************************************

%To determine CLalpha of the horizontal tail:CLalpha_horizontal

%************************************************************

%From equation 3.8 on page 3.2

CLalpha_horizontal=2*pi*AR_h/(2+sqrt((AR_h*beta/kappa_h)^2*(1+(tan(Lambda_c2_h))^2/beta^2)+4 )); 



%*************************************************************

%To determine CLalpha

%*************************************************************



%K_AR=wing aspect ratio factor

%from equation 3.13 on page 3.3

K_AR=(1./AR_w)-(1./(1+(AR_w)^1.7));

CLalpha_wing_M_is_zero=2*pi*(AR_w)/(2+sqrt((AR_w*1/kappa)^2*(1+(tan(Lambda_c2))^2/1^2)+4 ));

%K_H=horizontal tail location factor

%from equation 3.15 on page 3.3 and figure 3.6 on page 3.10 

K_H=(1-(h_h./b_w))/(((2.*l_h)/b_w)^(1./3));

%K_lambda=wing taper ratio factor

%from equation 3.14

K_lambda=(10-(3*lambda_w))./7;



%from equation 3.12

d_epsilon_over_d_alpha_M_is_zero=4.44*(K_AR*K_lambda*K_H*sqrt(cos(Lambda_c2)))^1.19;

%from equation 3.11

d_epsilon_over_d_alpha=d_epsilon_over_d_alpha_M_is_zero*CLalpha_wing./CLalpha_wing_M_is_zero;

%from equation 3.5 on page 3.3

CL_alpha = CLalpha_wing_b + CLalpha_horizontal*eta_h*(S_h/S_w)*(1-d_epsilon_over_d_alpha); 


%				Xach			Distance from the leading edge of the wing mean aerodynamic chord
%                               to the aerodynamic center of the horizontal tail (positive aftward) [ft]
%l_h=Xach-.25*c_w
Xach=l_h+.25*c_w;
XBARach=Xach/c_w;
XBARref=Xref/c_w;

% from equation 3.18 we computetheaerodynamic center of the wing, body, and tail.
numerator=(Xacwb/c_w)+CLalpha_horizontal*eta_h*(S_h/S_w)*XBARach*(1-d_epsilon_over_d_alpha)/CLalpha_wing_b;
denominator=1+CLalpha_horizontal*eta_h*(S_h/S_w)*(1-d_epsilon_over_d_alpha)/CLalpha_wing_b;
XBARac=numerator/denominator;

%Now d_C_m_over_d_C_L is calculated

%from equation 3.17 on page 3.3. Since we are using the arbitrary reverence point
% for our aerodynamic model we do not use the c.g. position in equation 3.17.
d_C_m_over_d_C_L=XBARref-XBARac;

%Finally, to determine Cm_alpha, CLalpha is multiplied by d_C_m_over_d_C_L according to equation 3.16 on page 3.3
Cm_alpha=CL_alpha*d_C_m_over_d_C_L;
