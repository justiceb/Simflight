function [CL_0 CL_0_h CL_0_wf]=CL_0(S_w,S_h,M,tc_w,alpha_0,epsilon_t,i_w,i_h,epsilon_0_h,AR_w,Lambda_c4,Lambda_c2,Lambda_c2_h,lambda_w,kappa,kappa_h,beta,b_w,d,AR_h,eta_h)

% *********************************************
% CL_0 - the aircraft zero-AOA lift coefficent
%
% *********************************************     
%
%************************************************************************
% Version 9.5 4/26/09
% Added outputs for CL_0_h and CL_0_wf
% Corrected CL_alpha_h to use kappa_h and lambda_c2_h
%
%************************************************************************
%
% Version 3.0 2/15/02. 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%
%
% A&AE 421 Fall 2001
% Matthew Wysel
%
% This function file calculates CL_0 - the aircraft zero-AOA lift coefficent.
% It take in the following variables:
% S_h					area of horizontal tail [ft^2]
% S_w					aera of wing [ft^2]r
% Lambda_c4			quater chord sweep angle [rad]
% lambda_w		   (wing) taper ratio
% M					Mach number
% tc_w				wing thickness to chord ratio
% alpha_0			Airfoil zero-lift AOA [rad]
% epsilon_t			horizontal tail twist angle [rad]
% i_w					wing incidence angle [rad]
% i_h					horizontal tail incidence angle [rad]
% epsilon_0_h 		Downwash angle at zero AOA [rad]

%
% and returns the following variables to 'constant.m':
% CL_0				aircraft zero-AOA lift
%
% Assumptions: Propellor driven, subsonic
%
% Applicable References (NB: the first number in parentheses is the number by 
% which the given reference will be referred to in the code)
% (3)[Ref. ] Roskam's Airplane Design Series: Book VI
%

					AR_tab = [1.5,3.5,6,10];						% Aspect ratio from fig.(8.41)
               Lambda_c4_tab = [0,30,45,50,60]*pi/180;				% Sweep chord angle from fig.(8.41)
               % converted to radians on 02/11/2002 by Murthy
					lambda_tab = [0,0.5,1.0];						% Taper ratio from fig.(8.41)
           		dalphadepsilon_tab(:,:,1) = ...
                    [-0.3989	-0.3813	-0.3665	-0.3602
                    -0.3943	-0.3648	-0.346	-0.3398
                    -0.3881	-0.3551	-0.3347	-0.3273
                    -0.3841	-0.3494	-0.3301	-0.3222
                    -0.3722	-0.3347	-0.317	-0.3091]; % taper ratio = 0 Ref.(3) fig(8.41)
                dalphadepsilon_tab(:,:,2) = ...
                    [-0.4071	-0.411	-0.4133	-0.4133
                    -0.4029	-0.3967	-0.4001	-0.4001
                    -0.3986	-0.3885	-0.3935	-0.3935
                    -0.3969	-0.3851	-0.3902	-0.3902
                    -0.3875	-0.3751	-0.3807	-0.3807]; % taper ratio = 0.5 Ref.(3) fig(8.41)
                dalphadepsilon_tab(:,:,3) = ...
                   [-0.4069	-0.4171	-0.4269	-0.4371
                   -0.408	-0.4109	-0.42	-0.4251
                   -0.4023	-0.4074	-0.4166	-0.4229
                   -0.3989	-0.4063	-0.416	-0.4229
                   -0.3891	-0.4051	-0.4166	-0.4246];	 % data for dalphadepsilon when taper ratio = 1.0 Ref.(3) fig(8.41)
            dalphadepsilon = interlim3(AR_tab,Lambda_c4_tab,lambda_tab,dalphadepsilon_tab,AR_w,Lambda_c4,lambda_w);	% Effect of linear twist on Wing AOA for zero lift            
            McosLambda_c4 = M*cos(Lambda_c4);
            %deleted pi/180, since the inpput is going to be in radians, by Murthy on 02/11/2002
          		tc_w_tab = [.07,.08,.09,.10,.12,.14,.16];											% Taper ratio from fig.(8.41)
					McosLambda_c4_tab = [0.3,.35,.4,.45,.5,.55,.6,.65,.7,.75,.8,.85,.9];		% Sweep chord angle from fig.(8.41)
               PrandetlGlaurtScale_tab(:,:) =...
                  [1	1	1	1	1	1	1
                  1	1	1	1	1	1	1
                  1	1	1	1	1	1	1
                  1	1	1	1	1	1	0.96
                  1	1	1	1	1	1	0.88
                  1	1	1	1	1	0.96	0.68
                  1	1	1	1	0.99	0.8	0.4
                  1	1	1	1	0.91	0.51	-0.09
                  1	1	1	1	0.6	0.011	-1
                  1	1	1	0.87	-0.21	NaN	NaN
                  1	0.95	0.82	0.092	NaN	NaN	NaN
                  0.95	0.68	0.12	NaN	NaN	NaN	NaN
                  0.87	NaN	NaN	NaN	NaN	NaN	NaN];	% Mach number correction for zero-lift angle AOA of cambered airfoils Ref.(3) fig.(8.42)
             PrandetlGlaurtScale = interlim2(tc_w_tab,McosLambda_c4_tab,PrandetlGlaurtScale_tab,tc_w,McosLambda_c4);
               if PrandetlGlaurtScale == NaN
                  warning backtrace
                  warning('PrandetlGlaurtScale value out of range')
               end
               alpha_0_L_w = (alpha_0 + dalphadepsilon*epsilon_t)*(PrandetlGlaurtScale);
               %No change made, but please remember that alpha_0_L_w is in radians - Murthy
%from equation 3.8 on page 3.2
CLalpha_wing= 2*pi*(AR_w)/ (2+sqrt((AR_w*beta/kappa)^2*(1+(tan(Lambda_c2))^2/beta^2)+4 ));
%This equation gives CLalpha_wing in rad^{-1} - Murthy
%from equation 3.7 on page 3.2
Kwb= (1-.25*(d/b_w)^2+.025*(d/b_w));
%from equation 3.6 from page 3.2
CLalpha_wing_b=Kwb*CLalpha_wing;%This is in rad^{-1} - Murthy

%************************************************************
%To determine CLalpha of the horizontal tail:CLalpha_horizontal
%************************************************************
%From equation 3.8 on page 3.2
CL_alpha_h=2*pi*AR_h/(2+sqrt((AR_h*beta/kappa_h)^2*(1+(tan(Lambda_c2_h))^2/beta^2)+4 )); % This is in 1/rad - Murthy

CL_0_wf = (i_w - alpha_0_L_w)*CLalpha_wing_b;
%No change made, but please remember that CL_0_wf is dimensionless, CLalpha_wing_b is in 1/rad, angles are in rad - Murthy
epsilon_0_h = 0;									% Downwash angle at the horizontal tail (see Note in Ref.(3) under section 8.1.5.2) [rad]
% Coefficient of lift for Zero AOA %
CL_0_h=CL_alpha_h*eta_h*(S_h/S_w)*(i_h - epsilon_0_h);
CL_0 = CL_0_wf + CL_alpha_h*eta_h*(S_h/S_w)*(i_h - epsilon_0_h); % Aircraft Lift coefficient at Zero AOA Ref.(3) Equ.(8.32)
return
