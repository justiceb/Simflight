function [Cm_0]=Cm_0(S_h_slip,S_w,S_h,M,tc_w,alpha_0,epsilon_t,i_w,i_h,epsilon_0_h,AR_w,Lambda_c4,lambda_w,beta,Cm_0_r,Cm_o_t,Lambda_c2_h,kappa_h,AR_h,Xh,c_w,eta_h)
%
% *********************************************
% Cm_0 - the aircraft zero-lift pitching moment coefficient
%
% *********************************************     
%
% Version 3.0 2/15/02. 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%
%
% A&AE 421 Fall 2001
% Matthew Wysel
% Adam Butt
%
% This function file calculates CL_0 - the aircraft zero-lift pitching moment coefficient
% It take in the following variables:
% S_h_slip			area of horizontal tail that is covered in prop-wash - see fig.(8.64) Ref.(3) [ft^2]
% S_h					area of horizontal tail [ft^2]
% S_w					aera of wing [ft^2]
% Lambda_c4			quater chord sweep angle [rad]
% lambda_w		   (wing) taper ratio
% M					Mach number
% tc_w				wing thickness to chord ratio
% alpha_0			Airfoil zero-lift AOA [rad]
% epsilon_t			horizontal tail twist angle [rad]
% i_w					wing incidence angle [rad]
% i_h					horizontal tail incidence angle [rad]
% epsilon_0_h 		Downwash angle at zero AOA [rad]
% wf_i            Average width of fuselage [ft]
% Deltax_i        Length of fuselage segment as shown in Fig. 8.112 of Ref(3) [ft]
% Xh=Xach-Xref;   Distance from airplane arbitrary reference point to the horizontal tail ac [ft]
%
% Cm_0_wf         The zero-lift pitching moment coefficient of the wing-fuselage combination (Pg. 320 of Ref(3))
% Cm_0_c          The zero-lift pitching moment coefficient of the canard
% Cm_0_h          The zero-lift pitching moment coefficient of the horizontal tail
% Cm_0_w          The zero-lift pitching moment coefficient of the wing
% Cm_0_f          The zero-lift pitching moment coefficient of the fuselage
%
% and returns the following variables to 'constant.m':
% Cm_0				aircraft zero-lift pitching moment coefficient
%
% Assumptions: Propellor driven, subsonic, AR<100 NB This can be increased - see comment
%
% Applicable References (NB: the first number in parentheses is the number by 
% which the given reference will be referred to in the code)
% (3)[Ref. ] Roskam's Airplane Design Series: Book VI
%


%you'll need this later too:
cosLambda_c4 = cos(Lambda_c4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cm_0 = Cm_0_wf + Cm_0_c + Cm_0_h
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Cm_0_c = 0;                   % Assumes there is no canard

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculation of CL_0_h
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

					AR_tab = [1.5,3.5,6,10];						% Aspect ratio from fig.(8.41)
					Lambda_c4_tab = [0,30,45,50,60]*pi/180;				% Sweep chord angle from fig.(8.41)
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


%************************************************************
%To determine CLalpha of the horizontal tail:CLalpha_horizontal
%************************************************************
%From equation 3.8 on page 3.2
CLalpha_horizontal=2*pi*AR_h/(2+sqrt((AR_h*beta/kappa_h)^2*(1+(tan(Lambda_c2_h))^2/beta^2)+4 ));
         

% From second term of eqn 8.32 of ref 3 (page 268)
CL_0_h =CLalpha_horizontal*eta_h*(S_h/S_w)*(i_h-epsilon_0_h);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Cm_0_h = (Xh/c_w)*CL_0_h;           % Equation (8.79) on Pg.322 of Ref(3) 

Cm_0_w = ((AR_w*(cos(Lambda_c4))^2)/(AR_w+2*cos(Lambda_c4)))*((Cm_0_r+Cm_o_t)/2);    % Equ. (8.70) on Pg.302 of Ref(3)

Cm_0_f = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Explanation of Cm_0_f = 0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% wf_i = [1 2 3 4 5 6 7 8 9 10 11 12 13];            Average width of fuselage (in 13 segments)
% Deltax_i = [1 2 3 4 5 6 7 8 9 10 11 12 13];        Length of fuselage segment as shown in Fig. 8.112 of Ref(3)
% z = wf_i^2.*(i_w + alpha_0_L_w + i_cl_f).*Deltax_i;  Product that can be summed  
% 
% Cm_0_f = ((k2-k1)/(36.5*S_w*c_w))*sum(z,2);        Equation (8.78) of Ref(3)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Cm_0_wf = Cm_0_w + Cm_0_f;                 % Equ(8.77) on page 320 
Cm_0 = Cm_0_wf + Cm_0_c + Cm_0_h;          % Equ(8.76) on page 320

return
