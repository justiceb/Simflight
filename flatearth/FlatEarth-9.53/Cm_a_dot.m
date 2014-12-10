%%Cm_a_dot - Variation of airplane moment coefficient with
%%the rate of alpha
%
% Version 3.0 2/15/02. 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Assumptions:
%%Cm_a_dot is the sum of the wing and horizontal tail
%%components, however, unless the wing is triangular in
%%shape, the wing component is assumed to be negligable.
%%This program does NOT incorporate triangular wings!
%%
%%AAE 421 Fall 2001
%%Brian K. Barnett
%%All equations from ref. 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%l_h        %%This is the horizontal distance from the wing mean quarter chord, to the horiz tail mean quarter chord (ft).
%h_h        %%This is the verical distance from the wing root trailing edge to the chord line of the horiz tail (ft).
%b_w 		%%This is the wing span (ft).
%lambda_w 	%%This is the wing taper ratio.
%AR_w 		%%This is the wing aspect ratio.
%Lambda_c4  %%This is the wing sweep angle (rad).
%M 			%%This is the flight Mach number.
%Cl_alpha 	%%This is the slope of the wing section lift curve.
%eta_h      %%This is the dynamic pressure ratio at the horizontal tail
%S_h 	    %%This is the surface area of the horiz tail (ft^2).
%Xh 		%%This is the distance from a/c CG to horiz tail aerodynamic center (ft).
%c_w 		%%This is the wing mean chord (ft).
%S_w 		%%This is the surface area of the wing (ft^2).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Cm_alpha_dot] = Cm_alpha_dot(l_h, h_h, b_w, lambda_w, AR_w, Lambda_c4, M, Cl_alpha, eta_h, S_h, Xh, c_w, S_w,kappa,kappa_h,Lambda_c2,AR_h)

K_H = (1-h_h/b_w)/(2*l_h/b_w)^(1/3);  %eqn 3.15

K_lambda_w = (10-3*lambda_w)/7;       %eqn 3.14

K_A = 1/AR_w - 1/(1+AR_w^1.7);		%eqn 3.13

d_epsilon_over_dalpha_M_is_zero = 4.44*(K_A*K_lambda_w*K_H*sqrt(cos(Lambda_c4)))^1.19;    %eqn 3.12

Beta = sqrt(1-M^2);  %eqn 3.9

CL_alpha_w= 2*pi*AR_w/(2+sqrt((AR_w*Beta/kappa)^2*(1+(tan(Lambda_c2))^2/Beta^2)+4 ));  %eqn 3.8

CL_alpha_w_M_is_zero= 2*pi*AR_w/(2+sqrt((AR_w*1/kappa)^2*(1+(tan(Lambda_c2))^2/1^2)+4 ));  %eqn 3.8 setting M=0
%kappa has to be there in above two formulae and not K_A
d_epsilon_over_dalpha = d_epsilon_over_dalpha_M_is_zero*CL_alpha_w/CL_alpha_w_M_is_zero; %eqn 3.11

K = Cl_alpha/(2*pi);  %Actual wing section lift curve slope over 2*PI

CL_alpha_H=2*pi*AR_h/(2+sqrt((AR_h*Beta/kappa_h)^2*(1+(tan(Lambda_c2))^2/Beta^2)+4 )); %From equation 3.8 on page 3.2


V_H_bar = Xh*S_h/c_w/S_w;  %eqn on pg vi, last line

Cm_alpha_H_dot = -2*CL_alpha_H*eta_h*V_H_bar*Xh/c_w*d_epsilon_over_dalpha;   %eqn 6.5


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Since the wing is assumed not to be triangular, the wing term is negligable, therefore:

Cm_alpha_dot = Cm_alpha_H_dot;   %eqn 6.4
return
