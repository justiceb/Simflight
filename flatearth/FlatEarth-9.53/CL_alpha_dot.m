%%CL_a_dot - Variation of airplane lift coefficient with
%%the rate of alpha
%
% Version 3.0 2/15/02. 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Assumptions:
%%CL_a_dot is the sum of the wing and horizontal tail
%%components, however, unless the wing is triangular in
%%shape, the wing component is assumed to be negligable.
%%This program does NOT incorporate triangular wings!
%%
%%AAE 421 Fall 2001
%%Brian K. Barnett
%%All equations from ref. 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%b_w 				 %%Wing span (ft)
%l_h        	 %%This is the horizontal distance from the wing mean quarter chord, to the horiz tail mean quarter chord (ft).
%h_h      	    %%This is the verical distance from the wing root trailing edge to the chord line of the horiz tail (ft).
%lambda  		 %%This is the wing taper ratio.
%AR_w 			 %%This is the wing aspect ratio.
%Lambda_c4      %%This is the wing sweep angle (rad).
%M 				 %%This is the flight Mach number.
%Cl_alpha 		 %%This is the slope of the wing section lift curve.
%q_bar_h 		 %%This is the dynamic pressure seen by the horiz tail (slug/(ft*sec^2))
%q_bar 			 %%This is the dynamic pressure of the freestream (slug/(ft*sec^2)).
%S_h 	        	 %%This is the surface area of the horiz tail (ft^2).
%Xh 				 %%This is the distance from a/c CG to horiz tail aerodynamic center (ft).
%c_w 				 %%This is the wing mean chord (ft).
%S_w 				 %%This is the surface area of the wing (ft^2).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [CL_alpha_dot] = CL_alpha_dot(l_h, h_h, b_w, lambda, AR_w, AR_h, Lambda_c4, Lambda_c4_h, beta, kappa, kappa_h, V_h, eta_h)


K_H=(1-(h_h./b_w))/(((2.*l_h)/b_w)^(1./3));  %eqn 3.15
K_lambda = (10-3*lambda)/7;       %eqn 3.14
K_A = 1/AR_w - 1/(1+AR_w^1.7);		%eqn 3.13

d_epsilon_over_dalpha_M_is_zero = 4.44*(K_A*K_lambda*K_H*sqrt(cos(Lambda_c4)))^1.19;    %eqn 3.12
% Lambda_c4 has to be in radians - Murthy
  
CL_alpha_w= 2*pi*AR_w/(2+sqrt((AR_w*beta/kappa)^2*(1+(tan(Lambda_c4))^2/beta^2)+4 ));  %eqn 3.8
% Lambda_c4 has to be in radians - Murthy
CL_alpha_w_M_is_zero= 2*pi*AR_w/(2+sqrt((AR_w*1/kappa)^2*(1+(tan(Lambda_c4))^2/1^2)+4 ));  %eqn 3.8 setting M=0
% Lambda_c4 has to be in radians - Murthy
d_epsilon_over_dalpha = d_epsilon_over_dalpha_M_is_zero*CL_alpha_w/CL_alpha_w_M_is_zero; %eqn 3.11

CL_alpha_H = (2*pi*AR_h)/(2+sqrt(AR_h^2*beta^2/kappa_h^2*(1+(tan(Lambda_c4_h))^2/beta^2)+4));    %eqn 3.8
% Lambda_c4 has to be in radians - Murthy
CL_alpha_H_dot = 2*CL_alpha_H*eta_h*V_h*d_epsilon_over_dalpha;   %eqn 6.3

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Since the wing is assumed not to be triangular, the wing term is negligable, therefore:

CL_alpha_dot = CL_alpha_H_dot;   %eqn 6.1
return
