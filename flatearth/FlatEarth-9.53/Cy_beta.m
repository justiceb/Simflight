%%Cy_beta - Variation of Side Force Coefficient with Sideslip Angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Cy_beta is the sum of the wing, body, and verical tail components.
%
% Version 8.0 1/23/03 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%
%%This program only calculates for aircraft with a single vertical tail!!!
%%
%%
%%AAE 421 Fall 2001
%%Brian K. Barnett
%%All equations from ref. 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%S_h 			   %%surface area of horizontal tail
%S_v  			%%surface area of vertical tail
%x_over_c_v    %%horizontal distance from leading edge of vertical fin mean chord to horizontal aero center
%b_v           %%vertical tail span measured from fuselage centerline
%Z_h      		%%the negative of the hoizontal distance from the fuselage centerline to the horizontal tail aero center (negative number)
%two_r_one     %%fuselage depth in region of vertical tail
%lambda_v      %%vertical tail taper ratio based on surface measured from fuselage centerline
%S_w        	%%This is the surface area of the wing
%Lambda_c4  	%%This is the sweep angle of the wing at the quarter chord (rad)
%Z_w 				%%This is the vertical distance from the wing root c/4 to the fuselage centerline, positive downward
%d 				%%Maximum body height at wing-body intersection
%beta 			%%sqrt (1-M^2)
%AR_v 			%%Aspect rato of the vertical tail
%eta_v 			%%Ratio of the dynmaic pressure at the vertical tail to that of the freestream
%dihedral      %%This is the geometric dihedral angle of the wing (deg)
%wingloc   	   %%If the aircraft is a highwing: (wingloc=1), low-wing: (wingloc=0)
%Z_w1 	      %%distance from body centerline to c/4 of wing root chord, positive for c/4 point below body centerline (ft) %%fig 7.1 
%S_o 	    		%%Take X1/l_b and plug into: .378+.527*(X1/l_b)=(Xo/l_b), S_o is the fuselage x-sectional area at Xo. (ft^2) %%fig 7.2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Cy_beta] = Cy_beta(Cl_alpha_w,two_r_one,eta_v,beta,AR_v,b_v,Z_h,x_over_c_v,lambda_v,S_v,S_w,Lambda_c4,Lambda_c4_v,Z_w,d,dihedral,wingloc,Z_w1,S_h,S_o)


	
%%Some derived constants

b_v_over_two_r_one = b_v/two_r_one;

S_h_over_S_v = S_h/S_v;

Z_h_over_b_v = -abs(Z_h/b_v);

fig7_1_xaxis = 2*Z_w1/d;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%The vertical tail component is given below:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%The following is used to find the vertical tail component

%%Interpolating in Figure 7.6
Z_h_over_b_v_fig8 = ...
[-0.014442	-0.075377	-0.17057	-0.30375	-0.38361	-0.43113	-0.48242	-0.52417	-0.56402	-0.6171	-0.67203	-0.71932	-0.74955	-0.7911	-0.82694	-0.87399	-0.9322	-0.97536];
A_v_hb_over_A_v_b_fig8 = ...
[1.2741	1.2141	1.1242	1.0272	0.97877	0.95656	0.94198	0.93866	0.93912	0.95489	0.98205	1.0243	1.0588	1.1161	1.1734	1.2839	1.4552	1.5921]; 
%%x/c_v=.8, Fig 7.6
%data1=[Z_h_over_b_v_fig8',A_v_hb_over_A_v_b_fig8']
A_v_hb_over_A_v_b8 = interlim1(Z_h_over_b_v_fig8, A_v_hb_over_A_v_b_fig8,Z_h_over_b_v);

Z_h_over_b_v_fig7 = ...
[-0.010859	-0.13075	-0.24678	-0.34946	-0.46538	-0.56029	-0.64558	-0.73449	-0.7874	-0.859	-0.89846	-0.9585	-0.97914];
A_v_hb_over_A_v_b_fig7 = ...
[1.2145	1.117	1.0386	0.97908	0.93121	0.92121	0.94924	1.027	1.0889	1.2313	1.3388	1.5306	1.5959]; 
   %%x/c_v=.7, Fig 7.6
A_v_hb_over_A_v_b7 = interlim1(Z_h_over_b_v_fig7, A_v_hb_over_A_v_b_fig7,Z_h_over_b_v);

Z_h_over_b_v_fig6 = ...
[-0.0092336	-0.10051	-0.23366	-0.3439	-0.41987	-0.51294	-0.60968	-0.68163	-0.76667	-0.85531	-0.9117	-0.97913];
A_v_hb_over_A_v_b_fig6 = ...
[1.1381	1.0822	0.99259	0.94081	0.92288	0.89757	0.90669	0.94979	1.0466	1.2007	1.3504	1.5997]; 
   %%x/c_v=.6, Fig 7.6
A_v_hb_over_A_v_b6 = interlim1(Z_h_over_b_v_fig6, A_v_hb_over_A_v_b_fig6,Z_h_over_b_v);

Z_h_over_b_v_fig5 = ...
[-0.01143	-0.16927	-0.27951	-0.40107	-0.532	-0.6552	-0.76115	-0.84601	-0.93625	-0.98294];
A_v_hb_over_A_v_b_fig5 = ...
[1.0541	0.95724	0.90546	0.87295	0.87113	0.9112	1.0006	1.1471	1.3852	1.5959]; 
  %%x/c_v=.5, Fig 7.6
A_v_hb_over_A_v_b5 = interlim1(Z_h_over_b_v_fig5, A_v_hb_over_A_v_b_fig5,Z_h_over_b_v);

Matrixa = [.5, .6, .7, .8];
Matrixb = [A_v_hb_over_A_v_b5, A_v_hb_over_A_v_b6, A_v_hb_over_A_v_b7, A_v_hb_over_A_v_b8];
A_v_hb_over_A_v = interlim1(Matrixa, Matrixb, x_over_c_v);
%%Done interp fig 7.6

%%Interpolating in Figure 7.5
%%The referenced figure assumes all values of lambda_v less than .6 are estimated at .6


if lambda_v < .6;
   lambda_v = .6;
end

b_v_over_two_r_one_fig1 = ...
[0.0089225	0.13036	0.2147	0.35595	0.60146	0.79048	1.0743	1.3015	1.5668	1.8701	2.0883	2.2782	2.5345	2.753	3.0286	3.5224	4.0159	4.5378	5.0216	5.5054	5.9986	6.4918	6.9377];
A_v_b_over_A_v_fig1 = ...
[0.10677	0.4613	0.6557	0.85004	1.0595	1.189	1.3374	1.421	1.4741	1.5081	1.5041	1.4696	1.4235	1.3508	1.2666	1.1555	1.0901	1.0629	1.0433	1.0351	1.0346	1.0378	1.0221]; 
   %%lambda_v=.1, figure 7.5
b_v_over_two_r_one_1 = interlim1(b_v_over_two_r_one_fig1,A_v_b_over_A_v_fig1,b_v_over_two_r_one);

b_v_over_two_r_one_fig6 = ...
[0.0079378	0.091375	0.1853	0.27945	0.42102	0.55323	0.7139	1.0262	1.3861	1.6513	1.8789	2.0402	2.2395	2.5245	2.7525	3.0281	3.5316	4.0252	4.5281	5.012	5.5148	6.0081	6.4824	6.9567];
A_v_b_over_A_v_fig6 = ...
[0.29364	0.65965	0.83497	0.96834	1.1017	1.2121	1.3225	1.4633	1.5659	1.6228	1.634	1.63	1.5993	1.5265	1.4462	1.3467	1.2051	1.1359	1.101	1.0738	1.0542	1.0307	1.0264	1.0259]; 
   %%lambda_v<=.6
b_v_over_two_r_one_6 = interlim1(b_v_over_two_r_one_fig6,A_v_b_over_A_v_fig6,b_v_over_two_r_one);

Matrixa = [.6, 1];
Matrixb = [b_v_over_two_r_one_6,b_v_over_two_r_one_1];
A_v_b_over_A_v = interlim1(Matrixa, Matrixb, lambda_v);
%%Done interp fig 7.5


%%Interpolating in Figure 7.7
S_h_over_S_v_fig = ...
[0.00			0.059518	0.13083	0.19024	0.37989	0.51796	0.66361	0.85212	1.0836	1.3735	1.5652	1.7686	1.9995];
K_H_fig = ...
[0.0039038	0.09466	0.19724	0.28009	0.50117	0.63941	0.75409	0.85712	0.95639	1.0441	1.0763	1.1086	1.1449]; 
  %%Fig 7.7
K_H = interlim1(S_h_over_S_v_fig, K_H_fig, S_h_over_S_v);
%%Done interp Figure 7.7

A_v_eff = A_v_b_over_A_v*AR_v*(1+K_H*(A_v_hb_over_A_v-1));  %%eqn 7.6

term1 = .724 + 3.06*((S_v/S_w)/(1+cos(Lambda_c4_v)))+.4*Z_w/d+0.009*A_v_eff;  %%eqn 7.5

%%Interpolating in Figure 7.3
b_v_over_two_r_one_fig = [0.04978	0.48968	0.9977	1.506	1.985	2.491	2.9873	3.484	4.0019	4.5002	4.9986	5.5166	5.9855];
k_fig = [0.76234	0.75567	0.76091	0.75441	0.74784	0.83516	0.91853	0.98628	0.98763	0.99283	0.99022	0.98767	0.9928]; 
  %%Fig 7.3
k = interlim1(b_v_over_two_r_one_fig,k_fig,b_v_over_two_r_one);
%%Done interp fig 7.3

CL_a_v = (2*pi*A_v_eff)/(2+sqrt((A_v_eff^2*beta^2/((Cl_alpha_w/(2*pi))^2))*(1+(tan(Lambda_c4_v))^2/beta^2) + 4)); 
  %%eqn 3.8 with A_v_eff subbed in for A (see pg 7.2)

Cy_beta_v = -k*CL_a_v*term1*S_v/S_w;    %%eqn 7.4

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%End of vertical tail component
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Wing component--Only significant if dihedral angle not equal to zero %%pg 7.1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Cy_beta_w = -.0001*dihedral*57.3;	%%eqn. 7.2 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Start of Body Component %%pg 7.1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if wingloc == 0;
	matrixa = [0 1];
	matrixb = [1 1.5]; %changed [0 1.5] to [1 1.5] based on Fig.7.1
	K_i = interlim1(matrixa,matrixb,fig7_1_xaxis);
else
	matrixa = [0 -1];
	matrixb = [1 1.85];%changed [0 1.85] to [1 1.85] based on Fig.7.1
	K_i = interlim1(matrixa,matrixb,fig7_1_xaxis);
end

Cy_beta_b = -2*K_i*S_o/S_w;  %%eqn. 7.3



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Summing all components to get the total
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Cy_beta = Cy_beta_w + Cy_beta_b + Cy_beta_v;


