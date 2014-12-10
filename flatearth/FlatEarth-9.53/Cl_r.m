%%Cl_r - Variation of airplane rolling moment coefficient with dimensionless
%%rate of change of yaw rate.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Cl_r is the sum of the wing and verical tail components.
%%This program only calculates for aircraft with a single vertical tail!!!
%
% Version 3.0 2/15/02. 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%
%%
%%
%%AAE 421 Fall 2001
%%Brian K. Barnett
%%All equations from ref. 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%alpha 			%%angle of attack
%S_h   			%%surface area of horizontal tail
%S_v  			%%surface area of vertical tail
%x_over_c_v    %%horizontal distance from leading edge of vertical fin mean chord to horizontal aero center
%b_v           %%vertical tail span measured from fuselage centerline
%Z_v 				%%the vertical distance from the aircraft CG to the vertical tail aero center
%l_v    			%%the horizontal distance from the aircraft CG to the vertical tail aero center
%Z_h      		%%the negative of the hoizontal distance from the fuselage centerline to the horizontal tail aero center (negative number)
%two_r_one     %%fuselage depth in region of vertical tail
%lambda_v      %%vertical tail taper ratio based on surface measured from fuselage centerline
%S_w           %%This is the surface area of the wing
%Lambda_c4  	%%This is the sweep angle of the wing at the quarter chord (rad)
%Z_w				%%This is the vertical distance from the wing root c/4 to the fuselage centerline, positive downward
%AR_w 			%%This is the a/c aspect ratio
%d 				%%Maximum body height at wing-body intersection
%Xw   			%%distance from the reference point to the aero center of the wing alone (postitve aft) (ft)
%c_w 				%%mean aero chord
%beta  			%%sqrt (1-M^2)
%AR_v 			%%Aspect ratio of the vertical tail
%eta_v 			%%Ratio of the dynmaic pressure at the vertical tail to that of the freestream
%b_w 				%%The span of the wing (ft)
%lambda_w 		%%This is the wing taper ratio.
%cf         	%%This is the length of the wing flap (ft)
%delta_f       %%This is the streamwise flap deflection (rad)
%theta     	   %%This is the wing twist in degrees, negative for washout (rad)
%Gamma   		%%This is the geometric dihedral angle, positive for dihedral, negative for anhedral (rad)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [Cl_r] = Cl_r(l_v, alpha, Z_v, S_h, S_v,x_over_c_v, b_v, Z_h, two_r_one, lambda_v, S_w, Z_w, d, AR_w, beta, Lambda_c4,c_w, Xw, AR_v, eta_v,b_w,cf,delta_f,theta, lambda_w ,Gamma,kappa,B)



%%Some derived constants

Xw_over_c_w = Xw/c_w;

S_h_over_S_v = S_h/S_v;

Z_h_over_b_v = -abs(Z_h/b_v);

b_v_over_two_r_one = b_v/two_r_one;

termb = 1/12*pi*AR_w*sin(Lambda_c4)/(AR_w+4*cos(Lambda_c4));  %%eqn 9.7







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%THE FOLLOWING CODE ONLY WORKS FOR A SINGLE VERTICAL TAIL!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

term1 = .724 + 3.06*((S_v/S_w)/(1+cos(Lambda_c4)))+.4*Z_w/d;  %%eqn 7.5

%%Interpolating in Figure 7.3
b_v_over_two_r_one_fig = [0.04978	0.48968	0.9977	1.506	1.985	2.491	2.9873	3.484	4.0019	4.5002	4.9986	5.5166	5.9855];
k_fig = [0.76234	0.75567	0.76091	0.75441	0.74784	0.83516	0.91853	0.98628	0.98763	0.99283	0.99022	0.98767	0.9928]; 
  %%Fig 7.3
k = interlim1(b_v_over_two_r_one_fig,k_fig,b_v_over_two_r_one);
%%Done interp fig 7.3

CL_a_v = (2*pi*A_v_eff)/(2+sqrt((A_v_eff*beta/kappa)^2*(1+(tan(Lambda_c4))^2/beta^2) + 4)); %added kappa - Murthy
  %%eqn 3.8 with A_v_eff subbed in for A (see pg 7.2)



C_y_beta_v = -k*CL_a_v*term1*eta_v*S_v/S_w;    %%eqn 7.4

Cl_r_v = -2/b_w^2*(l_v*cos(alpha) + Z_v*sin(alpha))*(Z_v*cos(alpha)-l_v*sin(alpha))*C_y_beta_v;  %eqn 9.8


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%This completes the vertical tail component
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%The following is used to compute the wing component
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%Interpolating in Figure 10.2 inset
matrixa = [0 .2 .5 .7 1];
matrixb = [0 -.55 -.8 -.91 -1]; %changed the last entry from 1 to -1 according to graph - Murthy
adf = interlim1(matrixa, matrixb, cf/c_w);  %%fig 10.2


%%The following is an approximation for figure 9.3 assumming only inboard flaps and average taper ratio and aspect ratio
%%This approximation is only important if flaps are deflected

terma = .003;

%%%Interpolating in Figure 9.2
if lambda_w>=.4;
   lambda_w==.4;
end

matrixa0 = [2 4 5 8 10];
matrixb0 = [-.0055 -.0082 -.0088 -.0082 -.0103];
lownum = interlim1(matrixa0, matrixb0, AR_w);

matrixa2 = [2 6 8 10];
matrixb2 = [-.007 -.0127 -.013 -.0144];
midnum = interlim1(matrixa2, matrixb2, AR_w);

matrixa4 = [2 6 8 10];
matrixb4 = [-.008 -.014 -.015 -.0171];
highnum = interlim1(matrixa4,matrixb4,AR_w);

newguy = [lownum midnum highnum];
newguy1 = [0 .2 .4];
Delta_CL_theta = interlim1(newguy1,newguy,lambda_w);

%%%Done interpolating in figure 9.2




%%Start interpolating in figure 9.1
matrixa0 = [2 4 8 10 20 25];
matrixb0 = [1.3 2.2 2.9 3 3 3];
num0 = interlim1(matrixa0, matrixb0, AR_w);

matrixa25 = [2 4 8 10 20 25];
matrixb25 = [2.2 3.4 4.4 4.5 4.5 4.5];
num25 = interlim1(matrixa25, matrixb25, AR_w);

matrixa5 = [2 4 8 10 20 25];
matrixb5 = [3.5 4.85 5.8 5.9 5.9 5.9];
num5 = interlim1(matrixa5, matrixb5, AR_w);

matrixa1 = [2 4 8 10 20 25];
matrixb1 = [4.8 6.1 7.2 7.4 7.4 7.4];
num1 = interlim1(matrixa1, matrixb1, AR_w);





midscale = [num0, num25, num5, num1];
midscale1 = [0 .25 .5 1];
midguy = interlim1(midscale1, midscale, lambda_w);



matrixa0 = [0 8];
matrixb0 = [.1 .3];
calc0 = interlim1(matrixa0, matrixb0, midguy);

matrixa15 = [0 8];
matrixb15 = [.12 .355];
calc15 = interlim1(matrixa15, matrixb15, midguy);

matrixa30 = [0 8];
matrixb30 = [.15 .435];
calc30 = interlim1(matrixa30, matrixb30, midguy);

matrixa45 = [0 8];
matrixb45 = [.185 .545];
calc45 = interlim1(matrixa45, matrixb45, midguy);

matrixa60 = [0 8];
matrixb60 = [.235 .675];
calc60 = interlim1(matrixa60, matrixb60, midguy);

finalmatrix = [calc0 calc15 calc30 calc45 calc60];
final1 = [0 15 30 45 60]*pi/180;
%changed to radians, since input is in radians

Clr_over_CL_Mzero = interlim1(final1, finalmatrix, Lambda_c4);

%%%%%%%%This completes the interpolation in Figure 9.1%%%%%%%%%%%


%Clr_over_CL_M = (1+(AR_w*(1-beta^2))/(2*beta*(AR_w*beta+2*cos(Lambda_c4)))+ (AR_w*beta+2*cos(Lambda_c4))/(AR_w*beta+4*cos(Lambda_c4))...
 %  *(tan(Lambda_c4))^2/8) / (1+(AR_w+2*cos(Lambda_c4))/(AR_w+4*cos(Lambda_c4))*(tan(Lambda_c4))^2/8)* Clr_over_CL_Mzero;  %%eqn 9.5
Clr_over_CL_M = (1+(AR_w*(1-B^2))/(2*beta*(AR_w*B+2*cos(Lambda_c4)))+ (AR_w*B+2*cos(Lambda_c4))/(AR_w*B+4*cos(Lambda_c4))...
   *(tan(Lambda_c4))^2/8) / (1+(AR_w+2*cos(Lambda_c4))/(AR_w+4*cos(Lambda_c4))*(tan(Lambda_c4))^2/8)* Clr_over_CL_Mzero;  %%eqn 9.5
%formula should have B and not beta. Hence the change.- Murthy


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%The total wing component is
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Cl_r_w = Clr_over_CL_M + termb*Gamma + Delta_CL_theta*theta*180/pi + terma*adf*delta_f*180/pi;
% added *180/pi since theta and delta_f should be in deg. according to the formula - Murthy
%Here Gamma has to be in rad.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Summing the wing and vertical tail components to yield the coefficient
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Cl_r = Cl_r_w + Cl_r_v;

return

