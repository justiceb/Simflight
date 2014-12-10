function [Cl_p]=Cl_p(b_h,b_w,Z_v,two_r_one,eta_v,AR_v,b_v,Z_h,x_over_c_v,lambda_v,S_v,S_w,Lambda_c4,Lambda_c4_h,Z_w,d,lambda_w,wingloc,Z_w1,S_h,S_o,beta,...
   Cl_alpha,Cl_alpha_h,AR_w,lambda_h, AR_h)% *********************************************
% Cl_p - Variation of rolling moment coefficient
%        with roll rate.
% *********************************************     
%
% Version 3.0 2/15/02. 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%
%
% Assumptions:
% Cl_p is considered to be the sum of the wing/bod and horizontal and vertical tail contributions.
% 
%
% Required Inputs:
% b_h              span of the horizontal tail [ft]
% b_w              span of the wing [ft]
% two_r_one        fuselage depth in region of vertical tail [ft]
% eta_v            Ratio of dynamic pressure at the vertical tail to that of the free stream
% AR_v             Aspect ratio of the horrizontal tail
% b_v              span of the vertical tail [ft]
% Z_h              the negative of the hoizontal distance from the fuselage centerline to the horizontal tail aero center (negative number)
% x_over_c_v       horizontal distance from leading edge of vertical fin mean chord to horizontal aero center
% lambda_v         vertical tail taper ratio based on surface measured from fuselage centerline
% S_v              surface area of vertical tail [ft^2]
% S_w              surface area of the wing [ft^2]
% d                Maximum body height at wing-body intersection
% wingloc          If the aircraft is a highwing: (wingloc=1), low-wing: (wingloc=0)
% S_h              surface area of horizontal tail [ft^2]
% S_o	             Take X1/l_b and plug into: .378+.527*(X1/l_b)=(Xo/l_b), S_o is the fuselage x-sectional area at Xo. (ft^2) %%fig 7.2
% l_v              Horizontal distance from the aircraft CG to the vertical tail aero center
% Z_v              Vertical distance from the aircraft CG to the vertical tail aero center
% Lambda_c4        Quater Chord Sweep angle [rad]
% Lambda_c4_h      Quater Chord Sweep angle of the horizontal tail [rad]
% Z_w              vertical distance from the wing root c/4 to the fuselage centerline, positive downward
% Z_w1             distance from body centerline to c/4 of wing root chord, positive for c/4 point below body centerline (ft) %%fig 7.1 
% Cl_alpha         Two dimensional lift curve slope of the wing
% Cl_alpha_h       Two dimensional lift curve slope of the horizontal tail
% lambda_w         Taper ratio of the wing
% lambda_h         Taper ratio of the horizontal tail
% AR_h             Aspect ratio of the horrizontal tail
% AR_w             Aspect ratio of the wing
% Xw               (positive reaward) distance from the arbitrary reference point to wing aerodynamic center [ft]
% eta_h            Ratio of dynamic pressure at the tail to that of the free stream
% c_w              Mean aerodynamic chord of the wing [ft]
% c_h              Mean aerodynamic chord of the horizontal tail [ft]
% V_h              Horizontal Tail Volume Coefficient
% Xh               Distance from arbitrary refernce point to the horizontal tail aerodynamic center [ft]
% B                Compressibility correction factor
% beta             Compressibility correction factor 
% M                Mach number
% Lambda           Mid chord sweep angle
% l_H              Defined in Fig 3.7 of Ref(2) Geometric parameter for horizontal tail location [ft]
% h_H              Defined in Fig 3.7 of Ref(2) Geometric parameter for horizontal tail location [ft]
%
%
% A&AE 421 Fall 2001
% Jaret Matthews
% 
% Equations/Figures can be found in :
% Ref. (2) Roskam, Jan. "Methods for Estimating Satbility and
%         Control Derivatives of Conventional Subsonic Airplanes"
%         


kapa=Cl_alpha/(2*pi);
kapa_h=Cl_alpha_h/(2*pi);
bA_k=(beta*AR_w)/kapa;
bA_k_h=(beta*AR_h)/kapa_h;
lam_b=atan((tan(Lambda_c4))/beta);
lam_b_h=atan((tan(Lambda_c4_h))/beta);

%--------------Figure 8.1 data----------
lam_b_fig=[-20 -10 0 10 20 30 40 50 60 70]*pi/180;%converted to radians as input is in radians
bA_k_fig=[10	9	8	7	6	5	4.5	4	3.5	3	2.5	2	1.5];
lambda_fig=[0 .25 .5 1];
fig_8_3(:,:,1)=[-0.346	-0.337	-0.324	-0.309	-0.29	-0.267	-0.255	-0.24	-0.226	-0.202	-0.183	-0.16	-0.13	;...
-0.367	-0.355	-0.339	-0.322	-0.302	-0.281	-0.268	-0.251	-0.233	-0.21	-0.189	-0.162	-0.131	;...
-0.378	-0.366	-0.35	-0.332	-0.312	-0.289	-0.276	-0.259	-0.241	-0.216	-0.193	-0.164	-0.131	;...
-0.38	-0.369	-0.352	-0.336	-0.316	-0.292	-0.278	-0.262	-0.244	-0.219	-0.196	-0.166	-0.132	;...
-0.375	-0.363	-0.349	-0.335	-0.312	-0.289	-0.276	-0.261	-0.245	-0.22	-0.196	-0.166	-0.131	;...
-0.361	-0.348	-0.338	-0.323	-0.304	-0.282	-0.269	-0.258	-0.241	-0.218	-0.195	-0.165	-0.131	;...
-0.336	-0.325	-0.313	-0.303	-0.285	-0.267	-0.259	-0.244	-0.231	-0.211	-0.193	-0.163	-0.13	;...
-0.296	-0.286	-0.278	-0.268	-0.257	-0.243	-0.235	-0.227	-0.212	-0.199	-0.183	-0.16	-0.128	;...
-0.243	-0.235	-0.23	-0.225	-0.213	-0.207	-0.199	-0.194	-0.184	-0.175	-0.162	-0.147	-0.121	;...
-0.175	-0.172	-0.168	-0.168	-0.162	-0.162	-0.15	-0.15	-0.144	-0.142	-0.129	-0.121	-0.104	];

fig_8_3(:,:,2)=[-0.466	-0.446	-0.428	-0.404	-0.376	-0.342	-0.324	-0.302	-0.278	-0.247	-0.217	-0.181	-0.141;...
-0.485	-0.467	-0.443	-0.417	-0.385	-0.353	-0.33	-0.307	-0.281	-0.25	-0.22	-0.183	-0.144;...
-0.495	-0.477	-0.449	-0.423	-0.392	-0.359	-0.335	-0.31	-0.282	-0.253	-0.222	-0.183	-0.143;...
-0.495	-0.475	-0.45	-0.423	-0.392	-0.359	-0.333	-0.31	-0.284	-0.254	-0.222	-0.184	-0.145;...
-0.483	-0.465	-0.441	-0.416	-0.387	-0.354	-0.331	-0.308	-0.284	-0.254	-0.222	-0.184	-0.145;...
-0.46	-0.442	-0.421	-0.401	-0.375	-0.344	-0.324	-0.301	-0.277	-0.249	-0.22	-0.182	-0.145;...
-0.421	-0.409	-0.39	-0.373	-0.354	-0.327	-0.313	-0.29	-0.267	-0.244	-0.216	-0.182	-0.144;...
-0.367	-0.355	-0.344	-0.334	-0.319	-0.298	-0.286	-0.27	-0.252	-0.233	-0.21	-0.175	-0.141;...
-0.303	-0.29	-0.286	-0.28	-0.27	-0.257	-0.249	-0.239	-0.226	-0.209	-0.193	-0.165	-0.136;...
-0.219	-0.219	-0.214	-0.211	-0.203	-0.198	-0.196	-0.186	-0.18	-0.17	-0.162	-0.141	-0.126];

fig_8_3(:,:,3)=[-0.523	-0.5	-0.469	-0.441	-0.405	-0.367	-0.341	-0.316	-0.29	-0.261	-0.221	-0.184	-0.146;...
-0.534	-0.51	-0.48	-0.451	-0.416	-0.374	-0.347	-0.32	-0.293	-0.262	-0.224	-0.185	-0.146;...
-0.539	-0.513	-0.485	-0.454	-0.419	-0.378	-0.352	-0.323	-0.295	-0.262	-0.224	-0.185	-0.146;...
-0.534	-0.509	-0.481	-0.452	-0.416	-0.375	-0.35	-0.324	-0.295	-0.263	-0.224	-0.185	-0.145;...
-0.519	-0.498	-0.471	-0.444	-0.409	-0.37	-0.347	-0.321	-0.293	-0.262	-0.222	-0.185	-0.145;...
-0.494	-0.476	-0.455	-0.43	-0.394	-0.36	-0.34	-0.316	-0.288	-0.258	-0.22	-0.183	-0.145;...
-0.456	-0.442	-0.42	-0.402	-0.371	-0.342	-0.325	-0.302	-0.279	-0.251	-0.217	-0.179	-0.143;...
-0.407	-0.391	-0.376	-0.358	-0.337	-0.315	-0.3	-0.281	-0.266	-0.24	-0.21	-0.174	-0.141;...
-0.338	-0.323	-0.313	-0.3	-0.289	-0.274	-0.261	-0.25	-0.238	-0.22	-0.194	-0.164	-0.135;...
-0.251	-0.244	-0.238	-0.231	-0.22	-0.212	-0.208	-0.2	-0.192	-0.182	-0.167	-0.144	-0.123];

fig_8_3(:,:,4)=[-0.565	-0.535	-0.509	-0.468	-0.429	-0.385	-0.36	-0.333	-0.297	-0.266	-0.23	-0.189	-0.145;...
-0.573	-0.545	-0.516	-0.477	-0.436	-0.388	-0.364	-0.334	-0.302	-0.267	-0.231	-0.19	-0.145;...
-0.575	-0.547	-0.517	-0.477	-0.437	-0.39	-0.364	-0.334	-0.302	-0.267	-0.233	-0.19	-0.145;...
-0.568	-0.54	-0.513	-0.473	-0.432	-0.387	-0.364	-0.334	-0.302	-0.267	-0.233	-0.189	-0.146;...
-0.549	-0.526	-0.498	-0.463	-0.424	-0.38	-0.357	-0.331	-0.3	-0.264	-0.231	-0.187	-0.145;...
-0.521	-0.499	-0.477	-0.446	-0.41	-0.37	-0.347	-0.323	-0.295	-0.261	-0.228	-0.186	-0.145;...
-0.481	-0.467	-0.444	-0.421	-0.385	-0.352	-0.334	-0.311	-0.285	-0.256	-0.223	-0.182	-0.145;...
-0.429	-0.414	-0.396	-0.377	-0.349	-0.328	-0.311	-0.29	-0.271	-0.244	-0.217	-0.177	-0.143;...
-0.369	-0.351	-0.339	-0.321	-0.3	-0.285	-0.272	-0.259	-0.241	-0.225	-0.2	-0.166	-0.138;...
-0.284	-0.274	-0.261	-0.246	-0.236	-0.228	-0.218	-0.21	-0.2	-0.186	-0.169	-0.145	-0.123];

BClp_Kap=interlim3(bA_k_fig,lam_b_fig,lambda_fig,fig_8_3,bA_k,lam_b,lambda_w);
BClp_Kap_h=interlim3(bA_k_fig,lam_b_fig,lambda_fig,fig_8_3,bA_k_h,lam_b_h,lambda_h);

Cl_p_WB=BClp_Kap*(kapa/beta);	%Ref. 2, pg 8.1, eqn 8.3
Cl_p_W=Cl_p_WB;	%Ref. 2, pg 8.1, eqn 8.3
Cl_ph=BClp_Kap_h*(kapa_h/beta);	%Ref. 2, pg 8.1
Cl_p_H=.5*Cl_ph*(S_h/S_w)*(b_h/b_w)^2;	%Ref. 2, pg 8.1, eqn 8.4
%%The following is used to find Cy_beta_v
%%Some derived constants

b_v_over_two_r_one = b_v/two_r_one;

S_h_over_S_v = S_h/S_v;

Z_h_over_b_v = -abs(Z_h/b_v);

fig7_1_xaxis = 2*Z_w1/d;

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

CL_a_v = (2*pi*A_v_eff)/(2+sqrt(A_v_eff^2*beta^2*(1+(tan(Lambda_c4))^2/beta^2) + 4)); 
  %%eqn 3.8 with A_v_eff subbed in for A (see pg 7.2)

Cy_beta_v = -k*CL_a_v*term1*eta_v*S_v/S_w;    %%eqn 7.4

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%End of Cy_beta_v
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Cl_p_V=2*(Z_v/b_v)^2*Cy_beta_v;		%Ref. 2, pg 8.2, eqn 8.5
Cl_p=Cl_p_WB+Cl_p_H+Cl_p_V; 	%Ref. 2, pg 8.1, eqn 8.2
