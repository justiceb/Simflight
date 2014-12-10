%[Cn_dr]=Cn_dr(S_w,b_w,S_h,S_v,b_v,c_v,x_AC_vh,two_r1,AR_v,l_v,Z_v, ...
%							eta_or,eta_ir,cr_cv,beta,kappa_v,Lambda_c2_v,lambda_v,delta_r,alpha)
%
%  Cn_dr - Variation of yawing moment coefficient  
%           with rudder deflection
%
% Version 3.0 2/15/02. 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%
%
%	[Ref.2]	Jan Roskam
%		Methods for Estimating
%		Stability and Control Derivatives
%		of Conventional Subsonic Airplanes  -  1977
%
%		S_w			- Reference area - main wing [ft^2]
%		b_w			- Wing span [ft]
%		S_h 			- Surface area - horizontal tail [ft^2]
%		S_v			- Surface area - vertical tail [ft^2]
%		b_v			- Vertical tail span [ft]
%		c_v			- Vertical mean chord [ft]
%		x_AC_vh		- X distance from LE of vertical tail to AC of horizontal tail
%		l_v			- X-Distance from aircraft CG to vertical tail AC
%		Z_v			- Z-Distance from aircraft CG to vertical tail AC
%		two_r1		- Fuselage depth in region of vertical tail
%		AR_v			- Aspect Ratio of vertical tail    
%		cr_ch			- Elevator flap chord to horizontal tail chord ratio 
%		eta_or		- Percent span position of outboard edge of rudder
%		eta_ir		- Percent span position of inboard edge of rudder
%		beta			- Compressibility correction factor (sqrt(1-Mach^2))
%		kappa_v		- Ratio of vertical tail 2D lift curve slope to 2pi
%		Lambda_c2_v - Vertical tail sweep angle at 1/2chord [rad] 
%		lambda_v		- Vertical tail taper ratio
%		delta_r 		- Rudder deflection [rad]
%

function [Cn_dr]=Cn_dr(S_w,b_w,S_h,S_v,b_v,c_v,x_AC_vh,two_r1,AR_v,l_v,Z_v, ...
							eta_or,eta_ir,cr_cv,beta,kappa_v,Lambda_c2_v,lambda_v,delta_r,alpha)


bv_2r=b_v/two_r1;
% Figure 7.5  -  Reference 2
bv_2r_fig = [0	0.25	0.5	1	1.5	2	2.5	3	3.5	4	4.5	5	5.5	6	6.5	7];
lambda_v_fig = [0 0.6 1];
Av_B_Av_fig = ...
[0.40	0.928	1.167	1.456	1.595	1.626	1.526	1.353	1.208	1.135	1.096	1.070	1.054	1.04	1.035	1.028; ...
0.400	0.928	1.167	1.456	1.595	1.626	1.526	1.353	1.208	1.135	1.096	1.070	1.054	1.04	1.035	1.028; ...
0.000	0.695	0.974	1.318	1.461	1.504	1.419	1.268	1.159	1.096	1.069	1.047	1.038	1.04	1.035	1.028];
Av_B_Av = interlim2(bv_2r_fig,lambda_v_fig,Av_B_Av_fig,bv_2r,lambda_v);

zH_bv=-Z_v/b_v;
x_cv=x_AC_vh/c_v;
% Figure 7.6  -  Reference 2
zH_bv_fig = [0	-0.1	-0.2	-0.3	-0.4	-0.5	-0.6	-0.7	-0.8	-0.9	-1];
x_cv_fig = [0.8 0.7 0.6 0.5];
Av_HB_Av_B_fig = ...
[1.2920	1.1900	1.0970	1.0190	0.9681	0.9343	0.9384	1.0020	1.126	1.364	1.7; ...
1.2200	1.1340	1.0590	0.9944	0.9453	0.9171	0.9251	0.9788	1.103	1.347	1.7; ...
1.1450	1.0770	1.0090	0.9525	0.9167	0.8943	0.8966	0.9597	1.086	1.328	1.7; ...
1.0500	0.9917	0.9407	0.9011	0.8730	0.8639	0.8775	0.9331	1.061	1.305	1.7];
Av_HB_Av_B = interlim2(zH_bv_fig,x_cv_fig,Av_HB_Av_B_fig,zH_bv,x_cv);

% Figure 7.7  -  Reference 2
Sh_Sv = S_h/S_v;
Sh_Sv_fig = ...
[0.00			0.059518	0.13083	0.19024	0.37989	0.51796	0.66361	0.85212	1.0836	1.3735	1.5652	1.7686	1.9995];
K_H_fig = ...
[0.0039038	0.09466	0.19724	0.28009	0.50117	0.63941	0.75409	0.85712	0.95639	1.0441	1.0763	1.1086	1.1449]; 
K_H=interlim1(Sh_Sv_fig,K_H_fig,Sh_Sv);

Av_eff=Av_B_Av*AR_v*(1+K_H*(Av_HB_Av_B-1));		%Eqn 7.6
CL_alpha_v=2*pi*Av_eff/(2+sqrt((Av_eff*beta/kappa_v)^2*(1+tan(Lambda_c2_v)^2/beta^2)+4)); 

% Figure 10.2  -  Reference 2
cf_c_fig = [0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1];
alpha_delta_cl_fig = [0	-0.394 -0.553 -0.667 -0.758 -0.829 -0.884 -0.93 -0.968 -0.994 -1];
alpha_delta_cl=interp1(cf_c_fig,alpha_delta_cl_fig,cr_cv);
AR_fig = [1 2 4 6 8 10];
alpha_delta_cl_fig = [-0.1 -0.2 -0.3 -0.4 -0.5 -0.6 -0.7 -0.8 -0.9 -1.0];
alpha_delta_ratio_fig = [1.2	1.25	1.33	1.46	1.78	2.2	;...
		1.14	1.16	1.2	1.3	1.49	1.71	;...	
		1.1	1.11	1.14	1.2	1.36	1.52	;...
		1.06	1.08	1.1	1.14	1.25	1.39	;...
		1.05	1.06	1.07	1.1	1.18	1.28	;...
		1.03	1.04	1.05	1.07	1.13	1.2	;...
		1.02	1.03	1.03	1.05	1.09	1.14	;...
		1.01	1.01	1.02	1.03	1.06	1.09	;...
		1.01	1.01	1.01	1.02	1.03	1.05	;...
		1	1	1	1	1	1	];
alpha_delta_ratio=interlim2(AR_fig,alpha_delta_cl_fig,alpha_delta_ratio_fig,Av_eff,alpha_delta_cl);

% Figure 10.7  -  Reference 2
delta_f_fig = [0 10 15 20 25 30 35 40 45 50 55 60]*pi/180;%converted to rad. - Murthy
cf_c_fig = [0.1 0.15 0.2 0.25 0.3 0.4 0.5];
K_prime_fig = [1	1	0.982	0.901	0.787	0.716	0.674	0.642	0.618	0.596	0.578	0.563	;...	
		1	1	0.982	0.889	0.769	0.697	0.653	0.62	0.591	0.569	0.549	0.531	;...	
		1	1	0.982	0.878	0.743	0.674	0.63	0.597	0.568	0.544	0.522	0.502	;...
		1	1	0.982	0.859	0.704	0.644	0.602	0.57	0.541	0.518	0.498	0.481	;...
		1	1	0.981	0.817	0.669	0.611	0.573	0.543	0.517	0.496	0.477	0.46	;...
		1	1	0.951	0.75	0.633	0.578	0.543	0.515	0.492	0.473	0.455	0.439	;...
		1	1	0.918	0.696	0.597	0.546	0.514	0.49	0.469	0.453	0.437	0.423	];	
K_prime=interlim2(delta_f_fig,cf_c_fig,K_prime_fig,delta_r,cr_cv);

% Figure 10.3  -  Reference 2
eta_fig = [0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1];
lambda_fig = [0 0.5 1.0];
K_b_fig = [0	0.149	0.3	0.433	0.555	0.669	0.768	0.857	0.933	0.986	1	;...	
		0	0.132	0.263	0.393	0.514	0.624	0.729	0.829	0.916	0.975	1	;...
      0	0.12	0.246	0.368	0.483	0.598	0.701	0.799	0.888	0.957	1] ;	
K_bo=interlim2(eta_fig,lambda_fig,K_b_fig,eta_or,lambda_v);
K_bi=interlim2(eta_fig,lambda_fig,K_b_fig,eta_ir,lambda_v);
K_b=K_bo-K_bi;

Cy_dr=CL_alpha_v*alpha_delta_ratio*abs(alpha_delta_cl)*K_prime*K_b*S_v/S_w;		% Eqn 12.1 [ref 2]
%removed negative sign in front - Murthy <<< What does this mean? -Andrisani
Cn_dr=-Cy_dr*(l_v*cos(alpha)+Z_v*sin(alpha))/b_w; %Equation 12.3

return
