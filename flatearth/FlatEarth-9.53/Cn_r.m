%%Cn_r - Variation of airplane yawing momentcoefficient with dimensionless
%%rate of change of yaw rate.
%
% Version 3.0 2/15/02. 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%CL_a_dot is the sum of the wing and verical tail components.
%%This program only calculates for aircraft with a single vertical tail!!!
%%Don't try this programming at home, folks.
%%
%%
%%AAE 421 Fall 2001
%%Brian K. Barnett
%%All equations from ref. 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%alpha  			%%angle of attack [rad]
%S_h 				%%surface area of horizontal tail
%S_v  			%%surface area of vertical tail
%CL 			%%3-D lift coefficient
%C_bar_D_o     %%Parasite drag
%x_over_c_v     PARAMETER ACCOUNTING FOR THE RELATIVE POSITIONS OF THE HORIZONTAL AND VERTICAL TAILS
%				defined as the fore-and-aft distance from leading edge of vertical fin to the
%				aerodynamic center of the horizontal tail divided by the chord of the vertical tail 
%				[nondimensional] - See Fig 7.6 of Ref. 2
%b_v           %%vertical tail span measured from fuselage centerline
%Z_v 			%%the vertical distance from the aircraft CG to the vertical tail aero center
%l_v   			%%the horizontal distance from the aircraft CG to the vertical tail aero center
%Z_h       		%%the negative of the VERTICAL distance from the fuselage centerline to the horizontal tail aero center (negative number for normal (high) vertical tails) See Figure 7.6 of Ref 2.
%two_r_one     %%fuselage depth in region of vertical tail
%lambda_v      %%vertical tail taper ratio based on surface measured from fuselage centerline
%S_w           %%This is the surface area of the wing
%Lambda_c4  	%%This is the sweep angle of the wing at the quarter chord (rad)
%Z_w 				%%This is the vertical distance from the wing root c/4 to the fuselage centerline, positive downward
%AR_w 			%%This is the a/c aspect ratio
%d 				%%Maximum body height at wing-body intersection
%Xw  			%%distance from the reference point to the aero center of the wing only (positive aft)
%c_w 			%%mean aero chord
%beta 			%%sqrt (1-M^2)
%AR_v       	%%Aspect ratio of the vertical tail
%eta_v			%%Ratio of the dynmaic pressure at the vertical tail to that of the freestream
%b_w 				%%Wing span (ft)
%Lambda_c4     %%This is the wing sweep angle (rad).
%c_w 				%%This is the wing mean chord (ft).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [Cn_r] = Cn_r(CL, C_bar_D_o, l_v, alpha, Z_v, S_h, S_v,x_over_c_v, b_v, Z_h, two_r_one, lambda_v, S_w, Z_w, d, AR_w, beta, Lambda_c4,c_w, Xw, AR_v, eta_v,b_w)



%%Some derived constants

Xw_over_c_w = Xw/c_w;

S_h_over_S_v = S_h/S_v;

Z_h_over_b_v = -abs(Z_h/b_v);

b_v_over_two_r_one = b_v/two_r_one;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%The following is used to find the WING component of the total moment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%The following is a small adjustment made to correct for when Figure 9.4 data was imported--there can't be A exactly = 10 the way it was imported
if AR_w > 25
   AR_w=25
end

%%Interpolating in Figure 9.4 for Xw/c_w=0
A_fig_0_xc_0 = ...
[1.0852	1.3954	1.8608	2.2824	3.0338	4.0381	4.9805	6.0189	6.9787	8.0329	9.0254	9.8602 20 25];
left_fig_0_xc_0 = ...
[0.022875	0.098083	0.20914	0.26735	0.33771	0.39527	0.43499	0.45062	0.46235	0.47806	0.47241	0.46949 0.46949 0.46949]; 
  %%Figure 9.4, Xw/c_w=0, Lambda_c4=0
A_0_xc_0 = interlim1(A_fig_0_xc_0,left_fig_0_xc_0,AR_w);

A_fig_40_xc_0 = ...
[1.0595	1.6205	2.042	3.0611	4.0501	5.0246	6.0311	7.0071	8.0145	9.0378	9.9675 20 25];
left_fig_40_xc_0 = ...
[0.18064	0.27463	0.33284	0.40451	0.45498	0.48433	0.50682	0.51161	0.52007	0.5251	0.51563 0.51563 0.51563];
A_40_xc_0 = interlim1(A_fig_40_xc_0,left_fig_40_xc_0,AR_w);

A_fig_50_xc_0 = ...
[1.0484	2.0354	3.0405	3.9993	5.006	6.0284	7.0043	7.9968	9.0205	9.9811 20 25];
left_fig_50_xc_0 = ...
[0.35603	0.43807	0.4816	0.51087	0.52985	0.54891	0.55721	0.55156	0.54957	0.55078 0.55078 0.55078];
A_50_xc_0 = interlim1(A_fig_50_xc_0,left_fig_50_xc_0,AR_w);

A_fig_60_xc_0 = ...
[1.0936	1.7747	2.4546	3.0539	4.0162	5.0244	6.0329	7.0094	8.0172	9.0407	9.9536 20 25];
left_fig_60_xc_0 = ...
[0.88958	0.82626	0.78399	0.76939	0.74254	0.73696	0.72788	0.72566	0.7271	0.72862	0.73661 0.73661 0.73661];
A_60_xc_0 = interlim1(A_fig_60_xc_0,left_fig_60_xc_0,AR_w);

Matrixa_xc_0 = [0, 40, 50,60]*pi/180;%converted to rad. - Murthy
Matrixb_xc_0 = [A_0_xc_0,A_40_xc_0,A_50_xc_0,A_60_xc_0];
A_xc_0 = interlim1(Matrixa_xc_0,Matrixb_xc_0,Lambda_c4);
%%Step 1 interpolation for Xw_over_c_w=0 completed

%%Retrieving values final values for Xw_over_c_w=0, figure 9.4
left_xc_0 = [0.078867	0.2211	0.45703	0.67422	0.86141	0.98499];
Cn_r_over_CL_squared_xc_0 = [-0.095716	-0.062269	-0.013754	0.031748	0.072738	0.098519];
%%%%%%%%%%%%%%%%%%
final_xc_0 = interlim1(left_xc_0,Cn_r_over_CL_squared_xc_0,A_xc_0);

%%Interpolating in Figure 9.4 for Xw/c_w=.2

A_fig_0_xc_2 = ...
[1.0153	2.0208	3.0266	4.0177	4.9939	6.0336	6.9946	8.0026	9.0438	9.958 20 25];
left_fig_0_xc_2 = ...
[0.15809	0.22012	0.27816	0.32011	0.34596	0.3682	0.38597	0.40802	0.40626	0.41575 0.41575 0.41575];
A_0_xc_2 = interlim1(A_fig_0_xc_2,left_fig_0_xc_2,AR_w);

A_fig_40_xc_2 = ...
[1.0339	2.0272	3.0358	3.9963	4.9741	6.0143	7.0076	8.0166	9.026	9.988 20 25];
left_fig_40_xc_2 = ...
[0.3622	0.36816	0.38221	0.40798	0.40984	0.42409	0.43005	0.4361	0.43816	0.43993 0.43993 0.43993];
A_40_xc_2 = interlim1(A_fig_40_xc_2,left_fig_40_xc_2,AR_w);

A_fig_50_xc_2 = ...
[1.0618	1.398	1.7636	2.2707	3.029	3.9915	5.0019	6.043	7.0055	8.0151	9.0405	9.971 20 25];
left_fig_50_xc_2 = ...
[0.67037	0.59239	0.54658	0.50962	0.49017	0.48395	0.47001	0.46826	0.46203	0.46009	0.45824   0.45983 0.45983 0.45983];
A_50_xc_2 = interlim1(A_fig_50_xc_2,left_fig_50_xc_2,AR_w);

A_fig_60_xc_2 = ...
[2.0344	2.4507	2.8952	3.402	4.0187	4.9975	6.0076	7.0014	8.011	9.0198	9.9663 20 25];
left_fig_60_xc_2 = ...
[1.0042	0.9067	0.86137	0.82841	0.80411	0.78998	0.78005	0.77801	0.77607	0.78612	0.7838 0.7838 0.7838];
A_60_xc_2 = interlim1(A_fig_60_xc_2,left_fig_60_xc_2,AR_w);

Matrixa_xc_2 = [0, 40, 50,60]*pi/180;%converted to rad. - Murthy
Matrixb_xc_2 = [A_0_xc_2,A_40_xc_2,A_50_xc_2,A_60_xc_2];
A_xc_2 = interlim1(Matrixa_xc_2,Matrixb_xc_2,Lambda_c4);
%%Step 1 interpolation for Xw/c-bar =.2 completed


%%Retrieving values final values for Xw_over_c_w=.2, figure 9.4
left_xc_2 = [0.02777	0.17861	0.3532	0.56352	0.70241	0.86906	0.98415];
Cn_r_over_CL_squared_xc_2 = [-0.094063	-0.065976	-0.030401	0.011062	0.039222	0.073319	0.095517];
%%%%%%%%%%%%%%%%%%
final_xc_2 = interlim1(left_xc_2,Cn_r_over_CL_squared_xc_2,A_xc_2);


%%Interpolating in Figure 9.4 for Xw/c_w=.4
A_fig_0_xc_4 = ...
[1.0312	2.0303	3.0298	4.0006	5.0194	6.0084	6.9977	8.0504	9.057	10.017 20 25];
left_fig_0_xc_4 = ...
[0.083284	0.17426	0.26132	0.31294	0.35303	0.37337	0.38979	0.39867	0.39556	0.38831 0.38831 0.38831];
A_0_xc_4 = interlim1(A_fig_0_xc_4,left_fig_0_xc_4,AR_w);

A_fig_40_xc_4 = ...
[1.0592	1.1921	1.434	1.7212	2.0226	2.5444	3.0322	4.007	4.9978	6.0192	7.0248	8.0452	9.0341	10.023 20 25];
left_fig_40_xc_4 = ...
[0.72655	0.63698	0.55969	0.50614	0.47226	0.4355	0.42996	0.43062	0.42744	0.43617	0.44482	0.46531	0.48565	0.50991 0.50991 0.50991];
A_40_xc_4 = interlim1(A_fig_40_xc_4,left_fig_40_xc_4,AR_w);

A_fig_50_xc_4 = ...
[1.6358	1.6744	1.7582	1.8895	2.1776	2.5897	3.0486	4.0261	5.0173	6.0078	6.9986	8.0205	9.0739	10.017 20 25];
left_fig_50_xc_4 = ...
[0.99201	0.902	0.83572	0.76575	0.70044	0.65925	0.62219	0.58757	0.58047	0.58121	0.57803	0.57892	0.57996	0.58047 0.58047 0.58047];
A_50_xc_4 = interlim1(A_fig_50_xc_4,left_fig_50_xc_4,AR_w);

A_fig_60_xc_4 = ...
[3.3811	4.0308	5.0111	6.0358	7.0285	8.0184	9.0403	10.015 20 25];
left_fig_60_xc_4 = ...
[0.99238	0.92877	0.85888	0.82448	0.79778	0.80636	0.80725	0.81183 0.81183 0.81183];
A_60_xc_4 = interlim1(A_fig_60_xc_4,left_fig_60_xc_4,AR_w);

Matrixa_xc_4 = [0, 40, 50,60]*pi/180;%converted to rad. - Murthy
Matrixb_xc_4 = [A_0_xc_4,A_40_xc_4,A_50_xc_4,A_60_xc_4];
A_xc_4 = interlim1(Matrixa_xc_4,Matrixb_xc_4,Lambda_c4);
%%Step 1 interpolation for Xw/c-bar = .4 completed

%%Retrieving values final values for Xw_over_c_w=.4, figure 9.4
left_xc_4 = [0.023695	0.24808	0.49227	0.74405	0.99213];
Cn_r_over_CL_squared_xc_4 = [-0.094019	-0.049595	0.00084713	0.048117	0.098523];
%%%%%%%%%%%%%%%%%%
final_xc_4 = interlim1(left_xc_4,Cn_r_over_CL_squared_xc_4,A_xc_4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%The final interpolation is between all three numbers
Matrix1 = [0,.2,.4];
Matrix2 = [final_xc_0,final_xc_2,final_xc_4];
Cn_r_over_CL_squared = interlim1(Matrix1,Matrix2,Xw_over_c_w);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%THIS COMPLETES ALL INTERPOLATION IN FIGURE 9.4





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%THIS BEGINS THE INTERPOLATION IN FIGURE 9.5 TO FIND Cn_r_over_C_bar_D_o
%%The figure 9.5 was not taken out far enough to have higher Aspect Ratios,but this is a good approximation:
if AR_w > 25
   AR_w = 25
end

A_fig_0_xc_0 = ...
[1.0636	1.2698	1.5738	1.9983	3.0433	4.0064	5.0293	5.9921	7.0338	7.9963 20 25];
left_fig_0_xc_0 = ...
[-0.57721	-0.49065	-0.44001	-0.38574	-0.34111	-0.32398	-0.31097	-0.2978	-0.30465	-0.29544 -0.29544 -0.29544]; 
  %%Figure 9.5, Xw/c_w=0, Lambda_c4=0
A_0_xc_0 = interlim1(A_fig_0_xc_0,left_fig_0_xc_0,AR_w);

A_fig_40_xc_0 = ...
[1.0631	1.3676	1.6708	2.0125	3.0357	4.0177	4.9995	5.9816	6.9828	7.9251 20 25];
left_fig_40_xc_0 = ...
[-0.58513	-0.52656	-0.4878	-0.47291	-0.45594	-0.4547	-0.45742	-0.45619	-0.46688	-0.46157 -0.46157 -0.46157];
A_40_xc_0 = interlim1(A_fig_40_xc_0,left_fig_40_xc_0,AR_w);

A_fig_50_xc_0 = ...
[1.0557	1.3389	1.7014	2.0024	3.0435	4.0056	5.0269	6.0287	7.0305	7.9525 20 25];
left_fig_50_xc_0 = ...
[-0.696	-0.65718	-0.63046	-0.62337	-0.63814	-0.63685	-0.6476	-0.65038	-0.65315	-0.65175 -0.65175 -0.65175];
A_50_xc_0 = interlim1(A_fig_50_xc_0,left_fig_50_xc_0,AR_w);


A_fig_60_xc_0 = ...
[1.2961	2.0377	2.7392	3.0785	3.9991	5.04	6.041	7.0033	8.0257 20 25];
left_fig_60_xc_0 = ...
[-0.99765	-0.99575	-0.99373	-1.0145	-1.0329	-1.0516	-1.0663	-1.061	-1.0559 -1.0559 -1.0559];
A_60_xc_0 = interlim1(A_fig_60_xc_0,left_fig_60_xc_0,AR_w);

Matrixa_xc_0 = [0, 40, 50,60]*pi/180;%converted to rad. - Murthy
Matrixb_xc_0 = [A_0_xc_0,A_40_xc_0,A_50_xc_0,A_60_xc_0];
A_xc_0 = interlim1(Matrixa_xc_0,Matrixb_xc_0,Lambda_c4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%End of interpolation where Xw/c_w=0


%%Start interpolating in Figure 9.5 for Xw/c_w=.2

A_fig_0_xc_2 = ...
[1.0379	1.1577	1.3573	1.5569	1.976	2.994	3.9521	4.9701	5.9481	6.9461	7.984 20 25];
left_fig_0_xc_2 = ...
[-0.72408	-0.64463	-0.55341	-0.50206	-0.44326	-0.36995	-0.34434	-0.33481	-0.31722	-0.32359	-0.31013 -0.31013 -0.31013];
A_0_xc_2 = interlim1(A_fig_0_xc_2,left_fig_0_xc_2,AR_w);

A_fig_40_xc_2 = ...
[1.0379	1.2774	1.5569	1.9162	2.4551	3.014	3.9721	4.99	5.988	6.9661	7.9042 20 25];
left_fig_40_xc_2 = ...
[-0.76394	-0.63296	-0.55788	-0.50292	-0.48029	-0.46967	-0.46	-0.47439	-0.46482	-0.47513	-0.48934 -0.48934 -0.48934];
A_40_xc_2 = interlim1(A_fig_40_xc_2,left_fig_40_xc_2,AR_w);

A_fig_50_xc_2 = ...
[1.018	1.2974	1.5569	1.8363	2.3154	3.014	4.012	5.01	5.988	6.9661	7.9242 20 25];
left_fig_50_xc_2 = ...
[-0.8237	-0.73267	-0.67748	-0.64227	-0.61949	-0.61718	-0.62355	-0.64188	-0.64821	-0.65454	-0.66082 -0.66082 -0.66082];
A_50_xc_2 = interlim1(A_fig_50_xc_2,left_fig_50_xc_2,AR_w);

A_fig_60_xc_2 = ...
[1.0579	1.3972	1.6966	1.996	3.0539	3.992	5.01	6.0279	6.986	7.984 20 25];
left_fig_60_xc_2 = ...
[-1.075	-1.016	-0.9808	-0.96956	-0.99202	-1.0062	-1.0206	-1.035	-1.0413	-1.0517 -1.0517 -1.0517];
A_60_xc_2 = interlim1(A_fig_60_xc_2,left_fig_60_xc_2,AR_w);

Matrixa_xc_2 = [0, 40, 50,60]*pi/180;%converted to rad. - Murthy
Matrixb_xc_2 = [A_0_xc_2,A_40_xc_2,A_50_xc_2,A_60_xc_2];
A_xc_2 = interlim1(Matrixa_xc_2,Matrixb_xc_2,Lambda_c4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%End of interpolation where Xw/c_w=2


%%Start interpolating in Figure 9.5 for Xw/c_w=.4
A_fig_0_xc_4 = ...
[1.0065	1.2307	1.6963	2.0185	3.0207	4.0198	5.0184	6.0172	6.9954	7.9935 20 25];
left_fig_0_xc_4 = ...
[-0.94332	-0.80461	-0.61064	-0.52374	-0.39818	-0.36401	-0.34572	-0.32347	-0.31708	-0.31469 -0.31469 -0.31469];
A_0_xc_4 = interlim1(A_fig_0_xc_4,left_fig_0_xc_4,AR_w);

A_fig_40_xc_4 = ...
[1.0463	1.3112	1.5343	1.6966	1.8779	2.0182	3.0379	3.9966	4.9949	6.0329	6.9709	7.9688 20 25];
left_fig_40_xc_4 = ...
[-0.94736	-0.78487	-0.67794	-0.59872	-0.55133	-0.53168	-0.48165	-0.46331	-0.45297	-0.45065	-0.45611	-0.4577 -0.4577 -0.4577];
A_40_xc_4 = interlim1(A_fig_40_xc_4,left_fig_40_xc_4,AR_w);

A_fig_50_xc_4 = ...
[1.064	1.309	1.4916	1.7331	1.9141	2.4945	3.0338	4.0116	5.0091	6.0069	7.0048	7.9828 20 25];
left_fig_50_xc_4 = ...
[-1.0149	-0.85241	-0.76131	-0.70209	-0.66264	-0.61589	-0.60482	-0.61035	-0.62783	-0.63339	-0.63497	-0.63653 -0.63653 -0.63653];
A_50_xc_4 = interlim1(A_fig_50_xc_4,left_fig_50_xc_4,AR_w);

A_fig_60_xc_4 = ...
[1.0581	1.3016	1.5234	1.7243	1.9243	2.4433	3.0812	4.0189	5.0169	6.0143	6.972	8.0095 20 25];
left_fig_60_xc_4 = ...
[-1.1937	-1.0749	-1.0077	-0.9683	-0.9567	-0.95355	-0.97841	-0.99182	-0.9934	-1.0109	-1.0204	-1.0339 -1.0339 -1.0339];
A_60_xc_4 = interlim1(A_fig_60_xc_4,left_fig_60_xc_4,AR_w);

Matrixa_xc_4 = [0, 40, 50,60]*pi/180;%converted to rad. - Murthy
Matrixb_xc_4 = [A_0_xc_4,A_40_xc_4,A_50_xc_4,A_60_xc_4];
A_xc_4 = interlim1(Matrixa_xc_4,Matrixb_xc_4,Lambda_c4);
%%End of interpolation for Xw/c-bar=.4

%%Interpolation between Xw/c_w figures
Matrix2a = [0, .2, .4];
Matrix2b = [A_xc_0, A_xc_2, A_xc_4];

%%Final Result for Figure 9.5
Cn_r_over_C_bar_D_o = interlim1(Matrix2a, Matrix2b,Xw_over_c_w);



%%THIS IS THE TOTAL MOMENT CONTRIBUTION FROM THE WING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Cn_r_w = Cn_r_over_CL_squared*CL^2+Cn_r_over_C_bar_D_o*C_bar_D_o;  %%Eqn. 9.10


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

CL_a_v = (2*pi*A_v_eff)/(2+sqrt(A_v_eff^2*beta^2*(1+(tan(Lambda_c4))^2/beta^2) + 4)); 
  %%eqn 3.8 with A_v_eff subbed in for A (see pg 7.2)



C_y_beta_v = -k*CL_a_v*term1*eta_v*S_v/S_w;    %%eqn 7.4



%%This is the total contribution by the single vertical tail
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Cn_r_v = 2/b_w^2*(l_v*cos(alpha)+Z_v*sin(alpha))^2*C_y_beta_v;  
  %%eqn 9.11


%%The following sums up the contributions of the wing and vertical tail
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Cn_r = Cn_r_w + Cn_r_v;

return


