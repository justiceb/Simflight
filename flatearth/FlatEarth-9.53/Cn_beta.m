%****************************************************************
% Function File for C_n_beta
%****************************************************************
%
% Version 8.0 1/23/03 
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2002
%
% Ref.(2)
%
% Inputs:           S_w = Surface area of the wing (ft^2)
%                               b_w = Wing Span (ft)
%                               alpha = angle of attack (deg)
%                               l_v = the horizontal distance from the airplane reference point to the aerodynamic centre of the vertical tail (ft)
%                                Z_v = the vertical distance from the airplane reference point to the aerodynamic centre of the vertical tail (ft)
%                               l_f = the horizontal length of the fuselage (ft)
%                               S_b_s = body side area (ft^2)
%                               x_m = centre of gravity location from the leading edge (ft)
%                               h1_fuse = body height of the fuselage at 1/4 of the length (ft)
%                               h2_fuse = body height of the fuselage at 3/4 of the length (ft)
%                               hmax_fuse = maximum body height of the fuselage (ft)
%                               wmax_fuse = maximum body width (ft)
%               two_r_one = fuselage depth in region of vertical tail (ft)
%               eta_v = ratio of the dynamic pressure of the vertical tail to that of the freestream
%               M = mach number
%               AR_v = aspect ratio of the vertical tail
%               b_v = vertical tail span measured from the fuselage centreline (ft)
%               Z_h = the negative of the horizontal distance from the fuselage centreline to the tail aero centre (ft)
%               x_over_c_v = horizontal distance from the fuselage centreline to the horizontal aero centre (ft)
%               lambda_v = vertical tail taper ratio based on surface measured from fuselage centreline
%               S_v = surface area of the vertical tail (ft^2)
%               Z_w = the vertical tail distance from the wing root c/4 to the fuselage centreline, positive downward (ft)
%               d = fuselage diameter (ft)
%               Z_w1 = distance from body centerline to c/4 of wing root chord, positive for c/4 point below body centerline (ft) %%fig 7.1 
%               S_h     = area of the horizontal tail (ft^2)
%               Lambda_c4 = the quater chord sweep angle (rad)
%               Rl_f = fuselage Reynold's number


%Notations:         Cn_beta = Variation of yawing moment coefficient with side slip angle
%                               C_n_beta_b = The body contribution to Cn_beta (rad^-1)
%                               C_n_beta_v = The vertical tail contribution to Cn_beta (rad^-1)
%                               C_n_beta_w = The wing contribution to Cn_beta
%                               K_N = is an empirical factor for body and nody and wing effects
%                               K_R_l = is a Reynold's number for the fuselage

function X=Cn_beta(Cl_alpha_w,S_w,b_w,alpha,l_v,Z_v,l_f,S_b_s,Rl_f,x_m,h1_fuse,h2_fuse,hmax_fuse,wmax_fuse,two_r_one,eta_v,M,AR_v,b_v,Z_h,x_over_c_v,lambda_v,S_v,Z_w,d,Z_w1,S_h,Lambda_c4,Lambda_c2_v)

% Some derived constants

beta=sqrt(1-M^2);
lB_sB=l_f^2/S_b_s;
xM_lB=x_m/l_f;
h1_h2=sqrt(h1_fuse/h2_fuse);
h_w=hmax_fuse/wmax_fuse;
Zh_bv=-abs(Z_h/b_v);
bv_2r1=b_v/two_r_one;
Sh_Sv=S_h/S_v;

% From equation 7.16 on page 7.5 the body contribution is calculated, including the interference effect of the wing on the body
% Using Figure 7.20 on page 7.20 the Reynold's factor for the fuselage can be obtained - KR_l
KR_l_fig=...
[1.009 1.336 1.595 1.695 1.896 2.051 2.184];
Rl_f_fig=...
[0.989 5.142 18.135 29.769 81.098 178.120 359.020]*10^6;

KR_l=interlim1(Rl_f_fig, KR_l_fig, Rl_f);


% Using figure 7.19 on page 7.19, K_N - the empirical factor for body and body+wing contribution can be obtained

l_fig=[20 14    10      8       7       6       5       4       3       2.5];
x_fig=[0.1 0.8];
ref1_fig=[-0.30013      0.047295        0.39479 0.64734 0.94739 1.2947  1.6735  2.2104  2.6526  3.1104;...
1.8945  2.2576  2.605   2.8419  3.1577  3.5367  3.8997  4.4208  5.0366  5.4314];
ref2=interlim2(l_fig,x_fig,ref1_fig,lB_sB,xM_lB);

ref2_fig=[0 6];
h_fig=[1.6      1.4     1.2     1       0.8];
ref3_fig=[0     0       0       0       0;...
9.4357  8.1798  7.1509  5.9529  4.5389];
ref3=interlim2(h_fig,ref2_fig,ref3_fig,h1_h2,ref2);

%ref3=4;
%h_w=0.8;
%hard coding removed by Murthy
ref4_fig=[0 6];
hw_fig=[0.5     0.6     0.8     1       2];
K_fig=[-0.00049007      -0.00049007     -0.00049007     -0.00049007     -0.00049007;...
      0.0024742 0.0034608       0.0047293       0.0055435       0.0064205];
K_N=interlim2(hw_fig,ref4_fig,K_fig,h_w,ref3);

% To calculate the vertical tail contribution:

% Interpolating in Figure 7.6
Z_h_over_b_v_fig8 = ...
[-0.014442      -0.075377       -0.17057        -0.30375        -0.38361        -0.43113        -0.48242        -0.52417        -0.56402        -0.6171 -0.67203        -0.71932        -0.74955        -0.7911 -0.82694        -0.87399        -0.9322 -0.97536];
A_v_hb_over_A_v_b_fig8 = ...
[1.2741 1.2141  1.1242  1.0272  0.97877 0.95656 0.94198 0.93866 0.93912 0.95489 0.98205 1.0243  1.0588  1.1161  1.1734  1.2839  1.4552  1.5921]; %%x/c_v=.8, Fig 7.6
%data1=[Z_h_over_b_v_fig8',A_v_hb_over_A_v_b_fig8']
A_v_hb_over_A_v_b8 = interlim1(Z_h_over_b_v_fig8, A_v_hb_over_A_v_b_fig8,Zh_bv);
   
Z_h_over_b_v_fig7 = ...
[-0.010859      -0.13075        -0.24678        -0.34946        -0.46538        -0.56029        -0.64558        -0.73449        -0.7874 -0.859  -0.89846        -0.9585 -0.97914];
A_v_hb_over_A_v_b_fig7 = ...
[1.2145 1.117   1.0386  0.97908 0.93121 0.92121 0.94924 1.027   1.0889  1.2313  1.3388  1.5306  1.5959];   %%x/c_v=.7, Fig 7.6
A_v_hb_over_A_v_b7 = interlim1(Z_h_over_b_v_fig7, A_v_hb_over_A_v_b_fig7,Zh_bv);

Z_h_over_b_v_fig6 = ...
[-0.0092336     -0.10051        -0.23366        -0.3439 -0.41987        -0.51294        -0.60968        -0.68163        -0.76667        -0.85531        -0.9117 -0.97913];
A_v_hb_over_A_v_b_fig6 = ...
[1.1381 1.0822  0.99259 0.94081 0.92288 0.89757 0.90669 0.94979 1.0466  1.2007  1.3504  1.5997];   %%x/c_v=.6, Fig 7.6   
A_v_hb_over_A_v_b6 = interlim1(Z_h_over_b_v_fig6, A_v_hb_over_A_v_b_fig6,Zh_bv);

Z_h_over_b_v_fig5 = ...
[-0.01143       -0.16927        -0.27951        -0.40107        -0.532  -0.6552 -0.76115        -0.84601        -0.93625        -0.98294];
A_v_hb_over_A_v_b_fig5 = ...
[1.0541 0.95724 0.90546 0.87295 0.87113 0.9112  1.0006  1.1471  1.3852  1.5959];  %%x/c_v=.5, Fig 7.6
A_v_hb_over_A_v_b5 = interlim1(Z_h_over_b_v_fig5, A_v_hb_over_A_v_b_fig5,Zh_bv);

Matrixa = [.5, .6, .7, .8];
Matrixb = [A_v_hb_over_A_v_b5, A_v_hb_over_A_v_b6, A_v_hb_over_A_v_b7, A_v_hb_over_A_v_b8];
A_v_hb_over_A_v = interlim1(Matrixa, Matrixb, x_over_c_v);
%%Done interp fig 7.6


b_v_over_2r1 = b_v/two_r_one;

%%Interpolating in Figure 7.5
%%The referenced figure assumes all values of lambda_v less than .6 are estimated at .6
if lambda_v < .6;
   lambda_v = .6;
end;

b_v_over_2r1_fig1 = ...
[0.0089225      0.13036 0.2147  0.35595 0.60146 0.79048 1.0743  1.3015  1.5668  1.8701  2.0883  2.2782  2.5345  2.753   3.0286  3.5224  4.0159  4.5378  5.0216  5.5054  5.9986  6.4918  6.9377];
A_v_b_over_A_v_fig1 = ...
[0.10677        0.4613  0.6557  0.85004 1.0595  1.189   1.3374  1.421   1.4741  1.5081  1.5041  1.4696  1.4235  1.3508  1.2666  1.1555  1.0901  1.0629  1.0433  1.0351  1.0346  1.0378  1.0221];   %%lambda_v=.1, figure 7.5
b_v_over_2r1_1 = interlim1(b_v_over_2r1_fig1,A_v_b_over_A_v_fig1,bv_2r1);

b_v_over_2r1_fig6 = ...
[0.0079378      0.091375        0.1853  0.27945 0.42102 0.55323 0.7139  1.0262  1.3861  1.6513  1.8789  2.0402  2.2395  2.5245  2.7525  3.0281  3.5316  4.0252  4.5281  5.012   5.5148  6.0081  6.4824  6.9567];
A_v_b_over_A_v_fig6 = ...
[0.29364        0.65965 0.83497 0.96834 1.1017  1.2121  1.3225  1.4633  1.5659  1.6228  1.634   1.63    1.5993  1.5265  1.4462  1.3467  1.2051  1.1359  1.101   1.0738  1.0542  1.0307  1.0264  1.0259];   %%lambda_v<=.6
b_v_over_2r1_6 = interlim1(b_v_over_2r1_fig6,A_v_b_over_A_v_fig6,bv_2r1);

Matrixa = [.6, 1];
Matrixb = [b_v_over_2r1_6,b_v_over_2r1_1];
A_v_b_over_A_v = interlim1(Matrixa, Matrixb, lambda_v);
%%Done interp fig 7.5


%%Interpolating in Figure 7.7
S_h_over_S_v_fig = ...
[0.00                   0.059518        0.13083 0.19024 0.37989 0.51796 0.66361 0.85212 1.0836  1.3735  1.5652  1.7686  1.9995];
K_H_fig = ...
[0.0039038      0.09466 0.19724 0.28009 0.50117 0.63941 0.75409 0.85712 0.95639 1.0441  1.0763  1.1086  1.1449];  %%Fig 7.7
K_H = interlim1(S_h_over_S_v_fig, K_H_fig, Sh_Sv);
%%Done interp

%Using equation 7.6 on page 7.2
A_v_eff = A_v_b_over_A_v*AR_v*(1+K_H*(A_v_hb_over_A_v-1));   

%Using equation 7.5 on page 7.2 
term1 = .724 + 3.06*((S_v/S_w)/(1+cos(Lambda_c4)))+.4*Z_w/d+0.009*AR_v;  
%%Interpolating in Figure 7.3
b_v_over_2r1_fig = [0.04978     0.48968 0.9977  1.506   1.985   2.491   2.9873  3.484   4.0019  4.5002  4.9986  5.5166  5.9855];
k_fig = [0.76234        0.75567 0.76091 0.75441 0.74784 0.83516 0.91853 0.98628 0.98763 0.99283 0.99022 0.98767 0.9928];  %%Fig 7.3
k = interlim1(b_v_over_2r1_fig,k_fig,b_v_over_2r1);
%%Done interp fig 7.3


C_n_beta_b=-57.3*K_N*KR_l*S_b_s/S_w*l_f/b_w;

C_L_a_v = (2*pi*A_v_eff)/(2+sqrt((A_v_eff^2*beta^2/((Cl_alpha_w/(2*pi))^2))*(1+(tan(Lambda_c2_v))^2/beta^2) + 4));  %%eqn 3.8 with A_v_eff subbed in for A (see pg 7.2)

%From equation 7.4 on page 7.1
C_y_beta_v = -k*C_L_a_v*term1*S_v/S_w;     

%From equation 7.17 on page 7.5, the vertical tail contribution is calculated
C_n_beta_v=-C_y_beta_v*(((l_v*cos(alpha))+( Z_v*sin(alpha)))/b_w);

%C_n_beta_w the wing contribution is very small except at high angles of attack, therefore it is considered here to be zero.  Therefore equation 7.15 on page 7.5 is modified to determine C_n_beta, neglecting the wing contribution.
X=C_n_beta_b +C_n_beta_v;
