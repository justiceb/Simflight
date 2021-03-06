% *********************************************
% Make_Constant    Version 9.51 1/25/2010
% This version uses the new Cl_beta code from Barnes McCormick
%
% OBJECTIVES: 1. Determine a set of derived constants that are 
%                used by the stability and control derivative functions.
%             2. Runs each of the stability derivitives functions
%                and sets up the array called constant that is used by other
%                MATLAB programs to perform various dynamic analysis.
%
% INPUTS: Many basic constants defined in the Matlab workspace.
% OUTPUTS: The array called constant of length 67.
% IMPORTANT NOTE: This script is INDEPENDENT of the particular aircraft being modeled.
%
%************************************************************************
% Make_Constants.m    Version 9.5 4/26/09
% - added function call [temp,press,rho,Hgeopvector]=atmosphere4(altitude,1) 
%   to determine density and temperature at altitude
% - Added constants R=1716.55 ft^2/(sec^2degR) and gamma=1.4 (air)S
% - Calculate speed of sound based on altitude given by sqrt(R*gamma*temp)
% - Corrected Vh to use c_w instead of c_h
% - Moved CL, Cd at trim calculations to Make_Constants from Basic_Constants
% - Constant(28) added kappa_h lambda_c2_h inputs
%
%************************************************************************
%
% This version corrects Cl_beta, Cy_beta, and Cn_beta.
% This version corrects Cm_alpha, rudder control power, units, and comments.
% AAE565 Spring 2003
%
% *********************************************     
%
% A&AE 565 Spring 2003 - Purdue University
% 
% Note: This code is provided for a first order approximation of the dynamic 
%       stability and control derivatives of an airplane.
%
% Equations/Figures can be found in :
% 
% (Ref.1) Roskam, Jan. "Airplane Flight Dynamics and Automatic Flight
%         Controls"
%         Published by DARcorporation
%         120 E. Ninth St., Suite 2
%         Lawarence, KS 66044
%         Third Printing, 2001.
%
% (Ref.2) Roskam, Jan. "Methods for Estimating Satbility and
%         Control Derivatives of Conventional Subsonic Airplanes"
%         Published by the Author 
%         519 Boulder 
%         Laurance, Kansas 66044
%         Third Printing, 1997.
%  
% (Ref.3) Roskam, Jan. "Airplane Design: Part VI: Preliminary Calculation
%         of Aerodynamic, Thrust and Power Characteristics"
%         Published by Roskam Aviation and Engineering Corporation
%         Rt4, Box 274
%         Ottawa, Kansas 66067
%         Second Printing, 1990.
%
% Ref: Barnes W. McCormick, Aerodynamics, Aeronautics, and Flight mechanics, Second Edition
%      John Wiley and Sons, Inc. 1995, pages 529-534 (used fror Cl_beta_Mc)

disp(' '); disp(' Starting Make_Constant: The array called constant is now being defined.'); disp(' ')
% The derived constants below are defined from the basicConstants defined
% in the BasicConstants script.
U1 = U*1.687808;   	    		% Trim flight speed [ft/s]
[temp,press,rho,Hgeopvector]=atmosphere4(altitude,1);
R=1716.55;	%ft^2/(sec^2degR)
gamma=1.4;  %Air
speed_sound=sqrt(gamma*R*temp);
M=U1/speed_sound;

AR_v = b_v^2/S_v;               % Aspect Ratio of Vertical Tail
AR_h = b_h^2/S_h;				% Aspect Ratio of Horizontal Tail
AR_w = b_w^2/S_w;               % Aspect Ratio of Horizontal Tail
B=sqrt(1-M^2*(cos(Lambda_c4))^2); % Compressibility correction factor
beta = sqrt(1-M^2);             % Compressibility correction factor
Beta = beta;                    % Compressibility correction factor
ca_cw = c_a/c_w;                % Ratio of aileron chord to wing chord
ce_ch = c_e/c_h;				% Elevator flap chord 
cr_cv			= c_r/c_v;      % Rudder flap chord 
eta_ie = b_h_ie/(b_h/2);	  	% Percent span position of inboard edge of elevator
eta_ir		= b_v_ir/b_v;       % Percent span position of inboard edge of rudder
eta_oe = b_h_oe/(b_h/2);		% Percent span position of outboard edge of elevator
eta_or		= b_v_or/b_v;       % Percent span position of outboard edge of rudder
kappa=Cl_alpha/(2*pi);          % Ratio of 2D lift coefficient to 2pi for the wing
kappa_h = Cl_alpha_h/(2*pi);	% Ratio of 2D lift coefficient to 2pi for the horiz. tail
kappa_v		= Cl_alpha_v/(2*pi);  % Ratio of 2D lift coefficient to 2pi for the vert. tail
Xh=Xach-Xref;         			% Distance from airplane arbitrary reference point to the horizontal tail ac [ft]
V_h = (Xh*S_h)/(c_w*S_w);       % Horizontal Tail Volume Coefficient.
Xw=Xacw-Xref;					% Distance from arbitrary ref point to aero center of wing alone. [ft]
l_h=Xach-.25*c_w;				% Distance from AC of Horizontal tail and wing c/4 (ft), positive aft.
delta_f = delf;         		% Streamwise flap deflection [rad] - same as delf
Gamma = dihedral;				% This is the geometric dihedral angle, positive for 
%								dihedral, negative for anhedral [rad]
lambda_w = lambda;     			% Taper ratio of wing again
S_b = S_b_s;           			% Body side area [ft^2] again
x_AC_vh=x_over_c_v*c_v;     	% X distance from LE of vertical tail to AC of horizontal tail [ft]

disp(['Trim airspeed= ',num2str(U1),' ft/sec'])
nu=nufun(altitude);             % Kinematic viscosity, ft*ft/sec

Rl_f= U1*l_f/nu;    			% Reynolds number of fuselage [non-dimensional]
l_b = l_f;	           			% Total length of fuselage [ft]
qbar=.5*rho*U1*U1;     %lbf/ft^2
CL=W/(qbar*S_w);
Cd = Cd_0 + (CL^2/(pi*AR_w*e));   % Drag Coefficient

% End of the derived constants section.

echo on
% The 57 "constant" computed below are used by four dynamic and control
% software programs. Specifically, Simulink scripts FlatEarth.mdl and E_Earth.mdl
% use the first 57 "constant". These MATLAB programs perform 6-degree-of-freedom 
% aircraft simulation over a flat earth or an elliptical earth.
% Two other programs (LongSC.m and LatSC.m) do simplified dynamic modeling (compute transfer functions)
% for the longitudinal or lateral-directional degrees of freedom.  They addtionally use all
% constant(58)-constant(67) of the "constant" defined below. 
%
echo off
% Mass related inputs
constant(1)=W;      	% W, Weight, pounds (lbf)
constant(2)= 1.4076431e16/(2.092565e7+altitude)^2; 	% g, Acceleration of gravity, ft/(sec*sec)
constant(3)=constant(1)/constant(2); % mass, slugs
constant(4)=Ixx;    	% Ixx, slug*ft*ft
constant(5)=Iyy;   		% Iyy, slug*ft*ft
constant(6)=Izz;   		% Izz, slug*ft*ft
constant(7)=Ixz;    	% Ixz, slug*ft*ft
constant(8)=eta_p;  	% propeller efficiency, eta, nondimensional
constant(9)=0; 			% unassigned

% Derived constants from the inertia data
constant(10)=constant(4)*constant(6)-constant(7)*constant(7);  %gamma
constant(11)=((constant(5)-constant(6))*constant(6)-constant(7)*constant(7))/constant(10);% c1
constant(12)=(constant(4)-constant(5)+constant(6))*constant(7)/constant(10);% c2
constant(13)= constant(6)/constant(10);% c3 
constant(14)= constant(7)/constant(10);% c4 
constant(15)=(constant(6)-constant(4))/constant(5);% c5 
constant(16)= constant(7)/constant(5);% c6
constant(17)= 1/constant(5);% c7
constant(18)=  (constant(4)*(constant(4)-constant(5))+constant(7)*constant(7))/constant(10);% c8 
constant(19)=  constant(4)/constant(10);% c9 

% aircraft geometry
constant(20)=S_w;    % S, wing area, ft^2
constant(21)=c_w;    % cbar, mean geometric chord, ft
constant(22)=b_w;    % b, wing span, ft
constant(23)=0;      % phiT, thrust inclination angle, RADIANS
constant(24)=0;      % dT, thrust offset distance, ft

% Nondimensional Aerodynamic stability and control derivatives

% Drag Polar CD=k(CLstatic-CLdm)^2 + CDm
constant(25)=Cd_0;   	% CDm, CD for minimum drag
constant(26)=k; 		% k
constant(27)=0;      	% CLdm, CL at the minimum drag point

disp(' '); disp(' ');
disp('Below this point in Make_Constant there may be multiple occurrances of warnings like the following')
disp('    Warning: 0.25 exceeded interpolation limits')
disp('    Warning: 0.25 was converted to 0.5 (plus some traceback information)')
disp('Each warning means that a variable was found to be out of limits')
disp('for using an empirical data found in a figure. No extrapolation was performed.')
disp('Instead, the variable was limited to the value shown and the program proceded.')
disp('The careful user will check the that figure in the appropriate reference for each warning to see ')
disp('if the action of the computer was reasonable.')
disp(' ')

% Lift Force
constant(28)=CL_0(S_w,S_h,M,tc_w,alpha_0,epsilon_t,i_w,i_h,epsilon_0_h,AR_w,Lambda_c4,Lambda_c2,Lambda_c2_h,lambda_w,kappa,kappa_h,beta,b_w,d,AR_h,eta_h);  % CL0
constant(29)=CL_alpha(AR_w,AR_h,Lambda_c2,lambda_w,l_h,h_h,b_w,d,eta_h,S_h,S_w,kappa_h,Lambda_c2_h,beta,kappa);   % CLalpha
constant(30)=CL_de(S_w,S_h,AR_h,ce_ch,eta_oe,eta_ie,beta,kappa_h,lambda_h,Lambda_c2_h,tc_h,delta_e,Cl_alpha_h);   % CLdeltaE
constant(31)=CL_alpha_dot(l_h, h_h, b_w, lambda, AR_w, AR_h, Lambda_c4, Lambda_c4_h, beta, kappa, kappa_h, V_h, eta_h);    % CLalphadot
constant(32)=CL_q(Xw,b_w,c_w,c_h,AR_w,Lambda_c4,Lambda_c2,Lambda_c2_h,Xh,S_h,S_w,eta_h,AR_h,beta, V_h,b_h, kappa, kappa_h);    % CLQ
% Side Force
constant(33)=0;      % CY0
constant(34)=Cy_beta(Cl_alpha_w,two_r_one,eta_v,beta,AR_v,b_v,Z_h,x_over_c_v,lambda_v,S_v,S_w,Lambda_c4,Lambda_c4_v,Z_w,d,dihedral,wingloc,Z_w1,S_h,S_o); % CYbeta
constant(35)=Cy_da(S_w);      % CYdeltaA
constant(36)=Cy_dr(S_w,b_w,S_h,S_v,b_v,c_v,x_AC_vh,two_r_one,AR_v,l_v,Z_v,eta_or,eta_ir,cr_cv,beta,kappa_v,Lambda_c2_v,lambda_v,delta_r,alpha);  % CydeltaR
constant(37)=Cy_p(b_w,l_v,Z_v,alpha,two_r_one,eta_v,beta,AR_v,b_v,Z_h,x_over_c_v,lambda_v,S_v,S_w,Lambda_c4_v,Z_w,d,dihedral,wingloc,Z_w1,S_h,S_o); % Cy_p
constant(38)=Cy_r(alpha,two_r_one,eta_v,beta,AR_v,b_v,b_w,Z_h,x_over_c_v,lambda_v,S_v,S_w,Lambda_c4,Z_w,d,dihedral,wingloc,Z_w1,S_h,S_o,l_v,Z_v);  % CYR
% Pitching Moment
constant(45)=Cm_0(S_h_slip,S_w,S_h,M,tc_w,alpha_0,epsilon_t,i_w,i_h,epsilon_0_h,AR_w,Lambda_c4,lambda_w,beta,Cm_0_r,Cm_o_t,Lambda_c2_h,kappa_h,AR_h,Xh,c_w,eta_h);   % CM0
constant(46)=Cm_alpha(AR_w,AR_h,Lambda_c2,lambda_w,l_h,h_h,b_w,c_w,d,eta_h,S_h,S_w,kappa_h,Lambda_c2_h,beta,kappa,Xref,Xacwb); % Cmalpha
constant(47)=Cm_de(S_w,S_h,AR_h,ce_ch,eta_oe,eta_ie,beta,kappa_h,lambda_h,Lambda_c2_h,tc_h,delta_e,Cl_alpha_h,V_h); % CMdeltaE
constant(48)=Cm_a_dot(l_h,h_h,b_w,lambda_w,AR_w,Lambda_c4,M,Cl_alpha,eta_h,S_h,Xh,c_w,S_w,kappa,kappa_h,Lambda_c2,AR_h);  % CMalphadot
constant(49)=Cm_q(Xw,b_w,c_w,c_h,AR_w,Lambda_c4,Lambda_c2,Lambda_c2_h,Xh,S_h,S_w,eta_h,AR_h,beta,V_h,b_h,Cl_alpha,Cl_alpha_h,B);  % CMQ

% Rolling Moment
constant(39)=0;       % Cl0
% Code below won't work unless CL_wb is defined in BasicConstants file
% CL_hb = 0;          	    % Lift coefficient of the horzontal tail/body
% CL_wb= 0.3625;	     	    % Lift coefficient of the wing/body - assuming iw=0
% disp('Compute Cl_beta two ways, first using Roskam, then using McCormick')
% constant(40)=Cl_beta(CL_wb,Cl_alpha_w,theta,theta_h,Lambda_c2, Lambda_c4,Lambda_c4_h,S_w,b_w,AR_w,AR_h,Z_v,alpha,l_v,... 
% M,two_r_one,eta_v,beta,AR_v,b_v,Z_h,x_over_c_v,lambda_v,S_v,Z_w,d,wingloc,Z_w1,S_h,S_o,dihedral,dihedral_h,Lambda_c2_h,l_b,b_h,CL_hb,lambda_w,lambda_h);% Clbeta
% disp(' '); disp('Cl_beta using the original code from Roskam, per radian')
% constant(40)
% disp(' '); 
% disp('Now use McCormick''s method')
disp(' '); disp('Given below is Cl_beta PER DEGREE broken down into individual components using the new code from McCormick')
constant(40)=Cl_beta_Mc(low_wing,Cl_alpha_v,S_v,Z_v,S_w,b_w,AR_v,AR_w,Lambda_c2,lambda_w,dihedral,W,U1,rho); % Clbeta
disp(' '); disp('Cl_beta using the new code from McCormick, per radian')
constant(40)
%disp('The user should decide whick Cl_beta to use (default is McCormick''s method')
constant(41)=Cl_da(S_w,AR_w,ca_cw,eta_ia,eta_oa,beta,kappa,Lambda_c4,lambda_w,tc_w,Cl_alpha_w);   % CldeltaA
constant(42)=Cl_dr(S_w,b_w,S_h,S_v,b_v,c_v,x_AC_vh,two_r_one,AR_v,l_v,Z_v,eta_or,eta_ir,cr_cv,beta,kappa_v,Lambda_c2_v,lambda_v,delta_r,alpha);  % CldeltaR
constant(43)=Cl_p(b_h,b_w,Z_v,two_r_one,eta_v,AR_v,b_v,Z_h,x_over_c_v,lambda_v,S_v,S_w,Lambda_c4,Lambda_c4_h,Z_w,d,lambda_w,wingloc,Z_w1,S_h,S_o,beta,Cl_alpha,Cl_alpha_h,AR_w,lambda_h, AR_h);  % ClP
constant(44)=Cl_r(l_v,alpha,Z_v,S_h,S_v,x_over_c_v,b_v,Z_h,two_r_one,lambda_v,S_w,Z_w,d,AR_w,beta,Lambda_c4,c_w,Xw,AR_v,eta_v,b_w,cf,delta_f,theta,lambda_w,Gamma,kappa,B);  % ClR
% Yawing Moment
constant(50)=0;         % CN0
constant(51)=Cn_beta(Cl_alpha_w,S_w,b_w,alpha,l_v,Z_v,l_f,S_b_s,Rl_f,x_m,h1_fuse,h2_fuse,hmax_fuse,wmax_fuse,two_r_one,eta_v,M,AR_v,b_v,Z_h,x_over_c_v,lambda_v,S_v,Z_w,d,Z_w1,S_h,Lambda_c4,Lambda_c2_v);  % CNbeta
constant(52)=Cn_da(S_w,AR_w,ca_cw,eta_ia,eta_oa,beta,kappa,Lambda_c4,lambda_w,tc_w,Cl_alpha_w,CL); % CNdeltaA
constant(53)=Cn_dr(S_w,b_w,S_h,S_v,b_v,c_v,x_AC_vh,two_r_one,AR_v,l_v,Z_v,eta_or,eta_ir,cr_cv,beta,kappa_v,Lambda_c2_v,lambda_v,delta_r,alpha); % CNdeltaR
constant(54)=Cn_p(c_w,B,theta,adelf,delf,b_w,l_v,b_h,Z_v,two_r_one,eta_v,AR_v,b_v,Z_h,x_over_c_v,lambda_v,S_v,S_w,Lambda_c4,Lambda_c2,Lambda_c4_h,Z_w,d,lambda_w,wingloc,Z_w1,S_h,S_o,beta,Cl_alpha,Cl_alpha_h,AR_w,lambda_h, AR_h, b_f,Xw,CL,Lambda_c4_v,alpha); % CNP
constant(55)=Cn_r(CL,C_bar_D_o,l_v,alpha,Z_v,S_h,S_v,x_over_c_v,b_v,Z_h,two_r_one,lambda_v,S_w,Z_w,d,AR_w,beta,Lambda_c4,c_w,Xw,AR_v,eta_v,b_w); % CNR
% Reference positions
constant(56)=Xref/c_w;  % XbarRef, nondimensional, measured aft from leading edge of wing mean aerodynamic chord.
constant(57)=Xcg/c_w;  	% XbarCG,  nondimensional, measured aft from leading edge of wing mean aerodynamic chord.
% constant(57 can be changed at a later time if you use program FlatEarth or E_Earth.

% Trim conditions. These may or not be used by subsequent programs LongSC.m and LatSC.m. Small 
% variations in these trim flight conditions are OK.
constant(58)=U1;    	% Trim speed, Vt, ft/sec
constant(59)=altitude;  % Trim altitude, ft
constant(60)=0;    		% Trim alpha, >>>DEGREES<<<This is not used by LongSC
% 						The constants below are used only by LongSC.m and LatSC.m MATLAB scripts for liear small perturbation
% 						longitudinal and lateral-directional analysis as defined in Reference 1.
constant(61)=0;     	% CLu
constant(62)=0;     	% CDu
constant(63)=0; 		% CTxu
constant(64)=0;     	% Cmu
constant(65)=0;     	% CmTu
constant(66)=0;     	% CmTalpha
constant(67)=0;     	% CDdeltae

disp(' '); disp(' ');
disp('Above this point in Make_Constant there may be multiple occurrances of warnings like the following')
disp('    Warning: 0.25 exceeded interpolation limits')
disp('    Warning: 0.25 was converted to 0.5 (plus some traceback information)')
disp('Each warning means that a variable was found to be out of limits')
disp('for using an empirical data found in a figure. No extrapolation was performed.')
disp('Instead, the variable was limited to the value shown and the program proceded.')
disp('The careful user will check the that figure in the appropriate reference for each warning to see ')
disp('if the action of the computer was reasonable.')
disp(' '); disp('Done with Make_Constant: The array called constant is now defined.'); disp(' ')




