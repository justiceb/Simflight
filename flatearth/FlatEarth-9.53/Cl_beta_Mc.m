function [Cl_beta]=Cl_beta_Mc(low_wing,Cl_alpha_v,S_v,Z_v,S_w,b_w,AR_v,AR_w,Lambda_c2,lambda_w,dihedral,W,U1,rho)
% function [Cl_beta]=Cl_beta_Mc(low_wing,Cl_alpha_v,S_v,Z_v,S_w,b_w,AR_v,AR_w,Lambda_c2,lambda_w,dihedral,W,U1,rho)
%
% low_wing=1 if the wing is low
% low_wing=-1 if the wing is high   
% low_wing=0 if the wing is mid 
% low-wing can vary as a continuous number between -1 and 1.
% Cl_alpha_v 2-d lift curve slope of vertical tail, per radian
% S_v aera of vertical tail
% Z_v vertical distance from the roll axis to the aerodynamic center of the
%    vertical tail, positive up
% S_w reference area of the wing
% b_w wing span
%  AR_v aspect ratio of vertical tail
%  AR_w aspect ratio of the main wing
% Lambda_c2 sweep angle of the mid cord of the main wing (rad)
% lambda_w  Taper ratio of wing
% dihedral Geometric dihedral angle of the wing [rad], positive for 
%						   dihedral (wing tips up), negative for anhedral(tips down) [rad]
% W aircraft weight, lbf
% U1 trim speed in ft/sec
% rho air density (slug/ft^3)
%
% Ref: Barnes W. McCormick, Aerodynamics, Aeronautics, and Flight mechanics, Second Edition
%      John Wiley and Sons, Inc. 1995, pages 529-534

% Wing position contribution
Cl_beta_wingposition=low_wing*.00016 % per degree

% Vertical tail contribution
fiftyseven=180/pi;
Cl_alpha_v_3d=Cl_alpha_v*AR_v/(AR_v+(2*(AR_v+4)/(AR_v+2))); % McCormick eqn 3.70 pg 116
Cl_alpha_v_perdeg=Cl_alpha_v_3d/fiftyseven;
Cl_beta_vert_tail=-Cl_alpha_v_perdeg*S_v*Z_v/(S_w*b_w) % per degree

% Geometric dihedral angle contribution
% Tabular data is from Figure 9.46 on page 531.
ARmat =[0 0.45659 0.86202 1.2673 1.7135 2.2009 2.7596 3.4107 4.1332 4.9888 5.9772 6.9953 8.0121];
Kamat =[0 0.10064 0.20132 0.29996 0.39857 0.50022 0.59876 0.7003 0.80077 0.90218 1.0005 1.0834 1.1449];
[Ka] = interlim1(ARmat,Kamat,AR_w);

biglamda=Lambda_c2*fiftyseven; % sweep of mid cord in deg
% Tabular data is from Figure 9.46 on page 531.
biglamdamat =[0 10.51 20.255 30.402 40.538 46.876 50.246 54.731 60.221];
KBIGlamdamat =[1 0.98699 0.96883 0.94146 0.89982 0.8636 0.83987 0.79871 0.73197];
[KBIGlamda] = interlim1(biglamdamat,KBIGlamdamat,biglamda);
  
% Tabular data is from Figure 9.46 on page 531.
smalllamdamat =[0 0.025809 0.053583 0.077227 0.1019 0.12964 0.15737 0.1892 0.22513 0.26618 0.30106 0.35132 0.40158 0.49899 0.59843 0.69888 0.80137 0.89974 1.0002];
KSMALLlamdamat =[0.81122 0.83354 0.85482 0.87001 0.88621 0.90035 0.91348 0.92658 0.93966 0.95272 0.96274 0.97371 0.98366 0.99847 1.0092 1.0168 1.0214 1.024 1.0235];
[KSMALLlamda] = interlim1(smalllamdamat,KSMALLlamdamat,lambda_w);
dihedraldeg=dihedral*fiftyseven;
Clbetadihedral=-.00021*Ka*KBIGlamda*KSMALLlamda*dihedraldeg   % per degree,  McCormick Equation 9.134 page 530.

% Sweep contribution
% Tabular data is from Figure 9.48 on page 533 of McCormick
ARmat=[1 2 4 6 8]; % Aspect ratio
LAMDAmat=[-20 -10 0 10 20 30 40 50 60]; %BIGlamda Sweep angle (degree)
Fig948mat =[0.0011877    0.0013169    0.0015015    0.0016861      0.00176 % ClbetaperCl (per degree)
   0.00058088   0.00065471    0.0007101   0.00078394   0.00087625
            0            0            0            0            0
  -0.00055911  -0.00065142  -0.00076219  -0.00078065  -0.00087296
   -0.0011291   -0.0013322   -0.0015353   -0.0016276   -0.0017199
   -0.0018283   -0.0020684   -0.0023453   -0.0025299   -0.0027145
   -0.0025645   -0.0030076   -0.0035245    -0.003746   -0.0040045
   -0.0034299   -0.0042976    -0.005073   -0.0054422   -0.0058115
   -0.0043691    -0.006363   -0.0074523   -0.0081538   -0.0087261];
[ClbetaperCl] = interlim2(ARmat,LAMDAmat,Fig948mat,AR_w,biglamda);
Qlamda= (1+2*lambda_w)/(3*(1+lambda_w)); % from equation 9.136, page 532
Qpoint5=(1+2*.5)/(3*(1+.5));
qbar=.5*rho*U1*U1;
CLtrim=W/(qbar*S_w);
ClbetaSweep=ClbetaperCl*CLtrim*Qlamda/Qpoint5 % per degree, from equation 9.137, page 532

% Add up the individual contributions
Cl_beta_perdeg=Cl_beta_wingposition+Cl_beta_vert_tail+Clbetadihedral+ClbetaSweep % per degree
% Change units
Cl_beta=Cl_beta_perdeg*fiftyseven;  % per radian