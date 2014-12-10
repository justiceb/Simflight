% *********************************************
% BasicConstants_PA-28-161   Version 9.51 2/17/2010
% Removed Mach, CL, Cd calculations
% Corrected U1 to U=u/1.687808
% Revision modified by: Oscar D. Garibaldi C.
% *********************************************
% Modified from BasicConstants_MPX5   Version 9.2 1/18/06
% This version requires Xcg and low_wing to be defined here.
%  
% OBJECTIVE: Collect into one location all the vehicle specific constants (a.k.a. basic constants).
%            From these basic constants all the stability and control derivatives 
%            can be determined.
% INPUTS: None
% OUTPUTS: Many basic constants defined in the Matlab workspace.
%

% Arbitrary reference point is the nose for some cases
% Moment reference point is the c.g.
% Trim velocity assumed to be 147 ft/s
%
% *********************************************
% BasicConstants - Identifies, describes, and assigns all of the 
%                  the most basic variables for analyzing the control
%                  and stability of a generic aircraft.
% *********************************************     
% 
% A&AE 590 F Spring 2010 - Purdue University
% 
% Note: This code is provided for a first order approximation of the dynamic 
%       analysis of an airplane and is not intended for final designs.
%
% Equations/Figures can be found in :
% 
% (Ref.1) Roskam, Jan. "Airplane Flight Dynamics and Automatic Flight
%         Controls"
%         Published by DARcorporation
%         120 E. Ninth St., Suite 2
%         Lawrence, KS 66044
%         Third Printing, 2001.
%
% (Ref.2) Roskam, Jan. "Methods for Estimating Stability and
%         Control Derivatives of Conventional Subsonic Airplanes"
%         Published by the Author 
%         519 Boulder 
%         Lawrence, Kansas 66044
%         Third Printing, 1997.
%  
% (Ref.3) Roskam, Jan. "Airplane Design: Part VI: Preliminary Calculation
%         of Aerodynamic, Thrust and Power Characteristics"
%         Published by Roskam Aviation and Engineering Corporation
%         Rt4, Box 274
%         Ottawa, Kansas 66067
%         Second Printing, 1990.
%
disp(' '); disp('Starting BasicConstants'); disp(' ')
aircraft='PA-28-161 PIPER WARRIOR III';
adelf = 0;                   % Two dimensional lift effectiveness parameter Ref.(2),Equ(8.7)
alpha = 0*pi/180;     % Trim Angle of attack [rad]. This should be zero since our
%                                          equations of motion are body axis system rather then the stability axis system.
alpha_0 = -2.6*pi/180;                % Airfoil zero-lift AOA [rad]
altitude= 2879; %0;                % Trim altitude [ft]
disp(['Trim altitude= ',num2str(altitude),' ft'])
AR_h = 5.67;                        % Aspect ratio of the horizontal tail
AR_w = 7.2;              % Aspect ratio of the wing
b_f = 16.92;                 % Span of the flap [ft] (Aileron total span)****
b_h = 12.98;                   % Span of the horizontal tail [ft]
b_h_oe = 12.98; %8.66;                % Elevator outboard position [ft]
b_h_ie = 0;                  % Elevator inboard position [ft]
b_w = 35;                  % Span of the wing [ft]
b_v = 5.210;                  % Vertical tail span measured from fuselage centerline[ft]
b_v_or = 5.210;                  % Outboard position of rudder [ft]
b_v_ir = 1.074;                     % Inboard position of rudder [ft]
c_a = 0.971;                   % Chord of aileron [ft]
C_bar_D_o = 0.035;       % Parasite drag
Cd_0 = 0.035;               % Drag coefficient at zero lift (parasite drag)
c_e = 2.5; %0.492;                         % Elevator chord [ft]
cf = 0.869;                      % Chord of the wing flap [ft]
c_h = 2.5;                  % Mean aerodynamic chord of the horizontal tail [ft]
Cl_alpha_h = 5.73;                % 2-D Lift curve slope of horizontal tail
Cl_alpha_v = 5.73;              % 2-D Lift curve slope of vertical tail
Cl_alpha = 4.762;             % 2-D lift curve slope of whole aircraft
Cl_alpha_w = 6.22;           % 2-D lift curve slope of wing
Cm_0_r = -0.072;               % Zero lift pitching moment coefficient of the wing root 
Cm_o_t = -0.072;               % Zero lift pitching moment coefficient of the wing tip **Cm_0_r = Cm_o_t because wing has no twist
c_r =  0.944;                 % MEAN Chord of the rudder [ft]
c_w = 4.846;                 % Mean aerodynamic chord of the wing [ft]
c_v = 3.170;                   % Mean aerodynamic chord of the vertical tail [ft]
D_p = 6.168;          % Diameter of propeller [ft]
d = 6.5;                                % Average diameter of the fuselage [ft]
delf = 0;             % Streamwise flap deflection [rad] NO FLAPS
delta_e = 0;                    % Elevator deflection [rad]
delta_r = 0;          % Rudder deflection [rad]
dihedral = 7*pi/180;    % Geometric dihedral angle of the wing [rad], positive for 
%                                                   dihedral (wing tips up), negative for
%                                                   anhedral(tips down) [rad] ***EST
dihedral_h = 0*pi/180;       % Geometric dihedral angle of the horizontal tail [rad]

e = 0.8333;           % Oswald's efficiency factor 
epsilon_t = 0;        % Horizontal tail twist angle [rad]
epsilon_0_h = 0*pi/180;             % Downwash angle at the horizontal tail (see Note in 
%                                                Ref.(3) under section 8.1.5.2) [rad] ***EST
eta_h = 1;                      % Ratio of dynamic pressure at the horizontal tail to that of the freestream ***EST I used Raymer’s approach
eta_ia = 0.491;                  % Percent semi-span position of inboard edge of aileron
eta_oa = 0.975;               % Percent semi-span position of outboard edge of aileron
eta_p = 0.65;                        % Propeller Efficiency ***EST
eta_v = 1;                        % Ratio of the dynamic pressure at the vertical tail (Raymer again)
%                                                to that of the freestream
h1_fuse = 4.167;               % Height of the fuselage at 1/4 of the its length 
h2_fuse = 2.231;              % Height of the fuselage at 3/4 of the its length 
h_h = 1.6;                   % Height from chord plane of wing to chord plane of
%                                                horizontal tail [ft] - Fig 3.7, Ref. 2
hmax_fuse = 4.25 ;             % Maximum height of the fuselage [ft]  
Ixx = 1087.8;          % Airplane moment of inertia about x-axis [slug-ft^2] *** TO Weight
Iyy = 1269.8;          % Airplane moment of inertia about y-axis [slug-ft^2]
Izz = 2350.5;         % Airplane moment of inertia about z-axis [slug-ft^2]
Ixz = 0;             % Airplane product of inertia [slug-ft^2]
i_h = 0*pi/180;      % Incidence angle of horizontal tail [rad]  
i_w = 2*pi/180;        % Incidence angle of wing [rad]  

k = 1/(pi*AR_w*e);                    % k of the drag polar, generally= 1/(pi*AR*e)
Lambda = 0*pi/180;            % Sweep angle of wing [rad]
Lambda_c2 = 0*pi/180;  % Sweep angle at the c/2 of the wing [rad]
Lambda_c4 = 0*pi/180;     % Sweep angle at the c/4 of the wing [rad]     
Lambda_c2_v = 30*pi/180;      % Sweep angle at the c/2 of the vertical tail [rad]
Lambda_c4_v = 36*pi/180;      % Sweep angle at the c/4 of the vertical tail [rad]
Lambda_c2_h = 0*pi/180;      % Sweep angle at the c/2 of the horizontal tail [rad]
Lambda_c4_h = 0*pi/180;      % Sweep angle at the c/4 of the horizontal tail [rad]
lambda = 1.0;     % Taper ratio of wing
lambda_h = 1;                    % Taper ratio of horizontal tail
lambda_v = 0.367;         % Taper ratio of vertical tail
l_f = 17.7265;                   % Horizontal length of fuselage [ft]  
l_v = 12.4672;                   % Horizontal distance from aircraft arbitrary reference point to vertical tail AC [ft]
%Ref fig 2.1 in thesis for l_v, ref pt is c/4
low_wing=1;            % low_wing=-1 if the wing is high  
                        % low_wing=1 if the wing is low  
                        % low_wing=0 if the wing is mid 
% Trim Airspeed  
u = 158.4; % Phugoid Test  %147; % ft/sec
S_b_s = 5.7;            % Body side area [ft^2]
S_h = 29.7;                    % Area of horizontal tail [ft^2]
S_h_slip = 15.38;                % Area of horizontal tail that is covered in 
%                                                prop-wash [ft^2] - See Fig.(8.64) - Ref.(3) ***EST
S_o = 1.895;                    % Fuselage x-sectional area at Xo [ft^2] - 
%                                                See Fig.(7.2) - Ref.(2)
%                                                 Xo is determined by plugging X1/l_b into: 
%                                                0.378 + 0.527 * (X1/l_b) = (Xo/l_b)
S_w = 170;                  % Surface area of wing [ft^2]
S_v = 16.52;                % Surface area of vertical tail [ft^2]
tc_w = 0.15;                % Thickness to chord ratio of wing 
tc_h = 0.12;                % Thickness to chord ratio of horizontal tail
theta = -3*pi/180;          % Wing twist - negative for washout [rad]
theta_h = 0*pi/180;         % Horizontal tail twist between the root and tip 
%                                                stations,negative for washout [rad]
two_r_one = 0.938;             % Fuselage depth in region of vertical tail [ft] Ref.(2),Figure 7.5

U = u/1.687808; % knots                % Free Stream Velocity (Trim velocity) [KNOTS true]
disp(['Trim airspeed= ',num2str(U),' knots'])
W =  2320; %2440;           % Weight of Airplane [lbf]
wingloc = 0;                % If the aircraft is a highwing: (wingloc=1), low-wing:(wingloc=0) 
wmax_fuse = 3.625;   % Maximum fuselage width [ft]
X1 = 15.584;                           % Distance from the front of the fuselage where the 
%                                                x-sectional area decrease (dS_x/dx) 
%                                                is greatest (most negative) [ft] - Ref.(2),Fig. 7.2  
x_m = 6.496;                    % Distance from nose of aircraft to arbitrary reference point [ft]
%                                                measured positive aftward.
x_over_c_v = 2.289/3.071;              % PARAMETER ACCOUNTING FOR THE RELATIVE POSITIONS OF THE HORIZONTAL AND VERTICAL TAILS
%                                                defined as the fore-and-aft distance from leading edge of vertical fin to the
%                                                aerodynamic center of the horizontal tail divided by the chord of the vertical tail 
%                                                [nondimensional] - See Fig 7.6 of Ref. 2 
Xach = 13.088+.25*c_w;            % Distance from the leading edge of the wing mean aerodynamic chord
%                       to the aerodynamic center of the horizontal tail (positive aftward) [ft]
Xacwb = .25*c_w;     % Distance from the leading edge of the wing mean aerodynamic chord
%                       to the aerodynamic center of the wing and body. 
%                                                Measured as positive aft, starting from the leading edge of the mean aero. chord. [ft]
Xacw = .25*c_w;      % Distance from the leading edge of the wing mean aerodynamic chord
%                       to the aerodynamic center of the wing ALONE. 
%                                                Measured as positive aft, starting from the leading edge of the mean aero. chord. [ft]
Xref = .25*c_w;      % Distance from the leading edge of the wing mean aerodynamic chord
%                       to the arbitrary moment reference point. The equivalent force system
%                                                 for the aerodynamic force system is given about this point.
%                                                Measured as positive aft, starting from the leading edge of the mean aero. chord. [ft]

Xcg = 0.805 + 0.1211 ; %1+0.1211;       % Distance from the leading edge of the wing mean aerodynamic chord
%                       to the center of gravity. 
%						Measured as positive aft, starting from the leading edge of the mean aero. chord. [ft]
%                     % Xcg is ignored until Step 2. It can be changed later in Step 2.
%                       





Z_h = -0.56;                        % Negative of the VERTICAL distance from the fuselage 
%                                                centerline to the horizontal tail aero center 
%                                                (Z_h is a negative number FOR TAILS ABOVE THE CENTERLINE)
%                                                - Ref.(2), Fig.7.6
%                                                ***This produces a bunch of interpolation errors because 
%                                                Roskam doesn't have data for horizontal tails below the 
%                                                centerline of the fuselage
Z_v = 3.6742;                            % Vertical distance from the aircraft arbirary reference point to the vertical 
%                                                tail aero center (positive up) - Ref.(2), Fig. 7.18
Z_w = 1.291;                               % This is the vertical distance from the wing root c/4 [ft]
%                                                to the fuselage centerline, 
%                                                positive downward - Ref.(2), Equ(7.5)
Z_w1 = 1.291;                         % Distance from body centerline to c/4 of wing root 
%                                                chord,positive for c/4 point
%                                                below body centerline (ft) - Ref.(2), Fig. 7.1  

