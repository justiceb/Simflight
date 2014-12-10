% *********************************************
% BasicConstants_MPX5   Version 9.51 1/25/2010
% Removed Mach, CL, Cd calculations
% Corrected U1 to U=u/1.687808
%
% *********************************************
% BasicConstants_MPX5   Version 9.2 1/18/06
% This version requires Xcg and low_wing to be defined here.
%  
% OBJECTIVE: Collect into one location all the vehicle specific constants (a.k.a. basic constants).
%            From these basic constants all the stability and control derivatives 
%            can be determined.
% INPUTS: None
% OUTPUTS: Many basic constants defined in the Matlab workspace.
%

% This version is the second one
% Arbitrary reference point is the firewall
% Moment reference point is the c.g.
% Trim velocity assumed to be 90 ft/s
%
% *********************************************
% BasicConstants - Identifies, describes, and assigns all of the 
%                  the most basic variables for analyzing the control
%                  and stability of a generic aircraft.
% *********************************************     
% 
% A&AE 565 Spring 2003 - Purdue University
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
% (Ref.3) Roskam, Jan. "Airplane Design: Part IV: Preliminary Calculation
%         of Aerodynamic, Thrust and Power Characteristics"
%         Published by Roskam Aviation and Engineering Corporation
%         Rt4, Box 274
%         Ottawa, Kansas 66067
%         Second Printing, 1990.
%
disp(' '); disp('Starting BasicConstants'); disp(' ')
aircraft='T2 CDR (See Masters Thesis of Mark Peters,1996)';
adelf = 0;                   % Two dimensional lift effectiveness parameter Ref.(2),Equ(8.7)
alpha = 0*pi/180;     % Trim Angle of attack [rad]. This should be zero since our
%                                          equations of motion are body axis system rather then the stability axis system.
alpha_0 = -5.2*pi/180;                % Airfoil zero-lift AOA [rad]
altitude= 200;                % Trim altitude [ft]
disp(['Trim altitude= ',num2str(altitude),' ft'])
AR_h = 2.07637;                        % Aspect ratio of the horizontal tail
AR_w = 6.709;              % Aspect ratio of the wing
b_f = 0/12;                 % Span of the flap [ft] (Alieron total span)****
b_h = 1.03136;                   % Span of the horizontal tail [ft]
b_h_oe = b_h;                % Elevator outboard position [ft]
b_h_ie = 0;                  % Elevator inboard position [ft]
b_w = 5.87;                  % Span of the wing [ft]
b_v = 0.429*2 + 0.417/2;                  % Vertical tail span measured from fuselage centerline[ft]
b_v_or = b_v;                  % Outboard position of rudder [ft]
b_v_ir = 0;                     % Inboard position of rudder [ft]
c_a = 0.21;                   % Chord of aileron [ft]
C_bar_D_o = 0.07;       % Parasite drag
Cd_0 = 0.078;               % Drag coefficient at zero lift (parasite drag)
c_e = 0.25;                         % Elevator chord [ft]
cf = 0/12;                      % Chord of the wing flap [ft]
c_h = 0.497;                  % Mean aerodynamic chord of the horizontal tail [ft]
Cl_alpha_h = 0.106/0.0174532925;                % 2-D Lift curve slope of horizontal tail
Cl_alpha_v = 0.106/0.0174532925;              % 2-D Lift curve slope of vertical tail
Cl_alpha = 0.1087/0.0174532925;             % 2-D lift curve slope of whole aircraft
Cl_alpha_w = 0.1087/0.0174532925;           % 2-D lift curve slope of wing
Cm_0_r = -0.08;               % Zero lift pitching moment coefficient of the wing root 
Cm_o_t = -0.08;               % Zero lift pitching moment coefficient of the wing tip **Cm_0_r = Cm_o_t because wing has no twist
c_r =  0.25;                 % MEAN Chord of the rudder [ft]
c_w = 0.857;                 % Mean aerodynamic chord of the wing [ft]
c_v = 0.497;                   % Mean aerodynamic chord of the vertical tail [ft]
D_p = 11/12;          % Diameter of propeller [ft]
d = 0.394;                                % Average diameter of the fuselage [ft]
delf = 0;             % Streamwise flap deflection [rad] NO FLAPS
delta_e = 0;                    % Elevator deflection [rad]
delta_r = 0;          % Rudder deflection [rad]
dihedral = 0*pi/180;    % Geometric dihedral angle of the wing [rad], positive for 
%                                                   dihedral (wing tips up), negative for
%                                                   anhedral(tips down) [rad] ***EST
dihedral_h = 0*pi/180;       % Geometric dihedral angle of the horizontal tail [rad]

e = 0.858;           % Oswald's efficiency factor 
epsilon_t = 0;        % Horizontal tail twist angle [rad]
epsilon_0_h = 0*pi/180;             % Downwash angle at the horizontal tail (see Note in 
%                                                Ref.(3) under section 8.1.5.2) [rad] ***EST
eta_h = 1.0;                      % Ratio of dynamic pressure at the horizontal tail to that of the freestream ***EST
eta_ia = 1.33/(5.87/2);                  % Percent semi-span position of inboard edge of aileron
eta_oa = 1;               % Percent semi-span position of outboard edge of aileron
eta_p = 0.65;                        % Propeller Efficiency ***EST
eta_v = 1.0;                        % Ratio of the dynamic pressure at the vertical tail 
%                                                to that of the freestream
h1_fuse = 0.417;               % Height of the fuselage at 1/4 of the its length 
h2_fuse = 0.417;              % Height of the fuselage at 3/4 of the its length 
h_h = 0/12;                   % Height from chord plane of wing to chord plane of
%                                                horizontal tail [ft] - Fig 3.7, Ref. 2
hmax_fuse = 0.417;             % Maximum height of the fuselage [ft]  
Ixx = 0.138*0.737562149;          % Airplane moment of inertia about x-axis [slug-ft^2] *** With 4 lb load
Iyy = 0.135*0.737562149;          % Airplane moment of inertia about y-axis [slug-ft^2]
Izz = 0.262*0.737562149;         % Airplane moment of inertia about z-axis [slug-ft^2]
Ixz = 0;             % Airplane product of inertia [slug-ft^2]
i_h = 0*pi/180;      % Incidence angle of horizontal tail [rad]  
i_w = 0*pi/180;        % Incidence angle of wing [rad]  

k = 1/(pi*AR_w*e);                    % k of the drag polar, generally= 1/(pi*AR*e)
Lambda = 0*pi/180;            % Sweep angle of wing [rad]
Lambda_c2 = 0*pi/180;  % Sweep angle at the c/2 of the wing [rad]
Lambda_c4 = 0*pi/180;     % Sweep angle at the c/4 of the wing [rad]     
Lambda_c2_v = 0*pi/180;      % Sweep angle at the c/2 of the vertical tail [rad]
Lambda_c4_v = 0*pi/180;      % Sweep angle at the c/4 of the vertical tail [rad]
Lambda_c2_h = 0*pi/180;      % Sweep angle at the c/2 of the horizontal tail [rad]
Lambda_c4_h = 0*pi/180;      % Sweep angle at the c/4 of the horizontal tail [rad]
lambda = 1.0;     % Taper ratio of wing
lambda_h = 1;                    % Taper ratio of horizontal tail
lambda_v = 1;         % Taper ratio of vertical tail
l_f = 1.42;                   % Horizontal length of fuselage [ft]  
l_v = 2.7;                   % Horizontal distance from aircraft arbitrary reference point to vertical tail AC [ft]
%Ref fig 2.1 in thesis for l_v, ref pt is c/4
low_wing=-1;            % low_wing=-1 if the wing is high  
                        % low_wing=1 if the wing is low  
                        % low_wing=0 if the wing is mid 
% Trim Airspeed
u = 30.25; % ft/sec
S_b_s = 0.59;            % Body side area [ft^2]
S_h = 0.512;                    % Area of horizontal tail [ft^2]
S_h_slip = 0.4;                % Area of horizontal tail that is covered in 
%                                                prop-wash [ft^2] - See Fig.(8.64) - Ref.(3) ***EST
S_o = 0.164;                    % Fuselage x-sectional area at Xo [ft^2] - 
%                                                See Fig.(7.2) - Ref.(2)
%                                                 Xo is determined by plugging X1/l_b into: 
%                                                0.378 + 0.527 * (X1/l_b) = (Xo/l_b)
S_w = 5.13625;          % Surface area of wing [ft^2]
S_v = 0.4265*2;               % Surface area of vertical tail [ft^2]
tc_w = 0.12;             % Thickness to chord ratio of wing 
tc_h = 0.12;             % Thickness to chord ratio of horizontal tail
theta = 0*pi/180;    % Wing twist - negative for washout [rad]
theta_h = 0*pi/180;  % Horizontal tail twist between the root and tip 
%                                                stations,negative for washout [rad]
two_r_one = 0/12;             % Fuselage depth in region of vertical tail [ft] Ref.(2),Figure 7.5

U = u/1.687808; % knots                % Free Stream Velocity (Trim velocity) [KNOTS true]
disp(['Trim airspeed= ',num2str(U),' knots'])
W = 6.25;           % Weight of Airplane [lbf]
wingloc = 1;                % If the aircraft is a highwing: (wingloc=1), low-wing:(wingloc=0) 
wmax_fuse = 6.675/12;   % Maximum fuselage width [ft]
X1 = 1.2;                           % Distance from the front of the fuselage where the 
%                                                x-sectional area decrease (dS_x/dx) 
%                                                is greatest (most negative) [ft] - Ref.(2),Fig. 7.2  
x_m = 0.577;                    % Distance from nose of aircraft to arbitrary reference point [ft]
%                                                measured positive aftward.
x_over_c_v = 0.25;              % PARAMETER ACCOUNTING FOR THE RELATIVE POSITIONS OF THE HORIZONTAL AND VERTICAL TAILS
%                                                defined as the fore-and-aft distance from leading edge of vertical fin to the
%                                                aerodynamic center of the horizontal tail divided by the chord of the vertical tail 
%                                                [nondimensional] - See Fig 7.6 of Ref. 2 
Xach = 2.81716;            % Distance from the leading edge of the wing mean aerodynamic chord
%                       to the aerodynamic center of the horizontal tail (positive aftward) [ft]
Xacwb = .232*c_w;     % Distance from the leading edge of the wing mean aerodynamic chord
%                       to the aerodynamic center of the wing and body. 
%                                                Measured as positive aft, starting from the leading edge of the mean aero. chord. [ft]
Xacw = .232*c_w;      % Distance from the leading edge of the wing mean aerodynamic chord
%                       to the aerodynamic center of the wing ALONE. 
%                                                Measured as positive aft, starting from the leading edge of the mean aero. chord. [ft]
Xref = 0.237*c_w;      % Distance from the leading edge of the wing mean aerodynamic chord
%                       to the arbitrary moment reference point. The equivalent force system
%                                                 for the aerodynamic force system is given about this point.
%                                                Measured as positive aft, starting from the leading edge of the mean aero. chord. [ft]

Xcg = 0.237*c_w;       % Distance from the leading edge of the wing mean aerodynamic chord
%                       to the center of gravity. 
%						Measured as positive aft, starting from the leading edge of the mean aero. chord. [ft]
%                     % Xcg is ignored until Step 2. It an be changed later in Step 2.
%                       





Z_h = -0.417/2;                        % Negative of the VERTICAL distance from the fuselage 
%                                                centerline to the horizontal tail aero center 
%                                                (Z_h is a negative number FOR TAILS ABOVE THE CENTERLINE)
%                                                - Ref.(2), Fig.7.6
%                                                ***This produces a bunch of interpolation errors because 
%                                                Roskam doesn't have data for horizontal tails below the 
%                                                centerline of the fuselage
Z_v = b_h/2;                            % Vertical distance from the aircraft arbirary reference point to the vertical 
%                                                tail aero center (positive up) - Ref.(2), Fig. 7.18
Z_w = 0.417/2;                               % This is the vertical distance from the wing root c/4 [ft]
%                                                to the fuselage centerline, 
%                                                positive downward - Ref.(2), Equ(7.5)
Z_w1 = Z_w;                         % Distance from body centerline to c/4 of wing root 
%                                                chord,positive for c/4 point
%                                                below body centerline (ft) - Ref.(2), Fig. 7.1  

