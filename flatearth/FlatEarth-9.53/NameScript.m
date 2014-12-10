%
% This script contains names for the various constants and comments about their typical values.
% It is used with the script Check_Constants.m
%
name1='W, Weight, pounds (lbf)';
comment1='Always positive.';
name2='g, Acceleration of gravity, ft/(sec*sec)';
comment2='Always roughly 32.2 at sea level, but varies with altitude.';
name3='mass, slugs';
comment3='';
name4='Ixx, slug*ft*ft';
comment4='Always positive.';
name5='Iyy, slug*ft*ft';
comment5='Always positive.';
name6='Izz, slug*ft*ft';
comment6='Always positive.';
name7='Ixz, slug*ft*ft';
comment7='';
name8='propeller efficiency, eta, nondimensional';
comment8='Typically .5 to .8';
name9='unassigned';
comment9='';
name10='constant(4)*constant(6)-constant(7)*constant(7);  %gamma';
comment10='A computed constant.';
name11='=((constant(5)-constant(6))*constant(6)-constant(7)*constant(7))/constant(10);% c1';
comment11='A computed constant.';
name12='=(constant(4)-constant(5)+constant(6))*constant(7)/constant(10);% c2';
comment12='A computed constant.';
name13='=constant(6)/constant(10);% c3 ';
comment13='A computed constant.';
name14='=constant(7)/constant(10);% c4'; 
comment14='A computed constant.';
name15='=(constant(6)-constant(4))/constant(5);% c5'; 
comment15='A computed constant.';
name16='=constant(7)/constant(5);% c6';
comment16='A computed constant.';
name17='=1/constant(5);% c7';
comment17='A computed constant.';
name18='=(constant(4)*(constant(4)-constant(5))+constant(7)*constant(7))/constant(10);% c8' ;
comment18='A computed constant.';
name19='=constant(4)/constant(10);% c9' ;
comment19='A computed constant.';

% aircraft geometry
name20='S_w, wing area, ft^2';
comment20='Always positive.';
name21='c_w, mean geometric chord, ft';
comment21='Always positive.';
name22='b_w, wing span, ft';
comment22='Always positive.';
name23='phiT, thrust inclination angle, RADIANS';
comment23='Typically <.09 radians';
name24='dT, thrust offset distance, ft';
comment24='Typically <5 ft';

% Nondimensional Aerodynamic stability and control derivatives

% Drag Polar CD=k(CLstatic-CLdm)^2 + CDm
name25='CDm, CD for minimum drag for drag polar CD=k(CLstatic-CLdm)^2 + CDm';
comment25='Typically .0200 to .0300';
name26='k';
comment26='Typically .04 to .07';
name27='CLdm, CL at the minimum drag point';
comment27='Typically 0';

% Lift Force
name28='CL_0, For Lift Force Equation';
comment28='Typically 0 to .5';
name29='CL_alpha';
comment29='Lift curve slope. Typically 3 to 6';
name30='CL_de';
comment30='Typically .3 to .9';
name31='CL_alpha_dot';
comment31='1 to 8';
name32='CL_q';
comment32='Typically 4 to 10';
% Side Force
name33='CY0, For Side Force Equation';
comment33='Almost always 0';
name34='Cy_beta';
comment34='Typically -.3 to -1';
name35='Cy_da';
comment35='Typically insignificant. <5% of Cy_dr';
name36='Cy_dr';
comment36='Typically .1 to .2';
name37='Cy_p';
comment37='Typically insignificant. 0 to -.3';
name38='Cy_r';
comment38='Typically .2 to .5';
% Rolling Moment
name39='Cl0, For Rolling Moment Equation';
comment39='Almost always 0.';
name40='Cl_beta';
comment40='Dihedral effect. Typically -.09 to -.3';
name41='Cl_da';
comment41='Aileron effectiveness. Typically .05 to .2';
name42='Cl_dr';
comment42='Typically 0 to .02 for high vertical tail aircraft. Negative for low vertical tail (like the Predator)';
name43='Cl_p';
comment43='Damping in roll. Typically -.3 to -.6';
name44='Cl_r';
comment44='Typically .07 to .2';
% Pitching Moment
name45='Cm_0, For Pitching Moment Equation';
comment45='Important because it must be trimmed away for steady flight.';
name46='Cm_alpha';
comment46='Static longitudinal stability parameter (pitch stiffness). Usually negative. =-CL_alpha*(static margin)';
name47='Cm_de';
comment47='Elevator effectiveness. Typically -1 to -2';
name48='Cm_a_dot';
comment48='Important in damping the short period mode. Typically -3 to -15';
name49='Cm_q';
comment49='Damping in pitch. Important in damping the short period mode. Typically -11 to -30';
% Yawing Moment
name50='CN0, For Yawing Moment Equation';
comment50='Almost always 0.';
name51='Cn_beta';
comment51='Weathercock stability derivative. Typically .06 to .2';
name52='Cn_da';
comment52='Might exhibit adverse (negative) or proverse (positive) aileron yaw. Magnitude<10% of Cn_dr';
name53='Cn_dr';
comment53='Rudder effectiveness. Typically -.06 to -.12';
name54='Cn_p';
comment54='Often insignificant. typically -.02 to -.2';
name55='Cn_r';
comment55='Damping in yaw. Typically -.09 to -.4';
% Reference positions
name56='XbarRef, nondimensional';
comment56='The equivalent force system for the input aerodynamic model uses this as its reference point.';
name57='XbarCG,  nondimensional';
comment57='The equations of motion will use this as a reference point. The correction from XbarRef to XbarCG is handled in the Simulink simulation';

% Trim conditions. These may or not be used by subsequent programs. Small 
% variations in these trim flight conditions are OK.
name58='Trim speed, Vt, ft/sec. These may or not be used by subsequent programs.';
comment58='';
name59='Trim altitude, ft';
comment59='';
name60='Trim alpha, >>>DEGREES<<<This is not used by LongSC';
comment60='';
% The constants below are used only by LongSC and LatSC
name61='CLu=0, These constants are used only by LongSC and LatSC';
comment61='';
name62='CDu=0';
comment62='';
name63='CTxu';
comment63='';
name64='Cmu';
comment64='';
name65='CmTu';
comment65='';
name66='CmTalpha';
comment66='';
name67='CDdeltae';
comment67='';
