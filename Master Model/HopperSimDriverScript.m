clc;
clear all;
close all;

%% Hopper Simulation Driver Script
% Spring 2025
% University of Southern California, Viterbi School of Engineering
% Liquid Propulsion Lab
% TVC: Enson Su

%% Properties of Hopper

% all units in imperial, ISP

mf = 600; % mass of fuselage in lb
Df = 12/12; % diameter of fuselage in feet
Hf = 96/12; % length of fuselage in feet

me = 110; % mass of engine
De = 5/12; % diameter of fueslage in feet
He = 20/12; % length of cone in feet

% total mass

mt = me+mf;

% distance from cg to centroid
       % also will change w.r.t. gimbal angle

xf = (96/2)/12; % in feet
xe = -(20-20/4)/12; % in feet

%cg location

x_cg = ((mf*xf)+(me*xe))/(mf+me);

% MOI matrix cal
      % eventually add TD change w.r.t. gimbal angle
Ixx = 0.5*mf*(Df/2)^2+(3/10)*me*(De/2)^2;
Iyy = (1/12)*mf*(3*(Df/2)^2+Hf^2)+(3/80)*me*(4*(De/2)^2+He^2)+(mf*(xf-x_cg))^2+(me*(xe-x_cg))^2;
Izz = Iyy;

      % add S_TD change w.r.t. gimbal angle
      % add y & z equations for each
      % add S_Eb change w.r.t. gimbal angle

% S_TD = [x_cg*mf; 0 ; 0]; % eqn 2.1.20, product of the total mass distribution and the position vector
% S_Eb = [(-3/4)*He*me; 0; 0]; % eqn 2.3.21  first moment of inertia of the engine about the gimbal point

S_TD = [0     0           0;
        0     0     -x_cg*(mf+me);
        0 x_cg*(mf+me)    0];
S_Eb = [0                   -(1/4)*(me^2+2*He^2) (1/4)*(me^2+2*He^2);
       (1/4)*(me^2+2*He^2)          0           -(1/2)*me*(De/2)^2;
       -(1/4)*(me^2+2*He^2) (1/2)*me*(De/2)^2           0]; % EN frame
%% Evan -- Slosh not included

% gimbal to TD cm
    syms rgx rgy rgz
rg = [ 0  -rgz  rgy
      rgz   0  -rgx
     -rgy  rgx   0 ];

% Engine inertia matrix / gimbal rotation pt
    syms Iexx Ieyy Iezz Iexy Iexz Ieyz
I_Eb = [Iexx -Iexy -Iexz
      -Iexy  Ieyy -Ieyz
      -Iexz -Ieyz  Iezz];

% Tail Wags Dog Equation
I_twd = I_Eb - cross(rg,S_Eb);

% Rotational Equation
    syms ax ay az g g_NL
a = [0 -az ay;az 0 -ax;-ay ax 0];
omega_dot = [0 0 0];
omega_dot_Eb = [0 0 0];
cross(S_TD,a) + I_TD.*omega_dot + I_twd.*omega_dot_Eb == g + g_NL;

%% User defined I.C.

v_b_0 = [1 1 1]';