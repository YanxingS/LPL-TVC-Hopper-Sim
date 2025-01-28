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

xf = (96/2)/12; % in feet
xe = -(20-20/4)/12; % in feet

%cg location

x_cg = ((mf*xf)+(me*xe))/(mf+me);

% MOI matrix cal

Ixx = 0.5*mf*(Df/2)^2+(3/10)*m e*(De/2)^2;
Iyy = (1/12)*mf*(3*(Df/2)^2+Hf^2)+(3/80)*me*(4*(De/2)^2+He^2)+(mf*(xf-x_cg))^2+(me*(xe-x_cg))^2;
Izz = Iyy;

S_TD = [x_cg*mf; 0 ; 0]; % eqn 2.1.20, product of the total mass distribution and the position vector
S_Eb = [(-3/4)*He*me; 0; 0]; % eqn 2.3.21  first moment of inertia of the engine about the gimbal point


%% User defined I.C.

v_b_0 = [1 1 1]';