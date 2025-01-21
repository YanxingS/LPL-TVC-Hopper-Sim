clc; clear all; close all;

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

% distance from cg to centroid

xf = (96/2)/12; % in feet
xe = -(20-20/4)/12; % in feet

%cg location

x_cg = ((mf*xf)+(me*xe))/(mf+me);

% MOI matrix cal

Ixx = 0.5*mf*(Df/2)^2+(3/10)*me*(De/2)^2;
Iyy = (1/12)*mf*(3*(Df/2)^2+Hf^2)+(3/80)*me*(4*(De/2)^2+He^2)+(mf*(xf-x_cg))^2+(me*(xe-x_cg))^2;
Izz = Iyy;

S_TD = [Ixx ;Iyy ;Izz]; % MOI about principal axis
S_Eb = [(3/10)*me*(De/2)^2; (3/80)*me*(4*(De/2)^2+He^2); (3/80)*me*(4*(De/2)^2+He^2)]; % MOI of engine wrt body cg


