clear all
close all
clc

%initialize parameters
v0=0.5
Ts=0.1

%% LQR
% Continuous time LQR state space matrices:
A=[0 v0;0 0]
B=[0;1]
C=diag([1 1])
D=[0;0]

% Define discrete LQR state space matrices here:
Ad=
Bd=
Cd=
Dd=

%execute this and then run the Check_LQR_sim.slx simulink simulation
%% LQRI
% Define continuous time LQRI state space matrices here:
A=
B=
C=
D=

% Define discrete time LQRI state space matrices here:
Ad=
Bd=
Cd=
Dd=

%execute this and then run the Check_LQRI_sim.slx simulink simulation