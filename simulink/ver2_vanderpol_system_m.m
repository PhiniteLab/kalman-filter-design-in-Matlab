%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ver1_mck_system

clear all;
close all;
clc;

%% constants

mu = 1;


general_constants = [mu];
                     

%% kalman parameters

dt = 0.001;

% Measurement noise variance
MNstd = 0.04;
MNV = MNstd*MNstd;
% Process noise variance
PNstd = 0.02;
PNV = PNstd*PNstd;
% Process noise covariance matrix
%s.Q = 1e5*eye(2)*PNV;
Q = 0.2*eye(2)*PNV;
% Define measurement function to return the state
H = [1, 0;
     0, 1];
 
C = [1,0;
     0,1];

R = 60*eye(2)*MNV; % variance



%% initial conditions

init_cond_cont = [2;
                  0];
              
              
init_cond_disc = init_cond_cont;

P_init_disc = MNV*eye(2);