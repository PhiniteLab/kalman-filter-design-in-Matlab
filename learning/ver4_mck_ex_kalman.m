%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MCK noise

clear all;
close all;
clc;

%% initial value
x_0 = 0;
v_0 = 0;

m = 1;
c = 2;
k = 1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% numerical solution

dt = 0.001;

% initial and final time
tf = 20;
ti = 0;

t = [ti:dt:tf-dt]';

%% how many points are utilized in loop

length_of_loop = (tf - ti)/dt;

%% input
F = cos(2*t);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% KALMAN FILTER DESIGN


% Dynamics modeled by A
clear s;
s.A = [1,          dt;
       -k*dt/m,    (1 - dt*c/m)];
   

% Measurement noise variance
MNstd = 0.004;
MNV = MNstd*MNstd;
% Process noise variance
PNstd = 0.002;
PNV = PNstd*PNstd;
% Process noise covariance matrix
%s.Q = 1e5*eye(2)*PNV;
s.Q = 0.000001*eye(2)*PNV;
% Define measurement function to return the state
s.H = [1, 0;
       0, 1];
   
C_out = [1, 0;
        0, 1];
% Define a measurement error
%s.R = 1e5*eye(2)*MNV; % variance
s.R = 0.01*eye(2)*MNV; % variance

% Use control to include gravity
s.B = [0;
       dt/m]; % Control matrix
%s.u = F; % Gravitational acceleration
% Initial state:
s.x = [x_0 v_0]';
s.P = eye(2)*MNV;
s.detP = det(s.P); % Let's keep track of the noise by
%keeping detP
s.z = zeros(2,1);


% Simulate falling in air, and watch the filter track it
tru=zeros(length_of_loop,2); % kalman dynamics
trutr =zeros(length_of_loop,2);% norm dynamics
tru_noisy =zeros(length_of_loop,2); % noisy dynamic 
tru_noisy_out =zeros(length_of_loop,2); % noisy out dynamic 
tru_output = zeros(length_of_loop,2);

tru(1,:)=[x_0 v_0];
detP(1,:)=s.detP;



%% System matrices
Atr = [1,          dt;
       -k*dt/m,    (1 - dt*c/m)];
   
Btr = [0;
       dt/m];

for i = 1 : 1 : length_of_loop - 1
    
    noise_process = PNstd*randn(2,1);
    noise_measurement = MNstd*randn(2,1);
    
    %% true system dynamic
    trutr(i+1,:) = Atr*trutr(i,:)' + Btr*F(i);
     
    %% noisy dynamic
    tru_noisy(i+1,:) = Atr*tru_noisy(i,:)' + Btr*F(i) + noise_process;
    tru_noisy_out(i+1,:) = C_out*tru_noisy(i+1,:)' + noise_measurement;
    
    %% kalman dynamic
    tru(i+1,:)=s(i).A*tru(i,:)'+ s(i).B*F(i) + noise_process;
    s(i).z = s(i).H*tru(i+1,:)' + noise_measurement;
    s(i+1) = phiKalmanFilter(s(i),F(i));
    detP(i+1) = s(i+1).detP;
    
    %% true output of kalman filter
    tru_output(i+1,:) = s(i+1).x;
  
end

figure
plot(t,trutr(:,1))
hold on
plot(t,tru_noisy_out(:,1))
hold on
plot(t,tru_output(:,1))
legend('True','Noise','Kalman')

Meas_err_noise = trutr(:,1) - tru_noisy_out(:,1);
Meas_Err_noise_cov = sum(Meas_err_noise.*Meas_err_noise)/length(Meas_err_noise)

Meas_err_kalman = trutr(:,1) - tru_output(:,1);
Meas_Err_kalman_cov = sum(Meas_err_kalman.*Meas_err_kalman)/length(Meas_err_kalman)


