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

ti = 0;
tf = 20;

t = [ti:dt:tf-dt]';

%% how many points are utilized in loop

length_of_loop = (tf - ti)/dt;

%% input
F = cos(2*t);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% KALMAN FILTER DESIGN

% Simulate falling in air, and watch the filter track it
tru=zeros(length_of_loop,2); % kalman dynamics
trutr =zeros(length_of_loop,2);% norm dynamics
tru_noisy =zeros(length_of_loop,2); % noisy dynamic 
tru_noisy_out =zeros(length_of_loop,2); % noisy out dynamic 
tru_output = zeros(length_of_loop,2);

tru(1,:)=[x_0 v_0];



%% System matrices
Atr = [1,          dt;
       -k*dt/m,    (1 - dt*c/m)];
   
Btr = [0;
       dt/m];
   
Cout = [1, 0;
        0, 1];
   
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% kalman parameters

% Measurement noise variance
MNstd = 0.4;
MNV = MNstd*MNstd;
% Process noise variance
PNstd = 0.02;
PNV = PNstd*PNstd;

% for position
Bparam = dt;
Fparam = 1;
Qparam = 0.0001*PNV;
Hparam = 1;
Rparam = 0.1*MNV;

xHatPos = zeros(length_of_loop,1);
yHatPos = zeros(length_of_loop,1);
Ppos  = zeros(length_of_loop,1);
Spos = zeros(length_of_loop,1);

xHatVel = zeros(length_of_loop,1);
yHatVel = zeros(length_of_loop,1);
PVel  = zeros(length_of_loop,1);
SVel = zeros(length_of_loop,1);

%% kalman parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% KALMAN FILTER DESIGN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

   
for i = 1 : 1 : length_of_loop - 1
    
    noise_process = PNstd*randn(2,1);
    noise_measurement = MNstd*randn(2,1);
    
    %% true system dynamic
    trutr(i+1,:) = Atr*trutr(i,:)' + Btr*F(i);
     
    %% noisy dynamic
    tru_noisy(i+1,:) = Atr*tru_noisy(i,:)' + Btr*F(i) + noise_process;
    tru_noisy_out(i+1,:) = Cout*tru_noisy(i+1,:)' + noise_measurement;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% kalman dynamic
    
    %%%%%%%%%%%%%%%%%%%%%
    %% for position
    
    %State
    xHatPos(i+1,1) = Fparam*xHatPos(i,1) + Bparam*F(i);

    % State uncertainty
    Ppos(i+1,1)=Fparam*Ppos(i,1)*Fparam' + Qparam;

    %% Update    

    % Innovation or measurement residual
    yHatPos(i+1,1) = tru_noisy_out(i+1,1)-Hparam*xHatPos(i+1,1);

    % Innovation (or residual) covariance
    Spos(i+1,1) = Hparam*Ppos(i+1,1)*Hparam'+Rparam;

    K = Ppos(i+1,1)*Hparam'* inv(Spos(i+1,1));

    xHatPos(i+1,1) = xHatPos(i+1,1) + K*yHatPos(i+1,1);

    Ppos(i+1,1) = (1-K*Hparam)*Ppos(i+1,1);    
    
    %% for position
    %%%%%%%%%%%%%%%%%%%%%
    
    
    %% kalman dynamic
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% true output of kalman filter
    tru_output(i+1,:) = tru_noisy_out(i+1,:);
  
end

figure
plot(t,trutr(:,1))
hold on
plot(t,tru_noisy_out(:,1))
hold on
plot(t,xHatPos(:,1))
legend('True','Noise','Kalman')

Meas_err_noise = trutr(:,1) - tru_noisy_out(:,1);
Meas_Err_noise_cov = sum(Meas_err_noise.*Meas_err_noise)/length(Meas_err_noise)

Meas_err_kalman = trutr(:,1) - xHatPos(:,1);
Meas_Err_kalman_cov = sum(Meas_err_kalman.*Meas_err_kalman)/length(Meas_err_kalman)

figure
plot(t,trutr(:,1))
hold on
plot(t,2 + tru_noisy_out(:,1))
hold on
plot(t,4 + xHatPos(:,1))
legend('True','Noise','Kalman')



