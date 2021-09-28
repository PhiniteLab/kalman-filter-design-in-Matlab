%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% nonlinear ode solution + our solution

clear all;
close all;
clc;


%% constants
mu = 1;

ti = 0;
tf = 10;

dt = 1e-4;

t = [ti:dt:tf];

% inital cond

yi_1 = 3;
yi_2 = 0;

%% vander pol equation

% odeset
options = odeset('RelTol',1e-5,'AbsTol',1e-5,'MaxStep',10e-3);

[t_ode,y_ode] = ode45(@vdp1,[ti tf],[yi_1; yi_2],options); % func, time span, initial

plot(t_ode,y_ode(:,1))
hold on;
title('Solution of van der Pol Equation (\mu = 1) with ODE45');
xlabel('Time t');
ylabel('Solution y');




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% our solution

% state matrices
length_of_loop = length(t);

z = zeros(length_of_loop,2);

z(1,:) = [yi_1 yi_2];

for i = 1 : 1 : length_of_loop - 1

    
    Atr = [1,         dt;
          -dt,   (mu*(1 - z(i,1)^2)*dt + 1)];
      
    z(i+1,:) = Atr*z(i,:)';

end

hold on;
plot(t,z(:,1));
legend('y_ode','z_ode');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% KALMAN FILTER DESIGN


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% kalman parameters
% Measurement noise variance
MNstd = 0.004;
MNV = MNstd*MNstd;
% Process noise variance
PNstd = 0.002;
PNV = PNstd*PNstd;

clear s;

s.A = [1,         dt;
      -dt,   (mu*(1 - z(i,1)^2)*dt + 1)];
      
    Btr = [0;
           0];  
       
% Process noise covariance matrix
s.Q = 0.00001*eye(2)*PNV;

% Define measurement function to return the state
s.H = [1, 0;
       0, 1];

C_out = [1, 0;
        0, 1];
    
% Define a measurement error
%s.R = 1e5*eye(2)*MNV; % variance
s.R = 1e-2*eye(2)*MNV; % variance
    
% Use control
s.B = [0;
       0]; % Control matrix
    
   
% Initial state:
s.x = [yi_1 yi_2]';
s.P = eye(2)*MNV;
s.detP = det(s.P); % Let's keep track of the noise by
%keeping detP
s.z = zeros(2,1);
%% kalman parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% state matrices
length_of_loop = length(t);

z_noise = zeros(length_of_loop,2);
z_noise_out = zeros(length_of_loop,2);
z_tru = zeros(length_of_loop,2);
z_kalman = zeros(length_of_loop,2);

z_noise(1,:) = [yi_1 yi_2];
z_noise_out(1,:) = [yi_1 yi_2];
z_tru(1,:) = [yi_1 yi_2];
z_kalman(1,:) = [yi_1 yi_2];

for i = 1 : 1 : length_of_loop - 1

    %% noise
    noise_process = PNstd*randn(2,1);
    noise_measurement = MNstd*randn(2,1);
    
    %% system eq
    Atr = [1,         dt;
          -dt,   (mu*(1 - z(i,1)^2)*dt + 1)];
      
    Btr = [0;
           0];  
      
    z_noise(i+1,:) = Atr*z_noise(i,:)' + noise_process;
    z_noise_out(i+1,:) = C_out*z_noise(i+1,:)' + noise_measurement;
    
    %% kalman dynamic

    Atr_jacob = [1,      dt;
                 -dt*(1 + 2*mu*z(i,1)*z(i,2)),  (mu*(1 - z(i,1)^2))*dt+1];
    
    Btr_jacob = [0;
                 0]; 
             
             
    s(i).z = z_noise_out(i+1,:)';
    s(i+1) = phiKalmanFilter(s(i),0,Atr_jacob,Btr_jacob);
    detP(i+1) = s(i+1).detP;
    
    z_kalman(i+1,:) = s(i+1).x;
    
    
end

hold on;
plot(t,z_noise_out(:,1));
hold on
plot(t,z_kalman(:,1))
legend('y_ode','z_ode','z_noise','kalman');





