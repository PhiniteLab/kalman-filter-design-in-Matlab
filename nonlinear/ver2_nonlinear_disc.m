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

dt = 1e-3;

t = [ti:dt:tf];

% inital cond

yi_1 = 2;
yi_2 = -2;

%% vander pol equation

% odeset
options = odeset('RelTol',1e-5,'AbsTol',1e-5,'MaxStep',10e-3);

[t_ode,y_ode] = ode45(@vdp1,[ti tf],[yi_1; yi_2],options); % func, time span, initial

plot(t_ode,y_ode(:,1),'-o')
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
plot(t,z(:,1),'-o');
legend('y_ode','z_ode');


