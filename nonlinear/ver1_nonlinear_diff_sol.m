%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% nonlinear ode solution

clear all;
close all;
clc;


%% vander pol equation

% odeset
options = odeset('RelTol',1e-5,'AbsTol',1e-5,'MaxStep',10e-3);

[t,y] = ode45(@vdp1,[0 20],[2; 0],options); % func, time span, initial

plot(t,y(:,1),'-o',t,y(:,2),'-o')
title('Solution of van der Pol Equation (\mu = 1) with ODE45');
xlabel('Time t');
ylabel('Solution y');
legend('y_1','y_2')


