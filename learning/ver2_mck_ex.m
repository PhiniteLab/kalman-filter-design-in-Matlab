%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Forward - Backward - Centered Difference comparison

clear all;
close all;
clc;

%% initial value
x_0 = 0;
v_0 = 0;

syms s;

m = 1;
c = 2;
k = 1;

F_s = s/(s^2 + 2^2);

X_s = F_s*1/(m*s^2 + c*s + k);

X_t_cont = ilaplace(X_s);

fplot([X_t_cont])
xlim([0 10])
grid on
xlabel('Time (sec)')
ylabel('Position (m)')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% numerical solution

dt = 0.01;

% initial and final time
tf = 10;
ti = 0;

t = [ti:dt:tf-dt]';

%% how many points are utilized in loop

length_of_loop = (tf - ti)/dt;


%%%%%%%%%%%%%%%% FORWARD DIFFERENCE %%%%%%%%%%%%%%%%

X_t_f = zeros(length_of_loop,1);
V_t_f = zeros(length_of_loop,1);

%% input
F = cos(2*t);

% initial condition update
X_t_f(1,1) = x_0;
V_t_f(1,1) = v_0;

for i = 1 : 1 : length_of_loop - 1
   
    X_t_f(i+1,1) = X_t_f(i,1) + dt*V_t_f(i,1);
    V_t_f(i+1,1) = V_t_f(i,1) + dt*(F(i)/m - c/m*V_t_f(i,1) - k/m*X_t_f(i,1));
    
end

hold on
plot(t,X_t_f)














