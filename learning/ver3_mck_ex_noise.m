%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MCK noise

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
xlim([0 5])
grid on
xlabel('Time (sec)')
ylabel('Position (m)')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% numerical solution

dt = 0.0001;

% initial and final time
tf = 20;
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

noise_x = wgn(1,1,-80);
noise_v = wgn(1,1,-120);

for i = 1 : 1 : length_of_loop - 1
   

    X_t_f(i+1,1) = ( X_t_f(i,1) + noise_x ) + dt*( V_t_f(i,1) + noise_v );
    V_t_f(i+1,1) = ( V_t_f(i,1) + noise_v ) + dt*(F(i)/m - c/m*( V_t_f(i,1) + noise_v ) - k/m*( X_t_f(i,1) + noise_x ));
    
end

hold on
plot(t,X_t_f)






