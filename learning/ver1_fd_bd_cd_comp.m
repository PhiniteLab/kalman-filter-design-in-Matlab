%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Forward - Backward - Centered Difference comparison

clear all;
close all;
clc;

%% initial value
x_0 = 5;

syms s;

X_s = x_0/(5 + s);

X_t_cont = ilaplace(X_s);

fplot([X_t_cont])
xlim([0 5])
grid on
xlabel('Time (sec)')
ylabel('Position (m)')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% numerical solution

dt = 0.1;

% initial and final time
tf = 5;
ti = 0;

t = [ti:dt:tf-dt]';

%% how many points are utilized in loop

length_of_loop = (tf - ti)/dt;




%%%%%%%%%%%%%%%% BACKWARD DIFFERENCE %%%%%%%%%%%%%%%%
X_t_b = zeros(length_of_loop,1);

% initial condition update
X_t_b(1,1) = x_0;

for i = 2 : 1 : length_of_loop
   
    X_t_b(i,1) = X_t_b(i-1,1) + dt*(-5)*X_t_b(i-1,1);
    
end

hold on
plot(t,X_t_b)

%%%%%%%%%%%%%%%% FORWARD DIFFERENCE %%%%%%%%%%%%%%%%

X_t_f = zeros(length_of_loop,1);

% initial condition update
X_t_f(1,1) = x_0;

for i = 1 : 1 : length_of_loop - 1
   
    X_t_f(i+1,1) = X_t_f(i,1) + dt*(-5)*X_t_f(i,1);
    
end

hold on
plot(t,X_t_f)


%%%%%%%%%%%%%%%% Centered DIFFERENCE %%%%%%%%%%%%%%%%

X_t_c = zeros(length_of_loop,1);

% initial condition update
X_t_c(1,1) = x_0;
X_t_c(2,1) = x_0;

for i = 2 : 1 : length_of_loop - 1
   
    X_t_c(i+1,1) = X_t_c(i-1,1) + 2*dt*(-5)*X_t_c(i,1);
    
end


hold on
plot(t,X_t_c)

