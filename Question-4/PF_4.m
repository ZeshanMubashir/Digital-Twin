clear;
clc;
close all;

% PF.Q4. Solve the following system on the interval [0,2] using Euler and Runge-Kutta method.

% 1st Sys ODE \double_dot{x}(t) = 2*y(t) - sin(4*t^2*x(t)), 	            initial condition x(0) = 1, dot{x}(t) = 2;
% 2nd Sys ODE \double_dot{y}(t) = -2*x(t) - 1/(2*t^2*dot{x}(t)^2+3), 		initial condition y(0) = 1, dot{y}(t) = 0;

% Converted System of Eqs to 1st Order ODEs: 

T  = 2;      % simulation time
dt = 0.01;    % timestep
t  = 0:dt:T; % discretize timestep

x1E = 1;      % initial consition - Euler method
x2E = 2;      % initial consition - Euler method
x3E = 1;      % initial consition - Euler method
x4E = 0;      % initial consition - Euler method

x1R = 1;      % initial consition - Runge-Kutta method
x2R = 2;      % initial consition - Runge-Kutta method
x3R = 1;      % initial consition - Runge-Kutta method
x4R = 0;      % initial consition - Runge-Kutta method

N  = T/dt;

for i = 1:N

    % Euler method

    x1E(i+1) = x1E(i) + dt*x2E(i);
    x2E(i+1) = x2E(i) + dt*(-sin(4*t(i)^2*x1E(i))+2*x3E(i));
    x3E(i+1) = x3E(i) + dt*x4E(i);
    x4E(i+1) = x4E(i) + dt*(-2*x1E(i)-(1/(((2*t(i)^2*x2E(i)^2)+3))));

    % RK method

    k11 = x2R(i);
    k21 = x2R(i)+dt*(k11/2);
    k31 = x2R(i)+dt*(k21/2);
    k41 = x2R(i)+dt*(k31);
    x1R(i+1) = x1R(i) + (1/6)*dt*(k11+2*k21+2*k31+k41);

    k12 = -sin(4*t(i)^2*x1R(i))+(2*x3R(i));
    k22 = -sin(4*((t(i)+dt/2)^2)*(x1R(i)+dt*k12/2))+(2*(x3R(i)+dt*k12/2));
    k32 = -sin(4*((t(i)+dt/2)^2)*(x1R(i)+dt*k22/2))+(2*(x3R(i)+dt*k22/2));
    k42 = -sin(4*((t(i)+dt)^2)*(x1R(i)+dt*k32))+(2*(x3R(i)+dt*k32));
    x2R(i+1) = x2R(i) + (1/6)*dt*(k12+2*k22+2*k32+k42);
    
    k13 = x4R(i);
    k23 = x4R(i)+dt*k13/2;
    k33 = x4R(i)+dt*k23/2;
    k43 = x4R(i)+dt*k33;
    x3R(i+1) = x3R(i) + (1/6)*dt*(k13+2*k23+2*k33+k43);

    k14 = -2*x1R(i)-(1/((2*t(i)^2*x2R(i)^2)+3));
    k24 = -2*(x1R(i)+dt*k14/2)-(1/((2*(t(i)+dt/2)^2*(x2R(i)+dt*k14/2)^2)+3));
    k34 = -2*(x1R(i)+dt*k24/2)-(1/((2*(t(i)+dt/2)^2*(x2R(i)+dt*k24/2)^2)+3));
    k44 = -2*(x1R(i)+dt*k34)-(1/((2*(t(i)+dt)^2*(x2R(i)+dt*k34)^2)+3));
    x4R(i+1) = x4R(i) + (1/6)*dt*(k14+2*k24+2*k34+k44); 

end

figure(1)
plot(t,x1E,'r','LineWidth',2)
hold on;
% plot(t,x2E,'g:','LineWidth',2)
% hold on;
plot(t,x3E,'b:','LineWidth',2)
hold on;
% plot(t,x4E,'b:','LineWidth',2)
% hold on;

plot(t,x1R,'g','LineWidth',2)
hold on;
% plot(t,x2R,'b:','LineWidth',2)
% hold on;
plot(t,x3R,'r:','LineWidth',2)
hold on;
% plot(t,x4R,'g:','LineWidth',2)
% hold on;

legend('XE','YE','XR', 'YR')

grid on;