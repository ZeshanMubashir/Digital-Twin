clear;
clc;

% 3. Solve equation numerically using the Euler’s and Runge-Kutta’s method

% Portfolio Equation(2b) : 2*double_dot{x}(t) - 5*dot{x}(t) + {x}(t) = 0

T  = 5;      % simulation time
dt = 0.01;   % timestep
t  = 0:dt:T; % discretize timestep

x1E = 1;      % initial condition - Euler method
x2E = 2;      % initial condition - Euler method

x1R = 1;      % initial condition - Runge-Kutta method
x2R = 2;      % initial condition - Runge-Kutta method

N  = T/dt;

for i = 1:N

    % Euler method

    x1E(i+1) = x1E(i) + dt*x2E(i);
    x2E(i+1) = x2E(i) + dt*(-(1/2)*x1E(i))+dt*((5/2)*x2E(i));

    % RK method

    k11 = x2R(i);
    k21 = x2R(i)+dt*(k11/2);
    k31 = x2R(i)+dt*(k21/2);
    k41 = x2R(i)+dt*(k31);
    x1R(i+1) = x1R(i) + (1/6)*dt*(k11+2*k21+2*k31+k41);

    k12 = (-(1/2)*x1R(i))+((5/2)*x2R(i));
    k22 = (-(1/2)*(x1R(i)+dt*k12/2))+(5/2)*((x2R(i)+dt*k12/2));
    k32 = (-(1/2)*(x1R(i)+dt*k22/2))+(5/2)*((x2R(i)+dt*k22/2));
    k42 = (-(1/2)*(x1R(i)+dt*k32))+(5/2)*((x2R(i)+dt*k32));
    x2R(i+1) = x2R(i) + (1/6)*dt*(k12+2*k22+2*k32+k42);

end

figure(1)
plot(t,x1E,'r','LineWidth',2)
hold on;
plot(t,x2E,'g','LineWidth',2)
hold on;
plot(t,x1R,'b:','LineWidth',2)
hold on;
plot(t,x2R,'r:','LineWidth',2)
hold on;
legend('Euler: x1E','Euler: x2E','RK:x1R','RK:x2R')
grid on;