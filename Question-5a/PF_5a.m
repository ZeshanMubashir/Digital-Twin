clear;
clc;
close all;

%% Model Function for Ship Dynamics : dot{x}(t) = f(x) + B*u+hat{T}(w), t>0, x(0)=x0

%% State Initialization %%

s = [0;0;0];      % s=[x y theta]T
v = [0;0;0];      % v=[u v r]T
x = [s;v];        % initial conditions

%% Input Constraints & Initial Control Input u=[Tu Tr]T
Tu = 2; % Tu should be in range [-5 to +5]
Tr = 0.1; % Tr should be in range [-0.2 to +0.2]
u = [Tu;Tr];

%% Vessel Parameters %%

% Mass Matrix Parameters
m11 = 25.80;
m22 = 33.80;
m23 = 6.20;
m32 = 6.20;
m33 = 2.76;	

% Damping Matrix (Linear Parameters)
Xu = -12.0;
Yv = -17.0;
Yr = -0.2;
Nv = -0.5;
Nr = -0.5;

% Damping Matrix (Non-Linear Parameters)
Xuu = -2.1;
Yvv = -4.5;
Nrr = -0.1;
 
% Vessel Parameters
Ls = 1.20;
Ws = 0.35;
m = 17;

%% ////////// System Description ////////////// %%

A = eye(6);

% Mass Matrix
M = [m11 0 0;0 m22 m23;0 m32 m33];

% Actuator Configuration Matrix
B_t = [1 0;0 0; 0 1];

B = [0 0;0 0;0 0; inv(M)*B_t];

C = eye(6);

%% noise
QF = 0.01*eye(rank(A));
RF = 0.04*eye(rank(C));

%% for plotting
uArray = [];
xArray = [];

%% Time Horizon
T  = 20;      % simulation time
dt = 0.01;   % timestep
t  = dt:dt:T; % discretize timestep
N  = T/dt;

%% Ship Simulation
for i = 1:N

    if i>5000
        u = [3 -0.2]';
    end

    if i>10000
        u = [1 +0.2]';
    end
     
    uArray = [uArray u];
    xArray = [xArray x];
    

    % Coriolis and centripetal effects matrix
    c13 = -m22*x(5)-((m23+m32)/2)*x(6);
    c23 = m11*x(4);
    Cv  = [0 0 c13; 0 0 c23;-c13 -c23 0]; 

    % Damping Matrix
    Dv  = -[Xu+Xuu*abs(x(4)) 0 0;
            0 Yv+Yvv*abs(x(5)) Yr;
            0 Nv Nr+Nrr*abs(x(6))];

    % Eulers Method
    x = A*x+dt*[cos(x(3))*x(4)-sin(x(3))*x(5);
                sin(x(3))*x(4)+cos(x(3))*x(5);
                x(6);-inv(M)*(Cv+Dv)*[x(4);x(5);x(6)]]+dt*(B*u)...
        +QF*dt*randn(6,1);
    y = C*x+RF*dt*randn(6,1);

    % creating csv file for ship movement in XZ Coordinates of Unity 
    csvdata = [xArray(1,:)' xArray(2,:)'];
    writematrix (csvdata,'XZShipCoordinates.csv')

end

figure(1);
plot(xArray(1,:),xArray(2,:), 'k', 'LineWidth', 3)
grid on;
grid minor
ylabel('y (m)','FontSize',36)
xlabel('x (m)','FontSize',36)
set(gca,'FontSize',12)
figure(2);
plot(t,uArray(1,:), ':b', 'LineWidth', 3)
hold on;
plot(t,uArray(2,:), ':r', 'LineWidth', 3)
grid on;
grid minor
ylabel('u','FontSize',36)
xlabel('t (s)','FontSize',36)
legend('a','r','FontSize',18)
set(gca,'FontSize',18)


