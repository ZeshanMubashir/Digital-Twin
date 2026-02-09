clear;
clc;
close all;

%% Model Function for Ship Dynamics : dot{x}(t) = f(x) + B*u+hat{T}(w), t>0, x(0)=x0
% State Estimation by Adaptive Extended Kalman Filter
%% time horizon
tf  = 20;
dt  = 0.01;
t   = dt:dt:tf;

%% system description
A = eye(6);
B_t = [1 0;0 0; 0 1];
M = [25.8 0 0;0 33.8 6.2;0 6.2 2.76];
B = dt*[0 0;0 0;0 0;inv(M)*B_t];
C = eye(6);

%% noise
QF = 0.01*eye(rank(A));
RF = 0.04*eye(rank(C));

%% Initialization
x        = [0;0;0;0;0;0];
xbar     = [0;0;0;0;0;0];
xhat     = [0;0;0;0;0;0];
theta    = [0 0]';
thetabar = [0 0]';
thetahat = [0 0]';
temp     = [0 0 0 0 0 0]';
Pplus    = eye(rank(A));
 
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

u     = [2 0.1]';
at    = 0;
Psi   = -B*diag(u);
S     = 0.1;
UpsilonPlus = 0*B;
lambda      = 0.995;
a           = 0.999;

%% For plotting
uArray     = [];
xArray     = [];
xhatArray  = [];
thetaArray  = [];
thetahatArray  = [];

%% Simulation
for i=1:(tf/dt)
    
     Psi   = -B*diag(u);
    
    if i>1000
        u = [4 0.2]';
    end
    
    if i>5000
        theta    = [0.3 0.25]';
    end

    if i>10000
        u = [1 -0.1]';
    end    
    
    if i>15000
        theta = [0 0.141]';
    end    
     
    uArray         = [uArray u];
    xArray         = [xArray x];
    xhatArray      = [xhatArray xhat];
    thetaArray     = [thetaArray theta];
    thetahatArray  = [thetahatArray thetahat]; 
 
    c13 = -m22*x(5)-((m23+m32)/2)*x(6);
    c23 = m11*x(4);
    Cv  = [0 0 c13; 0 0 c23;-c13 -c23 0];
    Dv  = -[Xu+Xuu*abs(x(4)) 0 0;0 Yv+Yvv*abs(x(5)) Yr;0 Nv Nr+Nrr*abs(x(6))];
    
    x = A*x+dt*[cos(x(3))*x(4)-sin(x(3))*x(5);sin(x(3))*x(4)+cos(x(3))*x(5);x(6);-inv(M)*(Cv+Dv)*[x(4);x(5);x(6)]]+B*u+Psi*theta+QF*dt*randn(6,1);
    y = C*x+RF*dt*randn(6,1);
    
     % Adaptive Extended Kalman filter
     FX    = A+dt*[0 0 -sin(xhat(3))*xhat(4)-cos(xhat(3))*xhat(5) cos(xhat(3)) -sin(xhat(3)) 0;
                   0 0 cos(xhat(3))*xhat(4)-sin(xhat(3))*xhat(5) sin(xhat(3)) cos(xhat(3)) 0; 
                   0 0 0 0 0 1;
                   -inv(M)*[0 0 0 Xu+2*Xuu*abs(xhat(4)) -m22*xhat(6) -m22*xhat(5)-(m23+m32)*xhat(6);
                            0 0 0 Yr*xhat(6)+m11*xhat(6) Yv+2*Yvv*abs(xhat(5)) Yr+m11*xhat(4); 
                            0 0 0 m22*xhat(5)+((m23+m32)/2)*xhat(6)-m11*xhat(5) m22*xhat(4)+Nv-m11*xhat(4) ((m23+m32)/2)*xhat(4)+Nr+2*Nrr*abs(xhat(6))]];
 
     Pmin  = FX*Pplus*FX'+QF;
     Sigma = C*Pmin*C'+RF;
     KF    = Pmin*C'*inv(Sigma);
     Pplus = (eye(rank(A))-KF*C)*Pmin;
     
     ytilde = y-C*xhat;
     QF    = a*QF + (1-a)*(KF*(ytilde*ytilde')*KF');    
     RF    = a*RF + (1-a)*(ytilde*ytilde'+C*Pmin*C');
 
     Upsilon = (eye(rank(A))-KF*C)*FX*UpsilonPlus+(eye(rank(A))-KF*C)*Psi;
     Omega   = C*FX*UpsilonPlus+C*Psi;
     Lambda  = inv(lambda*Sigma+Omega*S*Omega');
     Gamma   = S*Omega'*Lambda;
     S       = (1/lambda)*S-(1/lambda)*S*Omega'*Lambda*Omega*S;
     UpsilonPlus = Upsilon;
     
     thetahat  = thetahat + Gamma*(y-C*xhat);
     xhat      = A*xhat+dt*[cos(xhat(3))*xhat(4)-sin(xhat(3))*xhat(5);
                            sin(xhat(3))*xhat(4)+cos(xhat(3))*xhat(5);
                            xhat(6);-inv(M)*(Cv+Dv)*[xhat(4);
                            xhat(5);xhat(6)]]+B*u+Psi*thetahat+KF*(y-C*xhat)+Upsilon*Gamma*(y-C*xhat);

    % creating csv file for ship state estimated movement in XZ Coordinates of Unity 
    csvdata = [xhatArray(1,:)' xhatArray(2,:)'];
    writematrix (csvdata,'XZShipCoordinates_1.csv')
end

figure(1);
subplot(2,2,[1,3])
plot(xArray(1,:),xArray(2,:), 'k', 'LineWidth', 3)
hold on;
plot(xhatArray(1,:),xhatArray(2,:), 'r:', 'LineWidth', 3)
grid on;
grid minor
ylabel('y (m)','FontSize',36)
xlabel('x (m)','FontSize',36)
set(gca,'FontSize',12)
legend('true','estimated','FontSize',18);
subplot(2,2,2)
plot(t,xArray(1,:)-xhatArray(1,:), 'k', 'LineWidth', 3)
grid on;
grid minor
ylabel('error x','FontSize',36)
xlabel('t (s)','FontSize',18)
set(gca,'FontSize',12)
xline(10,':b', 'LineWidth', 3)
xline(15,':b', 'LineWidth', 3)
text(12,1.5,'fault','FontSize',18)
subplot(2,2,4)
plot(t,xArray(4,:)-xhatArray(4,:), 'k', 'LineWidth', 3)
grid on;
grid minor
ylabel('error y','FontSize',36)
xlabel('t (s)','FontSize',36)
set(gca,'FontSize',12)
xline(10,':b', 'LineWidth', 3)
xline(15,':b', 'LineWidth', 3)
text(12,1.5,'fault','FontSize',18)

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

figure(3)
subplot(2,1,1)
plot(t,thetaArray(1,:), 'k', 'LineWidth', 3)
hold on;
plot(t,thetahatArray(1,:), 'r:', 'LineWidth', 3)
ylabel('\theta_1','FontSize',18)
xlabel('t (s)','FontSize',18)
grid on
grid minor
set(gca,'FontSize',18)
ylim([-0.05 0.35])
subplot(2,1,2)
plot(t,thetaArray(2,:), 'k', 'LineWidth', 3)
hold on;
plot(t,thetahatArray(2,:), 'r:', 'LineWidth', 3)
legend('true \theta','estimated \theta','FontSize',18);
ylabel('\theta_2','FontSize',18)
xlabel('t (s)','FontSize',18)
grid on
grid minor
set(gca,'FontSize',18)
ylim([-0.05 0.3])