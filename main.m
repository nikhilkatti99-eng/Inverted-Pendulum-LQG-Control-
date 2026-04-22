%% =========================
%  Section 1: System Parameters
% =========================
M = 0.5; 
m = 0.2; 
l = 0.3; 
I = (1/3)*m*(l^2); 
b = 0.1; 
g = 9.81;

p = I*(M+m) + M*m*l^2;

%% =========================
%  Section 2: State-Space Model
% =========================
A = [0 1 0 0;
     0 -(I+m*l^2)*b/p (m^2*g*l^2)/p 0;
     0 0 0 1;
     0 -(m*l*b)/p m*g*l*(M+m)/p 0];

B = [0;
     (I+m*l^2)/p;
     0;
     m*l/p];

C = [1 0 0 0;
     0 0 1 0];

D = [0;0];

sys = ss(A,B,C,D);

disp('Eigenvalues:'), eig(A)
disp('Controllability Rank:'), rank(ctrb(A,B))
disp('Observability Rank:'), rank(obsv(A,C))

%% =========================
%  Section 3: LQR Controller
% =========================
Q = diag([1 1 10 1]);
R = 0.001;
K = lqr(A,B,Q,R);
Acl = A - B*K;

sys_cl = ss(Acl,B,C,D);

%% =========================
%  Section 4: Simulation Setup
% =========================
t = 0:0.01:10;
x0 = [0;0;5*pi/180;0];

%% =========================
%  Section 5: Initial Condition Response
% =========================
[y_ic,~,x_ic] = initial(sys_cl,x0,t);

%% =========================
%  Section 6: Disturbance Response
% =========================
u_dist = zeros(size(t));
u_dist(t>2) = 1;

[y_dist,~,x_dist] = lsim(sys_cl,u_dist,t,x0);

%% =========================
%  Section 7: Control Effort
% =========================
u = (-K*x_ic')';

figure;
plot(t,u)
title('Control Effort')
ylabel('Force')
xlabel('Time')
grid on

%% =========================
%  Section 8: Comparison Plot
% =========================
figure;
plot(t,x_ic(:,3)*(180/pi),'b', ...
     t,x_dist(:,3)*(180/pi),'r')
legend('Initial Condition','Disturbance')
title('Pendulum Angle Response')
ylabel('Theta (deg)')
xlabel('Time')
grid on

%% =========================
%  Section 9: Add Measurement Noise
% =========================
noise = 0.01*randn(size(y_dist));
y_noisy = y_dist + noise;

%% =========================
%  Section 10: Kalman Filter Design
% =========================
G = eye(4);
sys_kf = ss(A,[B G],C,[D zeros(2,4)]);

Qn = 0.01*eye(4);
Rn = 0.1*eye(2);

[kalmf,L,~] = kalman(sys_kf,Qn,Rn);

%% =========================
%  Section 11: Estimation
% =========================
[y_est,~,x_est] = lsim(kalmf,[u_dist y_noisy],t);

figure;
subplot(2,1,1)
plot(t,x_dist(:,3)*(180/pi),'b', ...
     t,x_est(:,3)*(180/pi),'r--')
legend('True','Estimated')
title('Theta Estimation')

subplot(2,1,2)
plot(t,x_dist(:,2),'b', ...
     t,x_est(:,2),'r--')
legend('True','Estimated')
title('Velocity Estimation')

%% =========================
%  Section 12: LQG Simulation
% =========================
dt = 0.01;
x = x0;
x_hat = x0;

x_hist = [];
xhat_hist = [];

for i = 1:length(t)
    
    u = -K*x_hat;
    
    x_dot = A*x + B*u;
    x = x + x_dot*dt;
    
    y = C*x + 0.01*randn(2,1);
    
    xhat_dot = A*x_hat + B*u + L*(y - C*x_hat);
    x_hat = x_hat + xhat_dot*dt;
    
    x_hist(:,i) = x;
    xhat_hist(:,i) = x_hat;
end

figure;
plot(t,x_hist(3,:)*(180/pi),'b', ...
     t,xhat_hist(3,:)*(180/pi),'r--')
legend('True','Estimated')
title('LQG Control Performance')
xlabel('Time')
ylabel('Theta (deg)')
grid on