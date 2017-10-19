%   EGR Rate estimation
%   Multi-sensor optimal information fusion Kalman filter example 
%   Samuel Nyffenegger
%   16.10.2017

%%  calculations

clear all; close all; clc;

% control parameters 
dockFig = true;             % dock all figures



% apply control parameters
if dockFig
    set(0,'DefaultFigureWindowStyle','docked');
else
    set(0,'DefaultFigureWindowStyle','normal');
end


%% define system
% dynamics:     x(k+1) = Phi * x(k) + Gamma * w(k)
% measurement:  yi(k)  = H1  * x(k) + vi(k)

% parameters
K = 200;                    % sampling points
Ts = 0.01;                  % sampling time
x0 = zeros(3,1);            % initial state mean
P0 = 0.01*diag(ones(3,1));  % initial state variance 
Q = 1;                      % process noise variance (zero mean)
R = diag([8, 15, 20]) ;     % measurement noise variance (zero mean)
H{1} =[1,0,0];                % measurement matrice sensor 1
H{2} =[0,1,0];                % measurement matrice sensor 2
H{3} =[0,0,1];                % measurement matrice sensor 3
Phi = [1, Ts, Ts^2/2 ; ...  % state evolution matrix
       0, 1,  Ts     ; ...
       0, 0,  1      ];
Gamma = [0; 0; 1];          % process noise matrix


% generate data
clc

% prepare arrays to store data
x_true = zeros(3,K);            % true states
y = zeros(3,K);                 % measurements
t = Ts:Ts:K*Ts;                     % time index

% initial condition and first step
x0_true = mvnrnd(x0,P0)';

% first step
w = mvnrnd(0,Q);
x_true(:,1) = Phi*x0_true + Gamma*w;

% all other steps
for k = 2:K
    % state evolution with added normal noise
    w = mvnrnd(0,Q);
    x_true(:,k) = Phi*x_true(:,k-1) + Gamma*w;
    
    % measurements with added normal noise
    v = mvnrnd(zeros(3,1),R)';
    y(:,k) = [H{1}*x_true(:,k) + v(1) ; ...
              H{2}*x_true(:,k) + v(2) ; ...
              H{3}*x_true(:,k) + v(3) ];
    
end

%% centralized Kalman Filter
clc

% array to store data
x_central = zeros(3,K);

% init
xm = x0;
Pm = P0; 

for k = 1:K
    % step 1: prediction
    xp = Phi*xm; 
    Pp = Phi*Pm*Phi' + Gamma*Q*Gamma';
    
    % step 2: 3x measurement update
    xm = xp;
    Pm = Pp;
    for l = 1:3
        Kk = (H{l}*Pm*H{l}' + R(l,l))\Pm*H{l}';
        xm = xm + Kk*(y(l,k)-H{l}*xm);
        Pm = (eye(3)-Kk*H{l})*Pm*(eye(3)-Kk*H{l})'+Kk*R(l,l)*Kk';        
    end
    
    % store
    x_central(:,k) = xm;
end

% print data
figure(1); clf;
    ax(1) = subplot(311); hold on; grid on;
        plot(t,x_true(1,:),'k','linewidth',1);
        plot(t,x_central(1,:),'b');
        ylabel('$s(t)$','Interpreter','latex')
        legend('x_{1,true}(t)','location','SE')
    ax(2) = subplot(312); hold on; grid on;
        plot(t,x_true(2,:),'k','linewidth',1);
        plot(t,x_central(2,:),'b');
        ylabel('$\dot{s}(t)$','Interpreter','latex')
        legend('x_{2,true}(t)','location','SE')
    ax(3) = subplot(313); hold on; grid on;
        plot(t,x_true(3,:),'k','linewidth',1);
        plot(t,x_central(2,:),'b');
        xlabel('$t$','Interpreter','latex');
        ylabel('$\ddot{s}(t)$','Interpreter','latex')
        legend('x_{3,true}(t)','location','SE')
    linkaxes(ax,'x');


