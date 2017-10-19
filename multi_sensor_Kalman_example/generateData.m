function data = generateData()
% data = generateData();
% generates data struct
% Output:
%   data,   struct with all relevant signals  
% Samuel Nyffenegger, 19.10.17

%% calculations
clc;

% load data
K = param.K;
Ts = param.Ts;
x0 = param.x0;
P0 = param.P0;
Q = param.Q;
R = param.R;
Phi = param.Phi;
Gamma = param.Gamma; 
H = param.H;

% prepare arrays to store data
x_true = zeros(3,K);                % true states
y = zeros(3,K);                     % measurements
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
    y(:,k) = [H(1,:)*x_true(:,k) + v(1) ; ...
              H(2,:)*x_true(:,k) + v(2) ; ...
              H(3,:)*x_true(:,k) + v(3) ];
    
end

% prepare output
data.x_true = x_true;
data.x0_true = x0_true;
data.y = y;
data.t = t;

end