function x = distr_KF_corr(y)
% ();
% Kalman Filter
% Input:
%   
% Samuel Nyffenegger, 19.10.17

%% calculations

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

% array to store estimated state
x = zeros(3,K);

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
        Kk = (H(l,:)*Pm*H(l,:)' + R(l,l))\Pm*H(l,:)';
        xm = xm + Kk*(y(l,k)-H(l,:)*xm);
        Pm = (eye(3)-Kk*H(l,:))*Pm*(eye(3)-Kk*H(l,:))'+Kk*R(l,l)*Kk';        
    end
    
    % store
    x(:,k) = xm;
end

end