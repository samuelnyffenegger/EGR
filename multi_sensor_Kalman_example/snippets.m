%%
clc
N = 1e5;
Q = diag([0,0,0]);
v = mvnrnd(ones(3,1),Q,N)';
v_mean = mean(v')'
v_var = var(v')'

%%
param.


