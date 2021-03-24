%% HW3, Problem 1

%% problem data
k = 1;          % spring constant 
lam = 0;        % damping constant 
a = -2*k;
b = -2*lam;
c = k;
d = lam;

n = 12; % state dimension
m = 3; % input dimension

% system of 6 masses connected by springs and dampers
Acts = [zeros(n/2) eye(n/2);
       diag(ones(n/2,1)*a) + diag(ones(n/2-1,1)*c,1) + diag(ones(n/2-1,1)*c,-1) diag(ones(n/2,1)*b) + diag(ones(n/2-1,1)*d,1) + diag(ones(n/2-1,1)*d,-1)];
%     [a,c,0,0,0,0,b,d,0,0,0,0;
%      c,a,c,0,0,0,d,b,d,0,0,0;
%      0,c,a,c,0,0,0,d,b,d,0,0;
%      0,0,c,a,c,0,0,0,d,b,d,0;
%      0,0,0,c,a,c,0,0,0,d,b,d;
%      0,0,0,0,c,a,0,0,0,0,d,b]];

Bcts = [zeros(n/2,m);
    [1, 0, 0;
    -1, 0, 0;
     0, 1, 0;
     0, 0, 1;
     0,-1, 0;
     0, 0,-1]];

% convert to discrete-time system
ts = 0.2;       % sampling time
A = expm(ts*Acts);
B = (Acts\(expm(ts*Acts)-eye(n)))*Bcts;

T = 60;
x0 = randn(n,1);
gamma = 0.1;
xbar = 5;
ubar = 2;