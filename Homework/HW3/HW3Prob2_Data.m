%% HW3, Problem 2

%% problem data
k = 1;          % spring constant 
lam = 0;        % damping constant 
a = -2*k;
b = -2*lam;
c = k;
d = lam;

n = 6; % state dimension
m = 2; % input dimension

% system of 6 masses connected by springs and dampers
Acts = [zeros(n/2) eye(n/2);
       diag(ones(n/2,1)*a) + diag(ones(n/2-1,1)*c,1) + diag(ones(n/2-1,1)*c,-1) diag(ones(n/2,1)*b) + diag(ones(n/2-1,1)*d,1) + diag(ones(n/2-1,1)*d,-1)];

Bcts = [zeros(n/2,m);
    [1, 0;
     0, 1;
     0, -1]];

% convert to discrete-time system
ts = 0.2;       % sampling time
A = expm(ts*Acts);
B = (Acts\(expm(ts*Acts)-eye(n)))*Bcts;

x0 = [0; 0; 0; 0; 0; 0];
umax1 = 1;
umax2 = 0.1; % this means \underbar u = [-umax1;-umax2]  <= [u1;u2]  <= [+umax1;+umax2] = \overbar u
xdes = [2; 2; 2; 0; 0; 0];