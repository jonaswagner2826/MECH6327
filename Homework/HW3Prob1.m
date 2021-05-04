%% HW3, Problem 1
clc; clear; 


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

% dynamics block matrices
G = zeros(n*T,n);
H = eye(n*T);
BB = kron(eye(T),B);

for i=1:T
    G((i-1)*n+1:n*i,:) = A^i;
end

for i=1:T
    for j=1:T
        if i > j
            H((i-1)*n+1:n*i,(j-1)*n+1:n*j) = A^(i-j);
        end
    end
end

H = H*BB; 


cvx_begin
    variable x(n,T)
    variable u(m,T)
    minimize ( norm(x0, Inf) + sum(norms(x, Inf)) + gamma*sum(norms(u, Inf)) )
    subject to 
        vec(x) == G*x0 + H*vec(u)
        norms(x,inf) <= xbar
        norms(u,inf) <= ubar
cvx_end

%% plot
figure; 
plot(x', 'LineWidth', 2)
xlabel('time (s)');
ylabel('states');

figure;
plot(u', 'LineWidth', 2)
xlabel('time (s)');
ylabel('inputs');
