% Runs 100 steps of MPC on an example with 6 masses connected by springs

% problem data
% rand('state',0); % 
k = 1;          % spring constant 
lam = 0;        % damping constant 
a = -2*k;
b = -2*lam;
c = k;
d = lam;

n = 12; % state dimension
m = 3; % input dimension

% continuous time system matrices
Acts = [zeros(n/2) eye(n/2);
    [a,c,0,0,0,0,b,d,0,0,0,0;
     c,a,c,0,0,0,d,b,d,0,0,0;
     0,c,a,c,0,0,0,d,b,d,0,0;
     0,0,c,a,c,0,0,0,d,b,d,0;
     0,0,0,c,a,c,0,0,0,d,b,d;
     0,0,0,0,c,a,0,0,0,0,d,b]];

Bcts = [zeros(n/2,m);
    [1, 0, 0;
    -1, 0, 0;
     0, 1, 0;
     0, 0, 1;
     0,-1, 0;
     0, 0,-1]];

% convert to discrete-time system
ts = 0.5;       % sampling time
A = expm(ts*Acts);
B = (Acts\(expm(ts*Acts)-eye(n)))*Bcts;

% objective matrices
Q = eye(n);      
R = eye(m);        

% state and control limits
Umax = 0.5;
umin = -Umax*ones(m,1);
umax = Umax*ones(m,1);
% Xmax = 4; 
% xmin = -Xmax*ones(n,1);
% xmax = Xmax*ones(n,1);

% disturbance trajectory
nsteps = 100;
w = 2*rand(n,nsteps)-1; 
w(1:n/2,:) = 0; 
w = 0.5*w;

% initial state
x0 = randn(n,1);

% MPC parameters
T = 10; % time horizon

% allocate history matrices
Xhist = zeros(n,nsteps);  % state
Uhist = zeros(m,nsteps);  % input
Jhist = zeros(1,nsteps);  % stage cost

% set up initial state and input trajectories
X = zeros(n,T);
U = zeros(m,T);
x = x0;

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

tic;
for i = 1:nsteps
    i
    % solve open-loop optimization problem
    cvx_begin quiet
        variable X(n,T) % predicted state trajectory
        variable U(m,T) % planned control actions
        minimize ( vec(X)'*kron(eye(T), Q)*vec(X) + vec(U)'*kron(eye(T), R)*vec(U) )
        subject to
            vec(X) == G*x + H*vec(U)  % dynamics equality constraints, x is current state
            norms(U,inf) <= Umax  % input constraints
    cvx_end
    
    % implement only first input in plan
    u = U(:,1);
    
    % record state, input, stage cost, and mpc run time
    Xhist(:,i) = x; Uhist(:,i) = u;
    Jhist(i) = x'*Q*x+u'*R*u;
    
    % state update
    x = A*x+B*u+w(:,i);
end
t = toc;

% plot trajectories for x_1 and u_1
tvec = 1:100; figure;
subplot(2,1,1); stairs(tvec,Xhist(1,1:100),'LineWidth', 2);
set(gca, 'fontsize', 18);
axis([0,100,-2,2]); ylabel('x_1');
subplot(2,1,2); stairs(tvec,Uhist(1,1:100),'LineWidth', 2, 'Color', [0.8500    0.3250    0.0980]);
set(gca, 'fontsize', 18);
axis([0,100,-0.6,0.6]);
xlabel('t'); ylabel('u_1');