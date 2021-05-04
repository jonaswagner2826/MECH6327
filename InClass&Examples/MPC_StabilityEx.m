% MPC stability example

% problem data

n = 2; % state dimension
m = 1; % input dimension

% dynamics
A = [2 1; 0 0.5];
B = [0; 1];

% objective matrices
Q = eye(2);      
R = 30*eye(m);        

% state and control limits
Umax = 1;
Xmax = 10; % not used in this example

% initial state
% x0 = [-1.34; 1]; % unstable (T=5)
x0 = [-1.333333; 1]; % just barely stable (T=5)
% x0 = [-1.3; 1]; % stable (T=5)

% MPC parameters
T = 4; % time horizon

nsteps = 35;

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

for i = 1:nsteps
    i
    % solve open-loop optimization problem
    cvx_begin 
        variable X(n,T)
        variable U(m,T)
        minimize ( vec(X)'*kron(eye(T), Q)*vec(X) + vec(U)'*kron(eye(T), R)*vec(U) )
        subject to
            vec(X) == G*x + H*vec(U)  % dynamics equality constraints
            norms(U,inf) <= Umax  % input constraints
%             norms(X,inf) <= Xmax
    cvx_end
    
    % implement only first input in plan
    u = U(:,1);
    
    % record state, input, stage cost, and fmpc run time
    Xhist(:,i) = x; Uhist(:,i) = u;
    
    Jhist(i) = x'*Q*x+u'*R*u;
    % state update
    x = A*x+B*u
end

% plot trajectories
tvec = 1:nsteps; 
figure;
subplot(2,1,1); stairs(tvec,Xhist(1,1:nsteps),'LineWidth', 2);
set(gca, 'fontsize', 18); ylabel('state');
subplot(2,1,2); stairs(tvec,Uhist(1,1:nsteps),'LineWidth', 2, 'Color', [0.8500    0.3250    0.0980]);
set(gca, 'fontsize', 18);
xlabel('t'); ylabel('input');

% figure; 
% plot(Xhist(1,:),Xhist(2,:), 'LineWidth', 2);
% grid on;
