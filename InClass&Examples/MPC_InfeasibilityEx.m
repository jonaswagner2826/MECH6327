% MPC Infeasibility example

% problem data

n = 2; % state dimension
m = 1; % input dimension

% dynamics
A = [1 1; 0 1];
B = [0; 1];

% objective matrices
Q = eye(2);  
R = 30*eye(m);        

% state and control limits
Umax = 0.5;
Xmax = 5;

% initial state
% x0 = [-4; 3]; % not recursively feasible (T=4)
x0 = 0.892*[-4; 3]; % just recursively feasible (T=4)

% MPC parameters
T = 3; % time horizon

nsteps = 25;

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
    cvx_begin quiet
        variable X(n,T)
        variable U(m,T)
        minimize ( vec(X)'*kron(eye(T), Q)*vec(X) + vec(U)'*kron(eye(T), R)*vec(U) )
        subject to
            vec(X) == G*x + H*vec(U)  % dynamics equality constraints
            norms(U,inf) <= Umax  % input constraints
            norms(X,inf) <= Xmax
    cvx_end
    
%     figure; plot(X(1,:), X(2,:), 'LineWidth', 2); grid on;
    
    % implement only first input in plan
    u = U(:,1);
    
    % record state, input, stage cost, and fmpc run time
    Xhist(:,i) = x; Uhist(:,i) = u;
    
    Jhist(i) = x'*Q*x+u'*R*u;
    % state update
    x = A*x+B*u;
end

%% plot trajectories
tvec = 1:nsteps; 
figure;
subplot(2,1,1); stairs(tvec,Xhist(1,1:nsteps),'LineWidth', 2);
set(gca, 'fontsize', 18); ylabel('state');
subplot(2,1,2); stairs(tvec,Uhist(1,1:nsteps),'LineWidth', 2, 'Color', [0.8500    0.3250    0.0980]);
set(gca, 'fontsize', 18);
xlabel('t'); ylabel('input');

figure; 
plot(Xhist(1,:),Xhist(2,:), 'LineWidth', 2);
grid on;
