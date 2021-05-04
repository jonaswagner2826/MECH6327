%% MECH6327: Convex Optimization in Systems and Control
% examples and code for convex optimization problems


%% LPs

% linfinity (Chebyshev) minimization
n = 10;
m = 100;

A = rand(m,n);
b = rand(m,1) - 1;

cvx_begin
    variable x(n)
    variable t
    minimize t
    subject to
        -ones(n,1)*t <= x <= ones(n,1)*t
        A*x <= b
cvx_end

x1 = x;

cvx_begin
    variable x(n)
    minimize norm(x, inf)
    subject to
        A*x <= b
cvx_end

x2 = x;

[x1 x2]

%% l1 minimization
n = 10;
m = 30;
p = 50;

A = randn(m,n);
b = rand(m,1);
F = randn(p,n);
g = rand(p,1);

cvx_begin
    variable x(n)
    variable t(m)
    minimize (ones(m,1)'*t)
    subject to
        -t <= A*x - b <= t
        F*x <= g
cvx_end

x1 = x;

cvx_begin
    variable x(n)
    minimize norm(A*x - b, 1)
    subject to
        F*x <= g
cvx_end

x2 = x;

[x1 x2]

%% QPs
n = 10;

P = randn(n); P = P'*P;
q = randn(n,1);

cvx_begin
    variable x(n)
    minimize (x'*P*x + q'*x)
    subject to
        A*x <= b
cvx_end


%% QCQPs
P = randn(n); P = P'*P;
q = randn(n,1);
P1 = randn(n); P1 = P1'*P1;
q1 = randn(n,1);

cvx_begin
    variable x(n)
    minimize (x'*P*x + q'*x)
    subject to
        x'*P*x + q'*x <= 1
        A*x <= b
cvx_end



%% SOCPs
% robust LP (with ellipsoidal uncertainty set)
n = 2;
m = 5;

c = randn(n,1);
A = randn(m,n);
b = rand(m,1);

P = 0.05*randn(n);

cvx_begin
    variable x(n)
    minimize (c'*x)
    subject to
        A(1,:)*x + norm(P*x,2) <= b(1)
        A(2:end,:)*x <= 10*b(2:end)
cvx_end

%% validate
u = randn(n,1); u = u/norm(u);
(A(1,:) + u'*P')*x - b(1)

% for comparison
(A(3,:) + u'*P')*x - b(3)


%% SDPs
n = 10;
m = 2;

A = randn(n);
B = randn(n,m);

cvx_begin sdp
    variable P(n,n) symmetric
    variable L(m,n)
    minimize (0*trace(P))
    subject to
       [P P*A' + L'*B';
        A*P + B*L P] >= 0.001*eye(2*n)
        P >= 0
cvx_end


K = L*inv(P);
[abs(eig(A)) abs(eig(A + B*K))]
