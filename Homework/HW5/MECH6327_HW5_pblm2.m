% MECH 6327 - Homework 5 - Problem 1
% Author: Jonas Wagner
% Date: 2020-05-02

clear;
close all;


lqr = true;
mpc = true;
runModels = true;
plotResults = true;

if runModels
%% System Setup
MECH6327_HW5_sys_def
n = size(A,1);
p = size(B,2);
q = size(C,1);

A_c = A;
B_c = B;
C_c = C;
D_c = D;
F_c = F;


% Discritization
dt = 0.25;
A = expm(A_c * dt);
B = dt*B_c + (1/2)*dt*A_c*B_c + (1/6)*(dt^2)*(A_c^2)*B_c;
C = C_c;
D = D_c;
sys = ss(A,B,C,D,dt)

% System Limitations
u_limit = 0.262; % rad (input limit)
u_rate_limit  = 0.524 * dt; % rad/s (Input Change Max
x2_limit = 0.349; % rad (objective)

%% Simulation Setup
N = 20;

w_power = 0.1;
W = w_power * randn(size(F,2),N);

x0 = [0;0;0;10];

if lqr
%% LQR Control Design
% LQR Design Parameters
Q = eye(n);
R = 10;

% Feedback gain calculation
[~,K_LQR,~] = idare(A,B,Q,R) % Prints out K

%% Simulate LQR Methods
% Feedback Gain
K = K_LQR;
% Simulation
X = zeros(n,N); % States
U = zeros(p,N); % Inputs
Y = zeros(q,N); % Outputs
U_sat = U;
% Initialization
x = x0;
U(:,1) = - K*x;
U_sat(:,1) = min(u_rate_limit, max(-u_rate_limit, U(:,1)));
%Assuming U(:,0) = 0
X(:,1) = A * x + B * U_sat(:,1) + F * W(:,1);
Y(:,1) = C * x + D * U_sat(:,1);
for i = 2:N
    % Control Calculation and saturation
    U(:,i) = - K*x;
    U_sat(:,i) = min(u_limit, max(-u_limit, U(:,i)));
    %Apparently this isn't what the limitation meant...
%     % Applied Control and Limitation
%     Bu = B * U_sat(:,i);
%     Bu(3) = min(x3_limit, max(-x3_limit, Bu(3)));
    % Elevator Angle Rate Max
    if (U_sat(:,i) - U_sat(:,i-1) >= u_rate_limit)
        U_sat(:,i) = U_sat(:,i-1) + u_rate_limit;
    elseif (U_sat(:,i) - U_sat(:,i-1) <= -u_rate_limit)
        U_sat(:,i) = U_sat(:,i-1)-u_rate_limit;
    end
    % Time Update
    x = A * x + B * U_sat(:,i) + F * W(:,i);
    y = C * x + D * U_sat(:,i);
    % Save Values
    X(:,i) = x;
    Y(:,i) = y;
end

% Save to LQR Values
X_LQR = X;
U_LQR = U;
U_sat_LQR = U_sat;

end

if mpc
%% MPC Control Design
% MPC Parameters
Q = eye(n);
R = 10;
T = 10; % Time Horizon
umax = u_limit;

% History Matrices
Xhist = zeros(n,N);  % States
Uhist = zeros(p,N);  % Inputs
Yhist = zeros(q,N);  % Outputs
Jhist = zeros(1,N);  % Stage costs

% Simulation setup
X = zeros(n,T);
U = zeros(p,T);
x = x0;

% Dynamics Matrices
G = zeros(n*T,n);
for i=1:T
    G((i-1)*n+1:n*i,:) = A^i;
end

H = eye(n*T);
for i=1:T
    for j=1:T
        if i > j
            H((i-1)*n+1:n*i,(j-1)*n+1:n*j) = A^(i-j);
        end
    end
end
BB = kron(eye(T),B);
H = H * BB;

% Simulation
tic;
disp('------------ MPC Method: ------------')
for i = 1:N
    u_last = U(:,1); %assumes at 0 at start
    disp(['Iteraton: ', num2str(i)]);
    % solve open-loop optimization problem
    cvx_begin quiet
        variable X(n,T) % predicted state trajectory
        variable U(p,T) % planned control actions
        minimize ((vec(X)' * kron(eye(T), Q)...
                    * vec(X) + vec(U)'*kron(eye(T), R)...
                    *vec(U)))
        subject to
            vec(X) == G * x + H * vec(U);  % System Dynamics
            norms(U,inf) <= umax; % Input Limitatiosn
            norm(U(:,1)- u_last ,inf) <= u_rate_limit;
            for j = 2:T
                norm(U(:,j)-U(:,j-1),inf) <= u_rate_limit;
            end
    cvx_end
    
    % Current Input and Output
    u = U(:,1);
    y = C * x + D * u;
    
    % Store Data
    Xhist(:,i) = x;
    Uhist(:,i) = u;
    Yhist(:,i) = y;
    Jhist(i) = x'*Q*x+u'*R*u;
    
    % State Update
    x = A*x + B*u + F * W(:,i);
end
MPC_runtime = toc

X_MPC = Xhist;
U_MPC = Uhist;
Y_MPC = Yhist;
J_MPC = Jhist;

end
end



if plotResults
%% Ploting

if lqr
% Plot LQR Plots
% Plot MPC Plots
figure('position',[0,0,1200,1000])
sgtitle('LQR Method System Response')
for i = 1:4
    subplot(2,2,i)
    stairs(X_LQR(i,:));
    hold on
    if i == 2
        plot(x2_limit*ones(N,1),'r')
        plot(-x2_limit*ones(N,1),'r')
    end
    title(['State X',num2str(i)])
end
saveas(gcf,[pwd,'\Homework\HW5\fig\pblm2_LQR_sys_response.png'])

figure('position',[0,0,1200,500])
sgtitle('LQR Method Control Signal')
subplot(1,2,1)
stairs(U_LQR')
hold on
stairs(U_sat_LQR')
plot(u_limit*ones(N,1),'r')
plot(-u_limit*ones(N,1),'r')
title('U')

subplot(1,2,2)
stairs(diff(U_LQR'))
hold on
stairs(diff(U_sat_LQR'))
plot(u_rate_limit*ones(N,1), 'r')
plot(-u_rate_limit*ones(N,1), 'r')
title('U Rate')
saveas(gcf,[pwd,'\Homework\HW5\fig\pblm2_LQR_ctrl_signal.png'])
end

if mpc
% Plot MPC Plots
figure('position',[0,0,1200,1000])
sgtitle(['MPC Method System Response T = ',num2str(T)])
for i = 1:4
    subplot(2,2,i)
    stairs(X_MPC(i,:));
    hold on
    if i == 2
        plot(x2_limit*ones(N,1),'r')
        plot(-x2_limit*ones(N,1),'r')
    end
    title(['State X',num2str(i)])
end
saveas(gcf,[pwd,'\Homework\HW5\fig\pblm2_MPC_T',num2str(T),...
    '_sys_response.png'])

figure('position',[0,0,1200,500])
sgtitle(['MPC Method Control Signal T = ',num2str(T)])
subplot(1,2,1)
stairs(U_MPC')
hold on
plot(u_limit*ones(N,1),'r')
plot(-u_limit*ones(N,1),'r')
title('U')


subplot(1,2,2)
stairs(diff(U_MPC'))
hold on
plot(u_rate_limit*ones(N,1), 'r')
plot(-u_rate_limit*ones(N,1), 'r')
title('U Rate')
saveas(gcf,[pwd,'\Homework\HW5\fig\pblm2_MPC_T',num2str(T),...
    '_ctrl_signal.png'])

end


end
