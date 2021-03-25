% MECH 6327 - Homework 3 - Problem 1
% Author: Jonas Wagner
% Date: 2020-03-21

clear
close all

%% Problem Data
clear
HW3Prob1_Data

%% Norm Inf ------------------------------
% Derived Linear Program
cvx_begin
    variable r(T, 1)
    variable s(T-1, 1)
    variable x(n,T)
    variable u(m,T-1)
    sum = 0;
    for i = 1:T-1
        sum = sum + r(i) + gamma * s(i);
    end
    minimize(r(T) + sum);
    subject to
        x(:,1) == x0;
        for i = 1:T-1
            x(:,i+1) == A * x(:,i) + B * u(:,i);
        end
        for i = 1:T
            r(i) <= xbar;
            -r(i) <= xbar;
        end
        for i = 1:T-1
            s(i) <= ubar;
            -s(i) <= ubar;
        end
        for i = 1:T
            for j = 1:n
                x(j,i) <= r(i);
                -x(j,i) <= r(i);
            end
        end
        for i = 1:T-1
            for j = 1:m
                u(j,i) <= s(i);
                -u(j,i) <= s(i);
            end
        end
cvx_end

x_inf_lp = x;
u_inf_lp = u;


% CVX Norm Implimentation
p = Inf;
q = Inf;
cvx_begin
    variable x(n,T)
    variable u(m,T-1)
    sum = norm(x(:,T),p);
    for i = 1:T-1
        sum = sum + norm(x(:,i),p) + gamma * norm(u(:,i),q);
    end
    minimize(sum)
    subject to
        x(:,1) == x0;
        for i = 1:T-1
            x(:,i+1) == A * x(:,i) + B * u(:,i);
        end
        norm(x(:,i),Inf) <= xbar;
        for i = 1:T-1
            norm(x(:,i),Inf) <= xbar;
            norm(u(:,i),Inf) <= ubar;
        end
cvx_end

x_inf_norm = x;
u_inf_norm = u;


%% \infty-norm Ploting
fig = figure('position', [0, 0, 1200, 1000])
sgtitle('States for Open-Loop Optimal Control with \infty - norm')
for i = 1:n
    subplot(ceil(n/3),3,i)
    plot(x_inf_lp(i,:))
    hold on
    plot(x_inf_norm(i,:))
    legend
end
saveas(fig,fullfile([pwd '\\' 'Homework' '\\' 'HW3' '\\' 'fig'],'pblm1_inftyn_x.png'))

fig = figure('position', [0, 0, 750, 500])
sgtitle('Control Sequence for Open-Loop Optimal Control with \infty - norm')
for i = 1:m
    subplot(3,1,i)
    plot(u_inf_lp(i,:))
    hold on
    plot(u_inf_norm(i,:))
    legend
end
saveas(fig,fullfile([pwd '\\' 'Homework' '\\' 'HW3' '\\' 'fig'],'pblm1_inftyn_u.png'))


%% Norm 1 ------------------------------
% Derived Linear Program
cvx_begin
    variable r(n,T)
    variable s(m,T-1)
    variable x(n,T)
    variable u(m,T-1)
    sum = 0;
    for i = 1:T-1
        for j = 1:n
            sum = sum + r(j,i);
        end
        for j = 1:m
            sum = sum + gamma * s(j,i);
        end
    end
    minimize(r(T) + sum);
    subject to
        x(:,1) == x0;
        for i = 1:T-1
            x(:,i+1) == A * x(:,i) + B * u(:,i);
        end
        -xbar <= x(:,:) <= xbar;
        -ubar <= u(:,:) <= ubar;
        -r <= x <= r;
        -s <= u <= s;
cvx_end

x_1_lp = x;
u_1_lp = u;


% CVX Norm Implimentation
p = 1;
q = 1;
cvx_begin
    variable x(n,T)
    variable u(m,T-1)
    sum = norm(x(:,T),p);
    for i = 1:T-1
        sum = sum + norm(x(:,i),p) + gamma * norm(u(:,i),q);
    end
    minimize(sum)
    subject to
        x(:,1) == x0;
        for i = 1:T-1
            x(:,i+1) == A * x(:,i) + B * u(:,i);
        end
        norm(x(:,i),Inf) <= xbar;
        for i = 1:T-1
            norm(x(:,i),Inf) <= xbar;
            norm(u(:,i),Inf) <= ubar;
        end
cvx_end

x_1_norm = x;
u_1_norm = u;




%% 1-norm Ploting
fig = figure('position', [0, 0, 1200, 1000])
sgtitle('States for Open-Loop Optimal Control with 1 - norm')
for i = 1:n
    subplot(ceil(n/3),3,i)
    plot(x_1_lp(i,:))
    hold on
    plot(x_1_norm(i,:))
    legend
end
saveas(fig,fullfile([pwd '\\' 'Homework' '\\' 'HW3' '\\' 'fig'],'pblm1_1n_x.png'))

fig = figure('position', [0, 0, 750, 500])
sgtitle('Control Sequence for Open-Loop Optimal Control with 1 - norm')
for i = 1:m
    subplot(3,1,i)
    plot(u_1_lp(i,:))
    hold on
    plot(u_1_norm(i,:))
    legend
end
saveas(fig,fullfile([pwd '\\' 'Homework' '\\' 'HW3' '\\' 'fig'],'pblm1_1n_u.png'))