% MECH 6327 - Homework 3 - Problem 2
% Author: Jonas Wagner
% Date: 2020-03-21

%% Problem Data
clear
close all
HW3Prob2_Data

%% Bisection Attempt
lower = 0;
upper = 100;
error = 1;

umax = [umax1; umax2];

t_all = [];

while (upper - lower) > error
    T = upper;
    t = floor(1/2 * (lower + upper));
    cvx_begin
        variable x(n,T)
        variable u(m,T)
        x(:,1) == x0;
        for i = 1:T-1
            x(:,i+1) == A * x(:,i) + B * u(:,i);
        end
        for i = 1:T
            -umax <= u(i) <= umax;
        end
        x(:,t) == xdes;
    cvx_end
    if abs(cvx_optval) <= 1
        upper = t;
    else
        lower = t;
    end
    t_all = [t_all,t];
end

if abs(cvx_optval) <= 1
    tau = t * ts;
    T = t;
else
    tau = (t-1) * ts;
    T = t-1;
end

cvx_begin
    variable x(n,T)
    variable u(m,T)
    x(:,1) == x0;
    for i = 1:T-1
        x(:,i+1) == A * x(:,i) + B * u(:,i);
    end
    for i = 1:T
        -umax <= u(i) <= umax;
    end
    x(:,t) == xdes;
cvx_end

t_all

tau

%% Ploting
t = ts * T;

fig = figure('position',[0,0,750,500]);
subplot(2,1,1)
for i = 1:n
    plot(x(i,:))
    hold on
end
title('States for minimum time state transfer via quasiconvex optimization')
legend
% saveas(fig,fullfile([pwd '\\' 'Homework' '\\' 'HW3' '\\' 'fig'],'pblm2_x.png'))
% 
% fig = figure
subplot(2,1,2)
for i = 1:m
    plot(u(i,:))
    hold on
end
title('Control Sequence for minimum time state transfer via quasiconvex optimization')
legend
saveas(fig,fullfile([pwd '\\' 'Homework' '\\' 'HW3' '\\' 'fig'],'pblm2.png'))