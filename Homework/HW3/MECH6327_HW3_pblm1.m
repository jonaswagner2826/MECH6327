% MECH 6327 - Homework 3 - Problem 1
% Author: Jonas Wagner
% Date: 2020-03-21

%% Problem Data
HW3Prob1_Data


%% Norm Inf ------------------------------
% Derived Linear Program
% cvx_begin
%     variable r(T, 1)
%     variable s(T, 1)
%     variable x(n,T)
%     variable u(m,T)
%     sum = 0;
%     for i = 1:T-1
%         sum = sum + r(i) + gamma * s(i);
%     end
%     minimize(r(T) + sum)
%     subject to
%         x(:,1) == x0;
%         for i = 1:T-1
%             x(i+1) == A * x(:,i) + B * u(:,i);
%         end
%         for i = 1:T
%             x(i) <= r(i) * ones(n,1);
% %             x(i) >= r(i) * ones(n,1);
%             r(i) <= xbar;
%             r(i) >= -xbar;
%             u(i) <= s(i) * ones(m,1);
%             s(i) <= ubar;
% %             u(i) >= s(i) * ones(m,1);
%             s(i) >= -ubar;
%         end
% cvx_end




% CVX Norm Implimentation
p = Inf;
q = Inf;
cvx_begin
    variable x(n,T)
    variable u(m,T-1)
    sum = norm(x(:,T),p);
    for i = 1:T-1
        sum = sum + norm(x(:,i),p) + norm(u(:,i),q);
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




% %% Norm 1 ------------------------------
% % Derived Linear Program
% cvx_begin
%     variable r(T, 1)
%     variable s(T-1, 1)
%     variable x(n,T)
%     variable u(m,T)
%     sum = 0;
%     for i = 1:T-1
%         r(i) + gamma * s(i);
%     end
%     minimize(r(T) + sum)
%     subject to
%         for i = 1:T-1
%             x(i+1) == A * x(:,i) + B * u(:,i);
%         end
%         for i = 1:T
%             x(i) <= r(i);
%             r(i) <= xbar;
%         end
%         for i = 1:T-1
%             u(i) <= s(i);
%             s(i) <= ubar;
%         end
% cvx_end
% 
% x_lp = x;
% u_lp = u;
% 
% % CVX Norm Implimentation
% p = Inf;
% q = Inf;
% cvx_begin
%     variable x(n,T)
%     variable u(m,T)
%     sum = 0;
%     for i = 1:T-1
%         sum = norm(x(:,i),p) + norm(u(:,i),q);
%     end
%     minimize(norm(x(:,T),p) + sum)
%     for i = 1:T-1
%         x(i+1) == A * x(:,i) + B * u(:,i);
%     end
%     for i = 1:T
%         norm(x(:,i),Inf) <= xbar;
%         norm(u(:,i),Inf) <= ubar;
%     end
% cvx_end