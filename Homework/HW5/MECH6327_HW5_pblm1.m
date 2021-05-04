% MECH 6327 - Homework 5 - Problem 1
% Author: Jonas Wagner
% Date: 2020-05-02

clc;
clear;
close all;

h2norm = true;
hinftynorm = true;

%% System Setup
MECH6327_HW5_sys_def
n = size(A,1);
p = size(B,2);
q = size(C,1);

if h2norm
%% H-2 Norm Control
tol = 1e-6;
cvx_begin sdp
    variable X(n,n) symmetric
    variable W(q,q) symmetric
    variable L(1,n)
    variable Gamma
    minimize Gamma
    subject to
        [W, C*X+D*L;
         X*C'+L'*D', X] >= 0
        A*X + X*A' + B*L + L'*B' + F*F' <= 0
        trace(W) <= Gamma - tol
        X >= tol * eye(n)
        W >= tol * eye(q)
cvx_end

K_H2 = L*inv(X)
Norm_H2 = sqrt(Gamma)
end


if hinftynorm
%% H-infty Norm Control
tol = 1e-6;
cvx_begin sdp
    variable Q(n,n) symmetric
    variable L(p,n)
    variable eta
    
    minimize eta
    subject to 
        [Q*A'+L'*B'+A*Q+B*L F Q*C'+L'*D';
         F'  -eta zeros(p,q);
         C*Q+D*L zeros(q,p) -eye(q)] <= -tol*eye(n+q+1)
        Q >= tol*eye(n)
cvx_end


K_Hinfty = L*inv(Q)
Norm_Hinfty = sqrt(eta)
end