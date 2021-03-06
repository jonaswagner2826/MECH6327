% MECH 6327 - Homework 3 - Problem 3
% Author: Jonas Wagner
% Date: 2020-03-21

clear

%% Random Problem Generation
n = 4;
m = 2;
A = randn(n,n)
B = randn(n,m)
Q = randPDMatrix(n)
R = randPDMatrix(m)

%% Optimiztation SDP Solution
cvx_begin sdp
    variable P(n,n) symmetric
    minimize(-trace(P))
    subject to
        M = [R + B' * P * B,    B' * P * A;
         A' * P * B,        Q + A' * P * A - P] >= 0;
        P >= 0;
cvx_end

P_cvx = P
K_cvx = - inv(R + B'*P*B)*B'*P*A
%% DARE Function

[P_dare,K_dare] = idare(A,B,Q,R)

P_cvx
P_dare




%% Random Matrix Generation
% Random Symetric Matrix
function S = randSMatrix(n)
    A = randn(n,n);
    S = (A + A')/2;
end
% Random Semi-ositive Definite Matrix
function SPD = randSPDMatrix(n)
    A = randn(n-1,n);
    SPD = A' * A;
end
% Random Positive Definite Matrix
function PD = randPDMatrix(n)
    A = rand(n,n);
    PD = A' * A;
%     S = randSMatrix(n);
%     PD = S + n * eye(n);
end