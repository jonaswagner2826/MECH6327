%MATLAB-YALMIP example implementation of the H2-norm LMI
clc;clear all;close all;

% You need YALMIP toolbox to run this code
%% Setup matrices for this example
A = [0 0 1 0;
     0 0 0 1;
     -0.101 -0.1681 -0.04564 -0.01075;
     0.06082 -2.1407 -0.05578 -0.1273];
B = [0 0 0;
     0 0 0;
     0.1179 0.1441 0.1478;
     0.1441 1.7057 -0.7557];
C = [1 0 0 0;
     0 1 0 0];

%% Set variables to be solved
X = sdpvar(size(A,1));
sdpvar gamma;

% Define the constraints
Constraints = [X>=1e-5*eye(size(A,1));A*X+X*A'+B*B'<=0];
Constraints = [Constraints;trace(C*X*C')<=gamma];

% Solve the problem
optimize(Constraints,gamma);

% Extract the minimum H2 norm
gamma = value(gamma)

% Solution that attains the minimum H2 norm
X = value(X)