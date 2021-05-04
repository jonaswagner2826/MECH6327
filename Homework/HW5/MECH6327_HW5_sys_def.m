% MECH 6327 - Homework 5 - System Definition
% Author: Jonas Wagner
% Date: 2020-05-02

% Generates System Matricies and saves/displays ss of sys

%% System Definition -------------------------------------
% Cessna Citation Aircraft (at 5000m, at speed 128.2 m/s)
% Linearized Relative States:
% x1 = angle of attack
% x2 = pitch angle
% x3 = pitch rate
% x4 = altitude
% Input:
% u = pitch rate control
% w = pitch rate disturbance
% Output:
% y1 = angle of attck
% y2 = pitch angle
% y3 = pitch rate control


A =[-1.2822, 0, 0.98, 0;
    0, 0, 1, 0;
    -5.4923, 0, -1.8366, 0;
    -128.2, 128.2, 0, 0];
B =[-0.3; 0; -17; 0];
F = [0; 0; 1; 0];

C = [0, 1, 0, 0;
     0, 0, 0, 1;
     0, 0, 0, 0];
D = [0; 0; 1];

sys = ss(A,B,C,D)
