% MECH 6327 - HW 4

x = sdpvar(1,1);
y = sdpvar(1,1);
p = x^2 * y^4 + x^4 * y^2 + 1 - 3 * x^2 * y^2;
F = sos(p);
solvesos(F);
sdisplay(sosd(F))


% Results:
% ----------------------------
% -------------------------------------------------------------------------
% YALMIP SOS module started...
% -------------------------------------------------------------------------
% Detected 0 parametric variables and 2 independent variables.
% Detected 0 linear inequalities, 0 equality constraints and 0 LMIs.
% Using kernel representation (options.sos.model=1).
% Initially 8 monomials in R^2
% Newton polytope (2 LPs).........Keeping 4 monomials (0.20313sec)
% Finding symmetries..............Found 3 symmetries  (0sec)
% Partitioning using symmetry.....1x1(4) 
%  
% 
% Problem is unbounded.
% 
%  
% -> Solver reported unboundness of the dual problem.
% -> Your SOS problem is probably infeasible (SOS is dualized).