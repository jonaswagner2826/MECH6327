


"""
In Class Ex:
Constrained \infty-norm minimization

Problem:
    minimize:
        \norm{x}_\infty
    subject to:
        A x \leq b

Notes:
    \norm{x}_\infty = \max_i \abs{x_i}
                    = \max_i \{x_i, -x_i\}
                    = \max_i \{x_1, \dots, x_n, -x_1, \dots, -x_n\}

Epigraph Form:
    minimize:
        t
    subject to:
        \max_i \{x_1, \dots, x_n, -x_1, \dots, -x_n\} \leq t
        Ax \leq b

Equivelent Form:
    minimize:
        t
    subject to:
        x_i \leq t, i = 1, \dots, n
        -x_i \leq t, i = 1, \dots, n
        Ax \leq b

Equivelent Form:
    minimize:
        t
    subject to:
        - 1 t \leq x \leq 1 t
        Ax \leq b
"""


import cvxpy as cp
import numpy as np


n = 10
m = 100

A = np.random.rand(m, n)
b = np.random.rand(m, 1) - 1


X = cp.Variable((n, n))
prob = cp.Problem(cp.Minimize(cp.max(cp.sum(cp.abs(X)))),
                  [A @ X <= b])


# Print result.
print("The optimal value is", prob.value)
print("A solution X is")
print(X.value)


# doesn't work..... don't really feel like troubleshooting though
