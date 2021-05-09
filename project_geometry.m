syms theta r1 r2 r3

x = 0.1;
s2 = 5;
deltas = 1;
s1 = s2 + deltas;
s3 = s2 - deltas;

eqn1 = theta == s1 / r1;
eqn2 = theta == s2 / r2;
eqn3 = theta == s3 / r3;

eqn1 = s1 == r1*theta;
eqn2 = s2 == r2*theta;
eqn3 = s3 == r3*theta;
eqn4 = r1 == r2 + x;
eqn5 = r3 == r2 - x;

[A,B] = equationsToMatrix([eqn1, eqn2, eqn3, eqn4, eqn5]); %, [theta, r1, r2, r3]);
X = linsolve(A,B)
