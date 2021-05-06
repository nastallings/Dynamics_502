function [aphi,ak,atheta] = trajectory(Qi,te)
syms a0 a1 a2 a3
% Qe = [phiaf,kaf,phibf,kbf,phicf,kcf]
p = [1, 0, 0, 0;
     0, 1, 0, 0;
     1, te, te^2, te^3;
     0, 1, 2*te, 3*te^2];
ap = [a0; a1; a2; a3];
q1 = [0; 0; Qi(1); 0];
s1 = solve(p*ap==q1,ap);
aphi = double([s1.a0;s1.a1;s1.a2;s1.a3]);
q2 = [0; 0; Qi(2); 0];
s2 = solve(p*ap==q2,ap);
ak = double([s2.a0;s2.a1;s2.a2;s2.a3]);
q3 = [0; 0; Qi(3); 0];
s3 = solve(p*ap==q3,ap);
atheta = double([s3.a0;s3.a1;s3.a2;s3.a3]);
end
