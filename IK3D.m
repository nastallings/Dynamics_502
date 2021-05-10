
%% geometric method

function [phi,k,theta] = IK3D(x, y, z)
% section 2.2 says how to handle singularities but sam is tired
% P  = (x,y,z) % desired point

% figure 2
 knum = 2*sqrt(x^2+y^2);
 kden = x^2+y^2+z^2;
 k = knum/kden;
 
 % the curvature in space
 theta = atan2(y,x);
 
 % i think z = zprime cuz we only rotate about z axis?
 zprime = z;
 % figure 3
 if zprime > 0
     phi = acos(1 -k*(sqrt(x^2+y^2))); % theta in paper
 else 
     phi = 2*pi - acos(1 -k*(sqrt(x^2+y^2)));  % theta in paper
 end

end


%% analytical method --> results in no solution for system of equations
function [phiSol, kSol] = anIK2D(x,y,z)
% solves for Phi and K, s is a constant  = 8.34
s = 8.34; % for segment A
syms phi k

%M2 =  forward kinematics T matrix where right column = posVector(x,y,z)
M2 = [cos(phi)*cos(s*k), -sin(phi), -cos(phi)*sin(s*k), (1/k)*(cos(phi) - cos(phi)*cos(s*k)); sin(phi)*cos(s*k), cos(phi), -sin(phi)*sin(s*k), -(1/k)*(sin(phi)*cos(s*k)); sin(s*k), 0, cos(s*k), (1/k)*sin(s*k);0, 0, 0, 1];
posV = [M2(1,4) ;M2(2,4); M2(3,4)]

% solve system of equations to find phi and k
equ1 = posV(1) == x
equ2 = posV(2) == y 
equ3 =posV(3) == z

sol = solve([equ1, equ2, equ3], [phi k]);
phiSol = sol.phi;
kSol = sol.k;

end