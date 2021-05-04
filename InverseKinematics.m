 
% sams IK method from a paper
% example: P = 2,2,2 // segment length = 5 doesnt matter?
[phi1, k1] = genIK(2,2,2)

% INPUT:   x, y, z - units of length
% Known:   s (known constant) =  arc length - units of length Q: SAME AS SEGMENT LENGTH?

% OUTPUTS: phi = direction of curvature - units of radians
%          k = the curvature - units of  1/length  
function [phi,k] = genIK(x, y, z)
% section 2.2 says how to handle singularities but sam is tired
% P  = (x,y,z) % desired point

% figure 2
 knum = 2*sqrt(x^2+y^2);
 kden = x^2+y^2+z^2;
 k = knum/kden;
 
 phi = atan2(y,x);
 
 % FIGURE OUT WHY WE CALC THETA
%  % figure 3
%  if zprime > 0:
%      theta = arccos(1 -k*(sqrt(x^2-y^2)));
%  else 
%      theta = 2*pi - arccos(1 -k*(sqrt(x^2-y^2)));
    
end

% Kehan's function that follows standard calculations
function qi = InverseKin(pd,J,T03)
    % This function is a IK that used Jacobian method
    % input pd is the desired point
    % J is the Jv jacobian matrix
    % T03 is the transformation matrix
    % The error var is set to 0.01 to help faster calculate
    
    % define the symbols that are going to be used in J and T matrixes.
    syms t1 t2 t3
    qi = [0;0;0];
    % Define the error of the results we are going to have
    er = 0.01;
    % Functionlize the symbol matrix
    f1(t1,t2,t3) = T03;
    f2(t1,t2,t3) = J;
    p0 = double(f1(qi(1),qi(2),qi(3)));
    p0 = p0(1:3,4);
    pn = pd - p0;
    % Keep fitting the P0 until it's the Qi we want
    while norm(pn) > er
        j = pinv(double(f2(qi(1),qi(2),qi(3))));     
        delta = j * pn;
        qi = qi + delta;
        p0 = double(f1(qi(1),qi(2),qi(3)));
        p0 = p0(1:3,4);
        pn = pd - p0;
    end
    disp(qi)
   
end
