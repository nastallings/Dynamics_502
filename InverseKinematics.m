 
% sams IK method from 
% Closed-Form Inverse Kinematics for Continuum Manipulators Paper
% Mainly Pages 2081 & 2082

% NOTES:

% "move to a given curvature k and direction of curvature phi"
% "extend to trunk length s (s is constant for us)
% r = radius of arc
% k = 1 / r 



% example: P = 2,2,2 // segment length = 5 doesnt matter?
[phi1, k1] = genValues(2,2,2)

% INPUT:   x, y, z - units of length
% Known:   s (known constant for us) =  arc length - units of length Q: SAME AS SEGMENT LENGTH?
% OUTPUTS: phi = direction of curvature - units of radians
%          k = the curvature - units of  1/length  
function [phi,k] = genValues(x, y, z)
% section 2.2 says how to handle singularities but sam is tired
% P  = (x,y,z) % desired point

% figure 2
 knum = 2*sqrt(x^2+y^2);
 kden = x^2+y^2+z^2;
 k = knum/kden;
 
 phi = atan2(y,x);
 
 % i think z = zprime cuz we only rotate about z axis?
 zprime = z;
 % figure 3
 if zprime > 0
     theta = arccos(1 -k*(sqrt(x^2-y^2)));
 else 
     theta = 2*pi - arccos(1 -k*(sqrt(x^2-y^2)));  
 end

end

% With the list of desired endpoints for each section
%   1. subtract the translation due to the base section from
%      the other desired endpoints
%   2. Apply opposite rotation due to the base section
%      to the remaining sections
%      Rotation due to a single section 
%      w = [ -sin(phi) cos(phi) 0]' by the angle theta
%      Adjusted endpoint values = R(w,-theta)*(Pnext - Pcurrent)
%      where Pcurrent is the endpoint of the section whose K, phi
%      values are being computer, and Pnext is the endpoint of the
%      remaining section.
% 
% We have no deadlength sections in this model
function addSegs()
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
