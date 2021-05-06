 
%% Sams Grillo:
% IK method from "Closed-Form Inverse Kinematics for Continuum Manipulators" Paper
% Content from Pages 2081 & 2082


%% example: x = 2, y = 2, z = 2
[theta1, phi1, k1] = IK3D(2,2,2)
[phi2, k2] = IKnoCurve(3,4,1)

%% 3d Inverse Kinematics:
% This function geometrically calculates the inverse kinematics
% for a 3D model by including theta as an output

% INPUT:   x, y, z - units of length
% Known:   s (known constant for us) =  arc length - units of length Q: SAME AS SEGMENT LENGTH?
% OUTPUTS: theta = direction of curvature - units of radians
%          k = the curvature - units of  1/length 
%          phi = the angle that helps define curvature

function [theta,phi,k] = IK3D(x, y, z)
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

%% Inverse Kinematics - Without Direction of Curvature:
% This function geometrically calculates the inverse kinematics
% for a simplified model by removing theta as an output

% INPUTS: (x,y,z) - coordinates of desired point
% Known:   s (known constant for us) =  arc length - units of length Q: SAME AS SEGMENT LENGTH?
% OUTPUTS: k = the curvature - units of  1/length  
%          phi = the angle that helps define curvature

function [phi,k] = IKnoCurve(x, y, z)

% from figure 2 in paper
 knum = 2*sqrt(x^2+y^2);
 kden = x^2+y^2;
 k = knum/kden;

 % i think z = zprime cuz we only rotate about z axis?
 zprime = z;
 % figure 3
 if zprime > 0
     phi = acos(1 -k*(sqrt(x^2+y^2))); % theta in paper
 else 
     phi = 2*pi - acos(1 -k*(sqrt(x^2+y^2)));  % theta in paper
 end

end

%% Draft Functions: Not actively used

% The following code can be updated to combine
% multiple segments

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

% INPUT the two sets of x y z points, c = current, n = next
% outputs actual pn once the points are shifted by

function pNext = addSegs(cX, cY, cZ, nX, nY, nZ)
pc = [cX, cY, cZ];
pn = [nX, nY, nZ];

pNext = pn - pc;
% apply rotation somehow

%axis in which the segment rotates around
     % w = [ -sin(phi) cos(phi) 0]'; %plug in -theta
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
