 
%% Sams Grillo:
% IK method from "Closed-Form Inverse Kinematics for Continuum Manipulators" Paper
% Content from Pages 2081 & 2082

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
