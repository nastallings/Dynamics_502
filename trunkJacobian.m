
% NOTE: I thought I was calculating the inverse kinematics
% but the paper tricked me so this is like part of an 
% alternative way to calculate the jacobian we already have

% generate matrix for first joint
% INPUT: x, y - units of length
% INPUT: phi - units of radians
%        k = the curvature - units of  1/length
% INPUT  l =  arc length - units of length Q: SAME AS SEGMENT LENGTH?
% OUTPUT the filled in matrix template for a segment for IK

% example: x = 1, y = 2, phi = pi/6, l = 5;
m1 = genJmatrix(1,1, pi/6, 5)

% sams function that generates the matrix for each segment
% run for each segment with each desired x,y,phi
% then add(???) positions --> works as compounded positions
% ie the result of the first position is the starting loc for 
% next link?

% takes in desired position (x,y,phi) and 
% properties of the segment (k & l)and gives out matrix Ti
% where T is NOT a transformation matrix
% phidot and kdot are syms?
function Ti =genJmatrix(x, y, phi, l)
 syms phidot kdot
    % From Onal lecture
    %  w = (xt,yt, phi)'; % output vector, could be point, pose etc
    %  theta = (th1, th2)'; % input vector of angles
    
    % From trunk paper:
    % phi = curvature(k) * arcLength(l)
        % use this to calculate curvature since
        % phi is desired and l is known
        display('the curvature k is')
        k = phi/l
    
    % For each section, the partial derivatives are:
a13 = sin(phi)*sin(k*l)*phidot -cos(phi)*cos(k*l)*kdot*l;
a23 = -cos(phi)*sin(k*l)*phidot-sin(phi)*cos(k*l)*kdot*l;
a33 = -sin(k*l)*kdot*l;
a14 = (kdot/(k^2))*(cos(phi)-cos(phi)*cos(k*l))+(phidot/k)*(sin(phi)-sin(phi)*cos(k*l))-(kdot*l/k)*cos(phi)*sin(k*l);
a24 = (kdot/(k^2))*(sin(phi)-sin(phi)*cos(k*l))+(phidot/k)*(cos(phi)*cos(k*l)-cos(phi))-(kdot*l/k)*sin(phi)*sin(k*l);
a34 = -(kdot/(k^2))*sin(k*l)+((kdot*l)/k)*cos(k*l);

Ti = [-sin(phi)*cos(k*l)*phidot-cos(phi)*sin(k*l)*kdot*l, -cos(phi)*phidot, a13, a14;
    cos(phi)*cos(k*l)*phidot-sin(phi)*sin(k*l)*kdot*l, -sin(phi)*phidot, a23, a24;
    cos(k*l)*kdot*l, 0, a33, a34;
    0, 0, 0, 0];
    
end