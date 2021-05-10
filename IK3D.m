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

