function [phi,k] = IK2D(x, y, z)

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

