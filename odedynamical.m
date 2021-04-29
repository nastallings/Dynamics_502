function dx = odedynamical(t,x,M,C,N)
%param should include:m1,m2,I1,I2,l1,l2,r1,r2 (system parameters);
% note x is in the form of q_1, q_2,dot q_1, dot q_2
        theta= x(1:2,1);
        theta_d = [pi/3;pi/2];
        e = theta - theta_d;
        dtheta= x(3:4,1);
        e_d = dtheta;
        % the actual dynamic model of the system:
        a = param(1);
        b = param(2);
        d = param(3);
        Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
        invM = inv(Mmat);
        invMC = invM*Cmat;
        % TODO: compute the control input for the system, which
        % should provide the torques
        Kp = [200,0;0,300];
        Kd = [200,0;0,300];
        tau = -Kp*e-Kd*e_d;
        acc = invM*tau - invMC*e_d;
        %TODO: compute dx = f(x,u) hint dx(1)=x(3); dx(2)= x(4); the rest
        %of which depends on the dynamic model of the robot.
        dx = zeros(4,1);
        dx(1) = x(3);
        dx(2) = x(4);
        dx(3) = acc(1);
        dx(4) = acc(2);
end

