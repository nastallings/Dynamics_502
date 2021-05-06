function dx = odesetpoint(t,x,aphia,aka)
%param should include:aphi, as, ak (trajectory parameters);
% note x is in the form of phi, s, k, dot phi, dot s, dot k
        vec_t = [1; t; t^2; t^3]; % cubic polynomials
        theta_d= [aphia'*vec_t; aka'*vec_t];
        % compute the velocity and acceleration for all joints.
        aphia_vel = [aphia(2), 2*aphia(3), 3*aphia(4), 0];
        aphia_acc = [2*aphia(3), 6*aphia(4),0,0 ];
        aka_vel = [aka(2), 2*aka(3), 3*aka(4), 0];
        aka_acc = [2*aka(3), 6*aka(4),0,0 ];
        % compute the desired trajectory (assuming 3rd order polynomials for trajectories)
        dtheta_d =[aphia_vel*vec_t; aka_vel* vec_t ];
        ddtheta_d =[aphia_acc*vec_t; aka_acc* vec_t];
        theta= x(1:2,1);
        e = theta - theta_d;
        dtheta= x(3:4,1);
        ed = dtheta - dtheta_d;
        % comtroller model
        Kp = 200*eye(2);
        Kd = 300*eye(2);
        acc = ddtheta_d - Kd*ed - Kp*e;
        %state updates
        dx = zeros(4,1);
        dx(1) = x(3);
        dx(2) = x(4);
        dx(3) = acc(1);
        dx(4) = acc(2);

end
