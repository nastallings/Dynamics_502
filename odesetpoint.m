function dx = odesetpoint(t,x,aphi,ak,atheta)
%param should include:aphi, as, ak (trajectory parameters);
% note x is in the form of phi, s, k, dot phi, dot s, dot k
        vec_t = [1; t; t^2; t^3]; % cubic polynomials
        theta_d= [aphi'*vec_t; ak'*vec_t; atheta'*vec_t];
        % compute the velocity and acceleration for all joints.
        aphi_vel = [aphi(2), 2*aphi(3), 3*aphi(4), 0];
        aphi_acc = [2*aphi(3), 6*aphi(4),0,0 ];
        ak_vel = [ak(2), 2*ak(3), 3*ak(4), 0];
        ak_acc = [2*ak(3), 6*ak(4),0,0 ];
        atheta_vel = [atheta(2), 2*atheta(3), 3*atheta(4), 0];
        atheta_acc = [2*atheta(3), 6*atheta(4),0,0 ];

        % compute the desired trajectory (assuming 3rd order polynomials for trajectories)
        dtheta_d =[aphi_vel*vec_t; ak_vel* vec_t;atheta_vel*vec_t];
        ddtheta_d =[aphi_acc*vec_t; ak_acc* vec_t;atheta_acc*vec_t];
        theta= x(1:3,1);
        e = theta - theta_d;
        dtheta= x(4:6,1);
        ed = dtheta - dtheta_d;
        % comtroller model
        Kp = 200*eye(3);
        Kd = 300*eye(3);
        acc = ddtheta_d - Kd*ed - Kp*e;
        %state updates
        dx = zeros(6,1);
        dx(1) = x(4);
        dx(2) = x(5);
        dx(3) = x(6);
        dx(4) = acc(1);
        dx(5) = acc(2);
        dx(6) = acc(3);

end


