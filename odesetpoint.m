function dx = odesetpoint(t,x,aphia,aka,aphib,akb,aphic,akc)
%param should include:aphi, as, ak (trajectory parameters);
% note x is in the form of phi, s, k, dot phi, dot s, dot k
        vec_t = [1; t; t^2; t^3]; % cubic polynomials
        theta_d= [aphia'*vec_t; aka'*vec_t; aphib'*vec_t; akb'*vec_t;aphic'*vec_t; akc'*vec_t;];
        % compute the velocity and acceleration for all joints.
        aphia_vel = [aphia(2), 2*aphia(3), 3*aphia(4), 0];
        aphia_acc = [2*aphia(3), 6*aphia(4),0,0 ];
        aka_vel = [aka(2), 2*aka(3), 3*aka(4), 0];
        aka_acc = [2*aka(3), 6*aka(4),0,0 ];
        aphib_vel = [aphib(2), 2*aphib(3), 3*aphib(4), 0];
        aphib_acc = [2*aphib(3), 6*aphib(4),0,0 ];
        akb_vel = [akb(2), 2*akb(3), 3*akb(4), 0];
        akb_acc = [2*akb(3), 6*akb(4),0,0 ];
        aphic_vel = [aphic(2), 2*aphic(3), 3*aphic(4), 0];
        aphic_acc = [2*aphic(3), 6*aphic(4),0,0 ];
        akc_vel = [akc(2), 2*akc(3), 3*akc(4), 0];
        akc_acc = [2*akc(3), 6*akc(4),0,0 ];
        % compute the desired trajectory (assuming 3rd order polynomials for trajectories)
        dtheta_d =[aphia_vel*vec_t; aka_vel* vec_t;aphib_vel*vec_t; akb_vel* vec_t;aphic_vel*vec_t; akc_vel* vec_t ];
        ddtheta_d =[aphia_acc*vec_t; aka_acc* vec_t;aphib_acc*vec_t; akb_acc* vec_t;aphic_acc*vec_t; akc_acc* vec_t];
        theta= x(1:6,1);
        e = theta - theta_d;
        dtheta= x(7:12,1);
        ed = dtheta - dtheta_d;
        % comtroller model
        Kp = 200*eye(6);
        Kd = 300*eye(6);
        acc = ddtheta_d - Kd*ed - Kp*e;
        %state updates
        dx = zeros(12,1);
        dx(1) = x(7);
        dx(2) = x(8);
        dx(3) = x(9);
        dx(4) = x(10);
        dx(5) = x(11);
        dx(6) = x(12);
        dx(7) = acc(1);
        dx(8) = acc(2);
        dx(9) = acc(3);
        dx(10) = acc(4);
        dx(11) = acc(5);
        dx(12) = acc(6);
end


