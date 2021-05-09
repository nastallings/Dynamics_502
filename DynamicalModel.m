function [M,C,N] = DynamicalModel()
    %	This function should take whatever it needs to build a dynamical model for the robot 
    
    %   I'm not sure about the input, but I think it definately needs to take T&J
    %   to calculate the M,C,N matrixes
    syms q_dot_1 q_dot_2 q_dot_3 real   
    syms q_ddot_1 q_ddot_2 q_ddot_3 real
    syms I1 I2 I3 real
    syms m1 m2 m3
    syms l1 l2 l3
    syms q1 q2 q3 real
    syms lc1 lc2 lc3 g
    screw_axes = [[0;0;1;0;0;0], [0;-1;0;l1;0;0], [0;-1;0;l1;0;-l2]];
    thetalist = [q1; q2; q3];
 
    % space
    I = eye(3);
    joint_transformations = cell(length(thetalist),1);
    for joint = 1 : length(thetalist)
        w = skew(screw_axes(1:3,joint));
        v = screw_axes(4:6,joint);
        theta = thetalist(joint);
        rotation = I + sin(theta)* w + (1-cos(theta))*w^2;
        translation = (I * theta+(1-cos(theta))*w+(theta-sin(theta))*w^2)*v;
        joint_transformations{joint} = [rotation translation; 0 0 0 1];
    end
    M01 = [1 0 0 0;
           0 0 -1 0;
           0 1 0 l1;
           0 0 0 1];
    T01 = joint_transformations{1}*M01;
       
    M02 = [1 0 0 l2;
            0 0 -1 0;
            0 1 0 l1;
            0 0 0 1];
    T02 = simplify(joint_transformations{1}*joint_transformations{2}*M02);
    
    M03 = [1 0 0 l2+l3;
            0 0 -1 0;
             0 1 0 l1;
            0 0 0 1];
    T03 = simplify(joint_transformations{1}*joint_transformations{2}*joint_transformations{3}*M03);

   
    M0_c1 = [1 0 0 0;
            0 1 0 0;
            0 0 1 lc1;
            0 0 0 1];
    T0_c1 = simplify(joint_transformations{1}*M0_c1);  
    
    M0_c2 = [1 0 0 lc2;
            0 0 -1 0;
            0 1 0 l1;
            0 0 0 1];
    T0_c2 = simplify(joint_transformations{1}*joint_transformations{2}*M0_c2);
    
    
    M0_c3 = [1 0 0 l2+lc3;
            0 0 -1 0;
            0 1 0 l1;
            0 0 0 1];
    T0_c3 = simplify(joint_transformations{1}*joint_transformations{2}*joint_transformations{3}*M0_c3);    
    
    
    Oc1 = T0_c1(1:3,4);
    Jv_c1 = jacobian(Oc1, [q1 q2 q3]);

    z0 = [0;0;1];
    zero = zeros([3,1]);
    Jw_c1 = [z0 zero zero];


    Oc2 = T0_c2(1:3,4);
    Jv_c2 = jacobian(Oc2, [q1 q2 q3]);

    z1 = T01(1:3,3);
    Jw_c2 = [z0 z1 zero];

    Oc3 = T0_c3(1:3,4);
    Jv_c3 = jacobian(Oc3, [q1 q2 q3]);

    z2 = T02(1:3,3);
    Jw_c3 = [z0 z1 z2];


%% Calculate M

Mv = m1*Jv_c1.'*Jv_c1 + m2*Jv_c2.'*Jv_c2 + m3*Jv_c3.'*Jv_c3;


Rc1 = T0_c1(1:3,1:3);
Rc2 = T0_c2(1:3,1:3);
Rc3 = T0_c3(1:3,1:3);

Mw = Jw_c1.'*Rc1*I1*Rc1.'*Jw_c1 + Jw_c2.'*Rc2*I2*Rc2.'*Jw_c2 + Jw_c3.'*Rc3*I3*Rc3.'*Jw_c3;


M = simplify(Mv+Mw);



K = simplify((1/2)*[q_dot_1 q_dot_2 q_dot_3]*M*[q_dot_1; q_dot_2; q_dot_3]);

dDdq = zeros(3,3,3)*sym(1);
dDdq(:,:,1) = simplify(diff(M, q1));
dDdq(:,:,2) = simplify(diff(M, q2));
dDdq(:,:,3) = simplify(diff(M, q3));



%% Calculate C
    Cs = zeros(3,3,3)*sym(1);
    for joint = 1:3
        for j = 1:3
            for k = 1:3
                Cs(joint,j,k) = simplify(0.5*(dDdq(k,j,joint) + dDdq(k,joint,j) - dDdq(joint,j,k)));
            end
        end
    end


    C = zeros(2,2)*sym(1);
    dq = [q_dot_1, q_dot_2, q_dot_3];
    for k = 1:3
        for j = 1:3
            C(k,j) = simplify(squeeze(dq*Cs(:,j,k)));
        end
    end

%% Calculate N
% Calculate Potential Energy 
    Pc1 = m1 * g * T0_c1(3,4);
    Pc2 = m2 * g * T0_c2(3,4);
    Pc3 = m3 * g * T0_c3(3,4);
    P = Pc1 + Pc2 + Pc3;

    n1 = diff(P,q1);
    n2 = diff(P,q2);
    n3 = diff(P,q3);
    N = [n1; n2; n3];

%% Dynamical Model 
    Lagrange_Tau = simplify(expand(M*[q_ddot_1; q_ddot_2; q_ddot_3]+ C*[q_dot_1; q_dot_2; q_dot_3] + N));
end

function X = skew(x)
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end