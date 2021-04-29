function qi = InverseKinematics(pd,J,T03)
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

