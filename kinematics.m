function [T,J] = kinematics(q1,q2,q3)
    %This is the Transformation matrix and Jacobian matrix generation function
    % The inputs of this function are symbols that are going to be used in
    % the build of transformation matrix.
    % And the outputs are the symbol expression of T and J matrixes
    T = q1+q2;
    J = q3;
end

