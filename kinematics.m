[k,j] = forKin()

function [FK,J] = forKin() %(q1,q2,q3)
    syms phi s k % not sure what these are yet, maybe inputs? 
    %This is the Transformation matrix (FK) and Jacobian matrix generation function
    % The inputs of this function are symbols that are going to be used in
    % the build of transformation matrix.
    % And the outputs are the symbol expression of T and J matrixes
    
    
    M1 = [ cos(s*k), -sin(s*k), 0, (1/k)*(cos(s*k)-1); sin(s*k), cos(s*k), 0 ,(1/k)*sin(s*k); 0, 0, 1, 0; 0, 0, 0, 1];
    M2 = [cos(phi)*cos(s*k), -sin(phi), -cos(phi)*sin(s*k), (1/k)*(cos(phi) - cos(phi)*cos(s*k)); sin(phi)*cos(s*k), cos(phi), -sin(phi)*sin(s*k), -(1/k)*(sin(phi)*cos(s*k)); sin(s*k), 0, cos(s*k), (1/k)*sin(s*k);0, 0, 0, 1];

    % final T matrix representing forward kin
    FK = M1*M2*M2; %q1+q2;
    %get position vector from T
    posV = [FK(1,4);FK(2,4);FK(3,4)];
    % Calculate the jacobian with the partial derivative
    jointV = [phi,s, k]; % what joint variables get used for JV?
    jv1 = jacobian(posV,jointV);
        % bottom half jv2 = 
    %jacobian
    J = jv1; % ,jv2]'; % q3;
end