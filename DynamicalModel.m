function [M,C,N] = DynamicalModel(T,J)
    %	This function should take whatever it needs to build a dynamical model for the robot 
    
    %   I'm not sure about the input, but I think it definately needs to take T&J
    %   to calculate the M,C,N matrixes
    
    
    syms k_1 k_2 k_3 l_1 l_2 l_3 phi_1 phi_2 phi_3 k_1_dot k_2_dot k_3_dot phi_1_dot phi_2_dot phi_3_dot g

    T01 = [cos(phi_1)*cos(k_1 * l_1), -sin(phi_1), -cos(phi_1)*sin(k_2*l_2), -(1/k_1)*cos(phi_1)+(1/k_1)*cos(phi_1)*cos(k_1*l_1);
                 sin(phi_1)*cos(k_1*l_1),  cos(phi_1), -sin(phi_1)*sin(k_1*l_1), -(1/k_1)*sin(phi_1)+(1/k_1)*sin(phi_1)*cos(k_1*l_1);
                 sin(k_1*l_1), 0, cos(k_1*l_1), (1/k_1)*sin(k_1*l_1);
                 0, 0, 0, 1];
    T12 = [cos(phi_2)*cos(k_2 * l_2), -sin(phi_2), -cos(phi_2)*sin(k_2*l_2), -(1/k_2)*cos(phi_2)+(1/k_2)*cos(phi_2)*cos(k_2*l_2);
                 sin(phi_2)*cos(k_2*l_2),  cos(phi_2), -sin(phi_2)*sin(k_2*l_2), -(1/k_2)*sin(phi_2)+(1/k_2)*sin(phi_2)*cos(k_2*l_2);
                 sin(k_2*l_2), 0, cos(k_2*l_2), (1/k_2)*sin(k_2*l_2);
                 0, 0, 0, 1];
    T23 = [cos(phi_3)*cos(k_3 * l_3), -sin(phi_3), -cos(phi_3)*sin(k_3*l_3), -(1/k_3)*cos(phi_3)+(1/k_3)*cos(phi_3)*cos(k_3*l_3);
                 sin(phi_3)*cos(k_2*l_3),  cos(phi_3), -sin(phi_3)*sin(k_2*l_3), -(1/k_3)*sin(phi_3)+(1/k_3)*sin(phi_3)*cos(k_3*l_3);
                 sin(k_3*l_3), 0, cos(k_3*l_3), (1/k_3)*sin(k_3*l_3);
                 0, 0, 0, 1];           
    %T02         
    T02 = T12*T01
    %T03
    T03 = T23*T12*T01

             
             
    %Provided by Elephant Trunk Paper
    vel_1 = [(k_1_dot/(k_1^2))*(cos(phi_1)-cos(k_1*l_1)) + (phi_1_dot/k_1)*(sin(phi_1)-sin(phi_1)*cos(k_1*l_1)) - ((k_1_dot*l_1)/k_1)*cos(phi_1)*sin(k_1*l_1); 
             (k_1_dot/(k_1^2))*(sin(phi_1)-sin(phi_1)*cos(k_1*l_1)) + (phi_1_dot/k_1)*(cos(phi_1)*cos(k_1*l_1)-cos(phi_1)) - ((k_1_dot*l_1)/k_1)*sin(phi_1)*sin(k_1*l_1);
             (-k_1_dot/(k_1^2))*sin(k_1*l_1)+(k_1_dot*l_1)/k_1 * cos(k_1*l_1)];
    
         
    %will have to be adjusted which is terrifying      
    vel_2 = [(k_2_dot/(k_2^2))*(cos(phi_2)-cos(k_2*l_2)) + (phi_2_dot/k_2)*(sin(phi_2)-sin(phi_2)*cos(k_2*l_2)) - ((k_2_dot*l_2)/k_2)*cos(phi_2)*sin(k_2*l_2); 
             (k_2_dot/(k_2^2))*(sin(phi_2)-sin(phi_2)*cos(k_2*l_2)) + (phi_2_dot/k_2)*(cos(phi_2)*cos(k_2*l_2)-cos(phi_2)) - ((k_2_dot*l_2)/k_2)*sin(phi_2)*sin(k_2*l_2);
             (-k_2_dot/(k_2^2))*sin(k_2*l_2)+(k_2_dot*l_2)/k_2 * cos(k_2*l_2)];
    vel_3 = [(k_3_dot/(k_3^2))*(cos(phi_3)-cos(k_3*l_3)) + (phi_3_dot/k_3)*(sin(phi_3)-sin(phi_3)*cos(k_3*l_3)) - ((k_3_dot*l_3)/k_3)*cos(phi_3)*sin(k_3*l_3); 
             (k_3_dot/(k_3^2))*(sin(phi_3)-sin(phi_3)*cos(k_3*l_3)) + (phi_3_dot/k_3)*(cos(phi_3)*cos(k_3*l_3)-cos(phi_3)) - ((k_3_dot*l_3)/k_3)*sin(phi_3)*sin(k_3*l_3);
             (-k_3_dot/(k_3^2))*sin(k_3*l_3)+(k_3_dot*l_3)/k_3 * cos(k_3*l_3)];

    %Mass
    m1 = 1;
    m2 = 1;
    m3 = 1;
         
    %K Term
    K1 = 1/2*m1*vel_1.' * vel_1;
    K2 = 1/2*m2*vel_2.' * vel_2;
    K3 = 1/2*m3*vel_3.' * vel_3;        
    Kin_energy = K1+K2+K3
    
    %P Term 
    P1 = m1*g*T01(3,4);
    P2 = m2*g*T02(3,4);
    P3 = m3*g*T03(3,4); 
    Pot_energy = P1+P2+P3;
    
    L = simplify(Kin_energy - Pot_energy);
    
    % The output of this function should be a symbolic expression of M,C,N matrixes 
    M = inputArg1;
    C = inputArg2;
    N = inputArg2;
end

