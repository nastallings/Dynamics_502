function [M,C,N] = DynamicalModel(T,J)
    %	This function should take whatever it needs to build a dynamical model for the robot 
    
    %   I'm not sure about the input, but I think it definately needs to take T&J
    %   to calculate the M,C,N matrixes
    
    
    syms k_C k_B k_A l_C l_B l_A phi_C phi_B phi_A k_C_dot k_B_dot k_A_dot phi_C_dot phi_B_dot phi_A_dot g k_C_ddot phi_C_ddot k_B_ddot phi_B_ddot k_A_ddot phi_A_ddot

    %C
    T01 = [cos(phi_C)*cos(k_C * l_C), -sin(phi_C), -cos(phi_C)*sin(k_B*l_B), -(1/k_C)*cos(phi_C)+(1/k_C)*cos(phi_C)*cos(k_C*l_C);
                 sin(phi_C)*cos(k_C*l_C),  cos(phi_C), -sin(phi_C)*sin(k_C*l_C), -(1/k_C)*sin(phi_C)+(1/k_C)*sin(phi_C)*cos(k_C*l_C);
                 sin(k_C*l_C), 0, cos(k_C*l_C), (1/k_C)*sin(k_C*l_C);
                 0, 0, 0, 1];
    %B
    T12 = [cos(phi_B)*cos(k_B * l_B), -sin(phi_B), -cos(phi_B)*sin(k_B*l_B), -(1/k_B)*cos(phi_B)+(1/k_B)*cos(phi_B)*cos(k_B*l_B);
                 sin(phi_B)*cos(k_B*l_B),  cos(phi_B), -sin(phi_B)*sin(k_B*l_B), -(1/k_B)*sin(phi_B)+(1/k_B)*sin(phi_B)*cos(k_B*l_B);
                 sin(k_B*l_B), 0, cos(k_B*l_B), (1/k_B)*sin(k_B*l_B);
                 0, 0, 0, 1];
    
    %A         
    T23 = [cos(phi_A)*cos(k_A * l_A), -sin(phi_A), -cos(phi_A)*sin(k_A*l_A), -(1/k_A)*cos(phi_A)+(1/k_A)*cos(phi_A)*cos(k_A*l_A);
                 sin(phi_A)*cos(k_B*l_A),  cos(phi_A), -sin(phi_A)*sin(k_B*l_A), -(1/k_A)*sin(phi_A)+(1/k_A)*sin(phi_A)*cos(k_A*l_A);
                 sin(k_A*l_A), 0, cos(k_A*l_A), (1/k_A)*sin(k_A*l_A);
                 0, 0, 0, 1];           
    %T02         
    T02 = T12*T01;
    %T03
    T03 = T23*T12*T01;
    
    %Provided by Elephant Trunk Paper and confirmed
    vel_C = expand(chainRule(T01(1:3,4),[k_C, phi_C],[k_C_dot, phi_C_dot],[k_C_ddot, phi_C_ddot]));
      
    %Calculated     
    vel_B = expand(chainRule(T02(1:3,4),[k_B, phi_B],[k_B_dot, phi_B_dot],[k_B_ddot, phi_B_ddot]));
    vel_A = expand(chainRule(T03(1:3,4),[k_A, phi_A],[k_A_dot, phi_A_dot],[k_A_ddot, phi_A_ddot]));

    %Mass
    mC = 120;
    mB = 180;
    mA = 120;
         
    %K Term
    K1 = 1/2*mC*vel_C.' * vel_C;
    K2 = 1/2*mB*vel_B.' * vel_B;
    K3 = 1/2*mA*vel_A.' * vel_A;        
    Kin_energy = K1+K2+K3;
    
    %P Term 
    P1 = mC*g*T01(3,4);
    P2 = mB*g*T02(3,4);
    P3 = mA*g*T03(3,4); 
    Pot_energy = P1+P2+P3;
    
    L = simplify(Kin_energy - Pot_energy);
    
    x1 = diff(L, k_C_dot);
    x2 = diff(L, phi_C_dot);
    x3 = diff(L, k_B_dot);
    x4 = diff(L, phi_B_dot);
    x5 = diff(L, k_A_dot);
    x6 = diff(L, phi_A_dot);
    
    y1 = chainRule(x1, [k_C, phi_C, k_B, phi_B, k_A, phi_A],[k_C_dot, phi_C_dot, k_B_dot, phi_B_dot, k_A_dot, phi_A_dot],[k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot]);
    y2 = chainRule(x2, [k_C, phi_C, k_B, phi_B, k_A, phi_A],[k_C_dot, phi_C_dot, k_B_dot, phi_B_dot, k_A_dot, phi_A_dot],[k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot]);
    y3 = chainRule(x3, [k_C, phi_C, k_B, phi_B, k_A, phi_A],[k_C_dot, phi_C_dot, k_B_dot, phi_B_dot, k_A_dot, phi_A_dot],[k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot]);
    y4 = chainRule(x4, [k_C, phi_C, k_B, phi_B, k_A, phi_A],[k_C_dot, phi_C_dot, k_B_dot, phi_B_dot, k_A_dot, phi_A_dot],[k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot]);
    y5 = chainRule(x5, [k_C, phi_C, k_B, phi_B, k_A, phi_A],[k_C_dot, phi_C_dot, k_B_dot, phi_B_dot, k_A_dot, phi_A_dot],[k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot]);
    y6 = chainRule(x6, [k_C, phi_C, k_B, phi_B, k_A, phi_A],[k_C_dot, phi_C_dot, k_B_dot, phi_B_dot, k_A_dot, phi_A_dot],[k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot]);
    
    z1 = diff(L, k_C);
    z2 = diff(L, phi_C);
    z3 = diff(L, k_B);
    z4 = diff(L, phi_B);
    z5 = diff(L, k_A);
    z6 = diff(L, phi_A);
     
    Tau_1 = y1 - z1;
    Tau_2 = y2 - z2;
    Tau_3 = y3 - z3;
    Tau_4 = y4 - z4;
    Tau_5 = y5 - z5;
    Tau_6 = y6 - z6;
    
    Tau = [Tau_1; Tau_2; Tau_3; Tau_4; Tau_5; Tau_6];
      
    %% Calculate M
    M11 = simplify(Tau_1 - subs(Tau_1,k_C_ddot,0)) /k_C_ddot;
    M12 = simplify(Tau_1 - subs(Tau_1,phi_C_ddot,0)) /phi_C_ddot;
    M13 = simplify(Tau_1 - subs(Tau_1,k_B_ddot,0)) /k_B_ddot;
    M14 = simplify(Tau_1 - subs(Tau_1,phi_B_ddot,0)) /phi_B_ddot;
    M15 = simplify(Tau_1 - subs(Tau_1,k_A_ddot,0)) /k_A_ddot;
    M16 = simplify(Tau_1 - subs(Tau_1,phi_A_ddot,0)) /phi_A_ddot;
    
    M21 = simplify(Tau_2 - subs(Tau_2,k_C_ddot,0)) /k_C_ddot;
    M22 = simplify(Tau_2 - subs(Tau_2,phi_C_ddot,0)) /phi_C_ddot;
    M23 = simplify(Tau_2 - subs(Tau_2,k_B_ddot,0)) /k_B_ddot;
    M24 = simplify(Tau_2 - subs(Tau_2,phi_B_ddot,0)) /phi_B_ddot;
    M25 = simplify(Tau_2 - subs(Tau_2,k_A_ddot,0)) /k_A_ddot;
    M26 = simplify(Tau_2 - subs(Tau_2,phi_A_ddot,0)) /phi_A_ddot;
 
    M31 = simplify(Tau_3 - subs(Tau_3,k_C_ddot,0)) /k_C_ddot;
    M32 = simplify(Tau_3 - subs(Tau_3,phi_C_ddot,0)) /phi_C_ddot;
    M33 = simplify(Tau_3 - subs(Tau_3,k_B_ddot,0)) /k_B_ddot;
    M34 = simplify(Tau_3 - subs(Tau_3,phi_B_ddot,0)) /phi_B_ddot;
    M35 = simplify(Tau_3 - subs(Tau_3,k_A_ddot,0)) /k_A_ddot;
    M36 = simplify(Tau_3 - subs(Tau_3,phi_A_ddot,0)) /phi_A_ddot;
    
    M41 = simplify(Tau_4 - subs(Tau_4,k_C_ddot,0)) /k_C_ddot;
    M42 = simplify(Tau_4 - subs(Tau_4,phi_C_ddot,0)) /phi_C_ddot;
    M43 = simplify(Tau_4 - subs(Tau_4,k_B_ddot,0)) /k_B_ddot;
    M44 = simplify(Tau_4 - subs(Tau_4,phi_B_ddot,0)) /phi_B_ddot;
    M45 = simplify(Tau_4 - subs(Tau_4,k_A_ddot,0)) /k_A_ddot;
    M46 = simplify(Tau_4 - subs(Tau_4,phi_A_ddot,0)) /phi_A_ddot; 
    
    M51 = simplify(Tau_5 - subs(Tau_5,k_C_ddot,0)) /k_C_ddot;
    M52 = simplify(Tau_5 - subs(Tau_5,phi_C_ddot,0)) /phi_C_ddot;
    M53 = simplify(Tau_5 - subs(Tau_5,k_B_ddot,0)) /k_B_ddot;
    M54 = simplify(Tau_5 - subs(Tau_5,phi_B_ddot,0)) /phi_B_ddot;
    M55 = simplify(Tau_5 - subs(Tau_5,k_A_ddot,0)) /k_A_ddot;
    M56 = simplify(Tau_5 - subs(Tau_5,phi_A_ddot,0)) /phi_A_ddot; 
    
    M61 = simplify(Tau_6 - subs(Tau_6,k_C_ddot,0)) /k_C_ddot;
    M62 = simplify(Tau_6 - subs(Tau_6,phi_C_ddot,0)) /phi_C_ddot;
    M63 = simplify(Tau_6 - subs(Tau_6,k_B_ddot,0)) /k_B_ddot;
    M64 = simplify(Tau_6 - subs(Tau_6,phi_B_ddot,0)) /phi_B_ddot;
    M65 = simplify(Tau_6 - subs(Tau_6,k_A_ddot,0)) /k_A_ddot;
    M66 = simplify(Tau_6 - subs(Tau_6,phi_A_ddot,0)) /phi_A_ddot; 
    
    M = [[M11, M12, M13, M14, M15, M16]; [M21, M22, M23, M24, M25, M26]; [M31, M32, M33, M34, M35, M36]; [M41, M42, M43, M44, M45, M46]; [M51, M52, M53, M54, M55, M56]; [M61, M62, M63, M64, M65, M66]];
    M = simplify(M);
    
    %% Calculate N
    N1 = subs(Tau_1, [k_C_dot, phi_C_dot, k_B_dot, phi_B_dot, k_A_dot, phi_A_dot, k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot], [0,0,0,0,0,0,0,0,0,0,0,0]);
    N2 = subs(Tau_2, [k_C_dot, phi_C_dot, k_B_dot, phi_B_dot, k_A_dot, phi_A_dot, k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot], [0,0,0,0,0,0,0,0,0,0,0,0]);
    N3 = subs(Tau_3, [k_C_dot, phi_C_dot, k_B_dot, phi_B_dot, k_A_dot, phi_A_dot, k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot], [0,0,0,0,0,0,0,0,0,0,0,0]);
    N4 = subs(Tau_4, [k_C_dot, phi_C_dot, k_B_dot, phi_B_dot, k_A_dot, phi_A_dot, k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot], [0,0,0,0,0,0,0,0,0,0,0,0]);
    N5 = subs(Tau_5, [k_C_dot, phi_C_dot, k_B_dot, phi_B_dot, k_A_dot, phi_A_dot, k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot], [0,0,0,0,0,0,0,0,0,0,0,0]);
    N6 = subs(Tau_6, [k_C_dot, phi_C_dot, k_B_dot, phi_B_dot, k_A_dot, phi_A_dot, k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot], [0,0,0,0,0,0,0,0,0,0,0,0]);

    N = [N1;N2;N3;N4;N5;N6];
    N = simplify(N);
    
    %% Calculate C
    C1 = Tau_1 - (M(1,:) * [k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot].' + N1);
    C2 = Tau_2 - (M(2,:) * [k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot].' + N2);
    C3 = Tau_3 - (M(3,:) * [k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot].' + N3);
    C4 = Tau_4 - (M(4,:) * [k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot].' + N4);
    C5 = Tau_5 - (M(5,:) * [k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot].' + N5);
    C6 = Tau_6 - (M(6,:) * [k_C_ddot, phi_C_ddot,k_B_ddot, phi_B_ddot,k_A_ddot, phi_A_ddot].' + N6);
    
    C = [C1;C2;C3;C4;C5;C6];
    C = simplify(C);
    
end


function answer = chainRule(func,q,qd,qdd)
    for i = 1:length(func)
        for j = 1:length(q)                
            partials(:,j) = diff(func(i),q(j)) * qd(j) + diff(func(i),qd(j)) * qdd(j);
        end             
        temp(:,i) = simplify(partials);
    end       
    answer = sum(temp).';
end