structure = load('M_N_C_variables');
M = structure.M;
C = structure.C;
N = structure.N;

tf = 50;

% x0 = [k_C, phi_C, k_B, phi_B, k_A, phi_A, k_C_dot, phi_C_dot, k_B_dot,
% phi_B_dot, k_A_dot, phi_A_dot, U_A, U_B, U_C]
x0 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
xf = [4, 2, 3, 4, 2, 1];
[T,X] = ode45(@(t,x) IterativeODE(t, x, xf, M, C, N), [0 tf], x0);

figure,
ax1 = subplot(3, 1, 1);
plot(T,X(:,1), 'linewidth',2)
ylabel("AH");
xlabel("Time");
title('A_H vs Time')

ax2 = subplot(3, 1, 2);
plot(T,X(:,3), 'linewidth',2)
ylabel("AV");
xlabel("Time");
title('A_V vs Time')

ax3 = subplot(3, 1, 3);
plot(T,X(:,5), 'linewidth',2)
ylabel("CH");
xlabel("Time");
title('C_H vs Time')

figure,
ax4 = subplot(3, 1, 1);
plot(T,X(:,2), 'linewidth',2)
ylabel("AHdot");
xlabel("Time");
title('AH_d vs Time')

ax5 = subplot(3, 1, 2);
plot(T,X(:,4), 'linewidth',2)
ylabel("AVdot");
xlabel("Time");
title('AV_d vs Time')

ax6 = subplot(3, 1, 3);
plot(T,X(:,6), 'linewidth',2)
ylabel("CHdot");
xlabel("Time");
title('CH_d vs Time')


function [dx] = IterativeODE(t, x, xf, M, C, N)        
%     % C
%     q_C = [x(1);x(2)];
%     q_C_dot = [x(7);x(8)];
%     u_C = x(13);
%     error_C = q_C - xf;
%     Kp_C = 6;
%     Kd_C = 5;
%     C_beta = 0.25;
%     Ctau = 1/C_beta *(-Kp_C*error_C)-Kd_C*q_C_dot+u_C;    
%     duC= -1/C_beta * Kp_C*error_C+u_C;
%     
%     % B 
%     q_B = [x(3);x(4)];
%     q_B_dot = [x(9);x(10)];
%     u_B = x(14);
%     error_B = q_B - xf;
%     Kp_B = 6;
%     Kd_B = 5;
%     B_beta = 0.25;
%     Btau = 1/B_beta *(-Kp_B*error_B)-Kd_B*q_B_dot+u_B;    
%     duB= -1/B_beta * Kp_B*error_B+u_B; 
%     
%     % A 
%     q_A = [x(5);x(6)];
%     q_A_dot = [x(11);x(12)];
%     u_A = x(15);
%     error_A = q_A - xf;
%     Kp_A = 6;
%     Kd_A = 5;
%     A_beta = 0.25;
%     Atau = 1/A_beta *(-Kp_A*error_A)-Kd_A*q_A_dot+u_A;    
%     duA= -1/A_beta * Kp_A*error_A+u_A; 
 

    q = [x(1);x(2);x(3);x(4);x(5);x(6)];
    q_dot = [x(7);x(8);x(9);x(10);x(11);x(12)];
    u = x(13);
    error = q - xf;
    Kp = 6;
    Kd = 5;
    beta = 0.25;
    tau = 1/beta *(-Kp*error)-Kd*q_dot+u;    
    du= -1/beta * Kp*error+u;

    % Calculate Model 
    dx_dot = inv(M)*(tau - C*q_dot - N)
    
    %Use the computed torque and state space model to compute the increment in state vector.
    dx(1,1) = q_dot(1);
    dx(2,1) = q_dot(2);
    dx(3,1) = q_dot(3);
    dx(4,1) = q_dot(4);
    dx(5,1) = q_dot(5);
    dx(6,1) = q_dot(6);
    dx(7,1) = dx_dot(1);
    dx(8,1) = dx_dot(2);
    dx(9,1) = dx_dot(3);
    dx(10,1) = dx_dot(4);
    dx(11,1) = dx_dot(5);
    dx(12,1) = dx_dot(6);
    dx(13,1) = du;
    dx(14,1) = du;
    dx(15,1) = du;
end