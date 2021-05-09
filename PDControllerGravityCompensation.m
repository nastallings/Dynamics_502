tf = 100;

% x0 = [theta, theta_dot, phi, phi_dot]
x0 = [pi/12, 0, pi/3, 0];
xf = [2; pi/2;0];
[T,X] = ode45(@(t,x) IterativeODE(t, x, xf), [0 tf], x0);
figure,
ax1 = subplot(2, 2, 1);
plot(T,X(:,1), 'linewidth',2)
ylabel("theta");
xlabel("Time");
title('theta vs Time')
hold on
yline(xf(1), 'r--', 'LineWidth', 1);

ax2 = subplot(2, 2, 2);
plot(T,X(:,3), 'linewidth',2)
ylabel("phi");
xlabel("Time");
title('phi vs Time')
hold on
yline(xf(2), 'r--', 'LineWidth', 1);


ax3 = subplot(2, 2, 3);
plot(T,X(:,2), 'linewidth',2)
ylabel("theta dot");
xlabel("Time");
title('theta dot vs Time')
hold on
yline(0, 'r--', 'LineWidth', 1);

ax4 = subplot(2, 2, 4);
plot(T,X(:,4), 'linewidth',2)
ylabel("phi dot");
xlabel("Time");
title('phi dot vs Time')
yline(0, 'r--', 'LineWidth', 1);

function [dx] = IterativeODE(t, x, xf)   
    q = [x(1);x(3); 0];
    q_dot = [x(2);x(4); 0];
    error = q - xf;
    error_dot = q_dot - [0;0;0];
    Kp = 7;
    Kd = 20;

    tau = (-Kp*error)-(Kd*error_dot);

    %% Dynamical Model
    l1 = 0;
    l2 = 8.34;
    l3 = 0;
    lc1 = 0;
    lc2 = l2/2;
    lc3 = 0;

    g = 9.8;
    m1 = 0;
    m2 = 120;
    m3 = 0;
    q1=x(1);
    q2=x(3);
    q3=0;
    q_dot_1=x(2);
    q_dot_2=x(4);
    q_dot_3 = 0;
    
    I1 = 0; 
    I2 = 0;
    I3 = 0;
    
    % Calculate M
    M11 = I1 + I2 + I3 + (l2^2*m3)/2 + (lc2^2*m2)/2 + (lc3^2*m3)/2 + (l2^2*m3*cos(2*q2))/2 + (lc2^2*m2*cos(2*q2))/2 + (lc3^2*m3*cos(2*q2 + 2*q3))/2 + l2*lc3*m3*cos(q3) + l2*lc3*m3*cos(2*q2 + q3);
    M12 = 0;
    M13 = 0;
    M21 = 0;
    M22 = m3*l2^2 + 2*m3*cos(q3)*l2*lc3 + m2*lc2^2 + m3*lc3^2 + I2 + I3;
    M23 = m3*lc3^2 + l2*m3*cos(q3)*lc3 + I3;
    M31 = 0;
    M32 = m3*lc3^2 + l2*m3*cos(q3)*lc3 + I3;
    M33 = m3*lc3^2 + I3;
    M = [[M11, M12, M13]; [M21, M22, M23]; [M31, M32, M33]];

    % Calculate N
    N1 = 0;
    N2 = g*m3*(lc3*cos(q2 + q3) + l2*cos(q2)) + g*lc2*m2*cos(q2);
    N3 = g*lc3*m3*cos(q2 + q3); 
    N = [N1;N2; N3];

    % Calculate C
    C11 = -q_dot_2*((m3*sin(2*q2)*l2^2)/2 + m3*sin(2*q2 + q3)*l2*lc3 + (m2*sin(2*q2)*lc2^2)/2 + (m3*sin(2*q2 + 2*q3)*lc3^2)/2) - (lc3*m3*q_dot_3*(lc3*sin(2*q2 + 2*q3) + l2*sin(q3) + l2*sin(2*q2 + q3)))/2;
    C12 = -q_dot_1*((m3*sin(2*q2)*l2^2)/2 + m3*sin(2*q2 + q3)*l2*lc3 + (m2*sin(2*q2)*lc2^2)/2 + (m3*sin(2*q2 + 2*q3)*lc3^2)/2);
    C13 = -(lc3*m3*q_dot_1*(lc3*sin(2*q2 + 2*q3) + l2*sin(q3) + l2*sin(2*q2 + q3)))/2;
    C21 = q_dot_1*((m3*sin(2*q2)*l2^2)/2 + m3*sin(2*q2 + q3)*l2*lc3 + (m2*sin(2*q2)*lc2^2)/2 + (m3*sin(2*q2 + 2*q3)*lc3^2)/2);
    C22 = -l2*lc3*m3*q_dot_3*sin(q3);
    C23 = -l2*lc3*m3*sin(q3)*(q_dot_2 + q_dot_3);
    C31 = (lc3*m3*q_dot_1*(lc3*sin(2*q2 + 2*q3) + l2*sin(q3) + l2*sin(2*q2 + q3)))/2;
    C32 = l2*lc3*m3*q_dot_2*sin(q3);
    C33 = 0;
    C = [[C11, C12, C13]; [C21, C22, C23]; [C31, C32, C33]];

    ddx = pinv(M)*(tau-C*q_dot);
    
    dx = [0;0;0;0;];
    %% Update State
    %Use the computed torque and state space model to compute the increment in state vector.
    dx(1) = x(2);
    dx(2) = ddx(1);
    dx(3) = x(4);
    dx(4) = ddx(2);
end