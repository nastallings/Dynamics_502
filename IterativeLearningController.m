tf = 50;

% x0 = [Ahorizontal, Ahorizontal, Avertical, Avertical_dot, Chorizontal, Chorizontal_dot, uAh, uAv, uCh]
x0 = [0, 0, 0, 0, 0, 0, 0, 0, 0];
xf = [2, 0, 4, 0, 2, 0, 0, 0, 0];
[T,X] = ode45(@(t,x) IterativeODE(t, x, xf), [0 tf], x0);

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


function [dx] = IterativeODE(t, x, xf)        
    % Ahorizontal 
    AH_e = x(1)-xf(1);
    Kp_AH = 6;
    Kd_AH = 5;
    Ah_beta = 0.25;
    AHtau = 1/Ah_beta *(-Kp_AH*AH_e)-Kd_AH*x(2)+x(7);    
    duAH= -1/Ah_beta * Kp_AH*AH_e+x(7);
    
    % Avertical 
    AV_e = x(3)-xf(3);
    Kp_AV = 10;
    Kd_AV = 9;
    Av_beta = 0.25;
    AVtau = 1/Av_beta *(-Kp_AV * AV_e) -Kd_AV*x(4)+x(8);    
    duAV= -1/Av_beta * Kp_AV * AV_e+x(8);  
    
    % Chorizontal 
    CH_e = x(5)-xf(5);
    Kp_CH = 16;
    Kd_CH = 2;
    Ch_beta = 0.25;
    CHtau = 1/Ch_beta *(-Kp_CH*CH_e) -Kd_CH*x(6)+x(9);    
    duCH= -1/Ch_beta * (Kp_CH*CH_e) +x(9);  
    
    % Calculate Model 
    dx_dot = [AHtau;
              AVtau;
              CHtau;];
    
    %Use the computed torque and state space model to compute the increment in state vector.
    dx(1,1) = x(2);
    dx(2,1) = dx_dot(1);
    dx(3,1) = x(4);
    dx(4,1) = dx_dot(2);
    dx(5,1) = x(6);
    dx(6,1) = dx_dot(3);
    dx(7,1) = duAH;
    dx(8,1) = duAV;
    dx(9,1) = duCH;
end