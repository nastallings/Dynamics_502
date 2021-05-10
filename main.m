clc
clear all
close all
%% Get the desired position from user input
% Des_pose = zeros(3,1);
% Des_pose = input("Please input the desired position:\n");
%% The ode set point simulation of the model

%Assume Qi = [3,1,2];
T = forKin(3,1,2);
Des_x = T(1,4);
Des_y = T(2,4);
Des_z = T(3,4);
Qi = zeros(1,3);

% Check wether the inverse kinematics feedbacks the correct results
[Qi(1),Qi(2),Qi(3)] = IK3D(Des_x,Des_y,Des_z);

%Set up the initial status [phia0, ka0, phia0_d, ka0_d, phib0, kb0, phib0_d, kb0_d, phic0, kc0, phic0_d, kc0_d]
x0_setpoint = [0;0;0;0;0;0];
tf = 0:0.01:20;
% Qe = [phiaf,kaf,phibf,kbf,phicf,kcf]
[aphi,ak,atheta] = trajectory(Qi,20);
%Running the ode functions
[T,X] = ode45(@(t,x) odesetpoint(t,x,aphi,ak,atheta),tf,x0_setpoint);
%% Plotting the joint trajectory results
figure('Name','The trajectory of designed joint values');
plot(T,X(:,1),'color','r');
hold on
plot(T,X(:,2),'color','b');
hold on
plot(T,X(:,3),'color','g');
legend('phi','k','theta')
figure('Name','The velocity trajectory of designed joint values');
plot(T,X(:,4),'color','r');
hold on
plot(T,X(:,5),'color','b');
hold on
plot(T,X(:,6),'color','g');
legend('phid','kd','thetad')


%% Dynamical Model 
% x0 = [theta, theta_dot, phi, phi_dot]
x0 = [pi/12, 0, pi/3, 0];
xf = [2; pi/2;0];
Kp = 7;
Kd = 20;
tf = 100;
[TPD,XPD] = ode45(@(t,x) PDControllerODE(t, x, xf, Kp, Kd), [0 tf], x0);
figure,
ax1 = subplot(2, 2, 1);
plot(TPD,XPD(:,1), 'linewidth',2)
ylabel("theta");
xlabel("Time");
title('theta vs Time')
hold on
yline(xf(1), 'r--', 'LineWidth', 1);

ax2 = subplot(2, 2, 2);
plot(TPD,XPD(:,3), 'linewidth',2)
ylabel("phi");
xlabel("Time");
title('phi vs Time')
hold on
yline(xf(2), 'r--', 'LineWidth', 1);


ax3 = subplot(2, 2, 3);
plot(TPD,XPD(:,2), 'linewidth',2)
ylabel("theta dot");
xlabel("Time");
title('theta dot vs Time')
hold on
yline(0, 'r--', 'LineWidth', 1);

ax4 = subplot(2, 2, 4);
plot(TPD,XPD(:,4), 'linewidth',2)
ylabel("phi dot");
xlabel("Time");
title('phi dot vs Time')
yline(0, 'r--', 'LineWidth', 1);
