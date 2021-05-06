clc
clear a
close a
%% Get the desired position from user input
load('M_N_C_variables.mat')
Des_pose = zeros(3,1);
Des_pose = input("Please input the desired position:\n");
%% Get the kinematics & dynamical model of the robot
%The symbols are just examples, it can be changed to fit the need of modeling
syms q1 q2 q3
[T,J] = kinematics(q1,q2,q3);
% Get the desired joint positions
Qi = InverseKinematics(Des_pose,J,T);
% Calculate the dynamical model
% [M,C,N] = DynamicalModel(T,J);
%% The ode set point simulation of the model
%Set up the initial status [phia0, ka0, phia0_d, ka0_d, phib0, kb0, phib0_d, kb0_d, phic0, kc0, phic0_d, kc0_d]
Qi = [3,1,2];
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
