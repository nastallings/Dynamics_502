clc
clear a
close a
%% Get the desired position from user input
Des_pose = zeros(3,1);
Des_pose = input("Please input the desired position:\n");
%% Get the kinematics & dynamical model of the robot
%The symbols are just examples, it can be changed to fit the need of modeling
syms q1 q2 q3
[T,J] = kinematics(q1,q2,q3);
% Get the desired joint positions
Qi = InverseKinematics(Des_pose,J,T);
% Calculate the dynamical model
[M,C,N] = DynamicalModel(T,J);
%% The ode simulation of the model
%Set up the initial status
x0 = [];
tf = 0:0.01:10;
%Running the ode functions
[T,X] = ode45(@(t,x) odedynamical(t,x,M,C,N),tf,x0);
%% Plotting the joint trajectory results
figure('Name','The trajectory of designed theta and actual theta');
plot(T, X,'color','r');
legend('designed trajectory','actual trajectory')
