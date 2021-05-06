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
%% The ode set point simulation of the model
%Set up the initial status [phia0, ka0, phia0_d, ka0_d, phib0, kb0, phib0_d, kb0_d, phic0, kc0, phic0_d, kc0_d]
Qi = [3,1,3,1,3,1];
x0_setpoint = [0;0;0;0;0;0;0;0;0;0;0;0];
tf = 0:0.01:20;
% Qe = [phiaf,kaf,phibf,kbf,phicf,kcf]
[aphia,aka,aphib,akb,aphic,akc] = trajectory(Qi,20);
%Running the ode functions
[T,X] = ode45(@(t,x) odesetpoint(t,x,aphia,aka,aphib,akb,aphic,akc),tf,x0_setpoint);
% Plotting the joint trajectory results
figure('Name','The trajectory of designed theta and actual theta');
plot(T,X(:,1),'color','r');
hold on
plot(T,X(:,2),'color','b');
legend('phia','ka')
figure('Name','The trajectory of designed theta and actual theta');
plot(T,X(:,3),'color','r');
hold on
plot(T,X(:,4),'color','b');
legend('phib','kb')
figure('Name','The trajectory of designed theta and actual theta');
plot(T,X(:,5),'color','r');
hold on
plot(T,X(:,6),'color','b');
legend('phic','kc')
figure('Name','The trajectory of designed theta and actual theta');
plot(T,X(:,7),'color','r');
hold on
plot(T,X(:,8),'color','b');
legend('phiad','kad')
figure('Name','The trajectory of designed theta and actual theta');
plot(T,X(:,9),'color','r');
hold on
plot(T,X(:,10),'color','b');
legend('phibd','kbd')
figure('Name','The trajectory of designed theta and actual theta');
plot(T,X(:,11),'color','r');
hold on
plot(T,X(:,12),'color','b');
legend('phicd','kcd')
