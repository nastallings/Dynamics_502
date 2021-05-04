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
%Set up the initial status [phi0, s0, k0, phi0_d, s0_d, k0_d]
x0_setpoint = [0,0,0,0,0,0];
tf = 0:0.01:20;
% Qe = [Qi(1),Qi(2),Qi(3),0,0,0]
p = [1, 0, 0, 0;
     0, 1, 0, 0;
     1, 20, 400, 8000;
     0, 1, 40, 1200];
ap = [a0; a1; a2; a3];
q1 = [0; 0; Qi(1); 0];
s1 = solve(p*ap==q1,ap);
aphi = double([s1.a0;s1.a1;s1.a2;s1.a3]);
q2 = [0; 0; Qi(2); 0];
s2 = solve(p*ap==q2,ap);
as = double([s2.a0;s2.a1;s2.a2;s2.a3]);
q2 = [0; 0; Qi(3); 0];
s3 = solve(p*ap==q3,ap);
ak = double([s3.a0;s3.a1;s3.a2;s3.a3]);
%Running the ode functions
[T,X] = ode45(@(t,x) odesetpoint(t,x,aphi,as,ak),tf,x0_setpoint);
%% Plotting the joint trajectory results
figure('Name','The trajectory of designed theta and actual theta');
plot(T, X,'color','r');
legend('designed trajectory','actual trajectory')
