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
syms a0 a1 a2 a3
Qi = [3,1,3,1,3,1];
x0_setpoint = [0;0;0;0;0;0;0;0;0;0;0;0];
tf = 0:0.01:20;
% Qe = [phiaf,kaf,phibf,kbf,phicf,kcf]
p = [1, 0, 0, 0;
     0, 1, 0, 0;
     1, 20, 400, 8000;
     0, 1, 40, 1200];
ap = [a0; a1; a2; a3];
q1 = [0; 0; Qi(1); 0];
s1 = solve(p*ap==q1,ap);
aphia = double([s1.a0;s1.a1;s1.a2;s1.a3]);
q2 = [0; 0; Qi(2); 0];
s2 = solve(p*ap==q2,ap);
aka = double([s2.a0;s2.a1;s2.a2;s2.a3]);
q3 = [0; 0; Qi(3); 0];
s3 = solve(p*ap==q3,ap);
aphib = double([s3.a0;s3.a1;s3.a2;s3.a3]);
q4 = [0; 0; Qi(4); 0];
s4 = solve(p*ap==q4,ap);
akb = double([s4.a0;s4.a1;s4.a2;s4.a3]);
q5 = [0; 0; Qi(2); 0];
s5 = solve(p*ap==q5,ap);
aphic = double([s5.a0;s5.a1;s5.a2;s5.a3]);
q6 = [0; 0; Qi(3); 0];
s6 = solve(p*ap==q6,ap);
akc = double([s6.a0;s6.a1;s6.a2;s6.a3]);
%Running the ode functions
[T,X] = ode45(@(t,x) odesetpoint(t,x,aphia,aka,aphib,akb,aphic,akc),tf,x0_setpoint);
%% Plotting the joint trajectory results
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
