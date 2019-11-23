subplot(1,2,1)
plot(time,alpha1,'--');grid on;ylabel('alpha')
subplot(1,2,2)
plot(time,theta,'--');grid on;ylabel('theta')

%%
clear all
close all
clc
%% System
disp('System')
A=[-0.313 56.7 0;-0.0139 -0.426 0;0 56.7 0]
B=[0.232;0.0203;0]
C=[0 1 0]
D=[0;0;0]
%% redefine C,D to get all states out
C=eye(3) %let
D=[0;0;0]
%%
disp('Existing poles i.e. eigenvalues and their corresponding eigenvectors')
[T,D]=eig(A)
%% Check controllability
Qc=[B A*B A*A*B];
disp('Rank of Controllability matrix')
rank(Qc)
disp('check')
rank(ctrb(A,B))
%% Feedback from pole placement
disp('Required pole locations or required eigenvalues')
J=[-2+4*i -2-4*i -10]
disp('Computing the required Feedback matrix for poleplacement')
K=place(A,B,J)
%% Eigenvalues after feedback
disp('Eigenvalues and their eigenvectors after feedback')
[T,D]=eig(A-B*K)
%% Analyzing the output
disp('Analyzing the output of the system to initial condition')
sys_ob = ss(A,B,C,0);
t=0:0.01:4;
u=zeros(size(t));
x0=[0.01 0 0];
[y,t,x] = lsim(sys_ob,u,t,x0);
subplot(2,2,1);
plot(t,y);grid on;
title('Open-Loop Response to Non-Zero Initial Condition')
xlabel('Time (sec)')
ylabel('Y=C*X')
subplot(2,2,2)
pzmap(sys_ob);grid on;

sys_cl=ss(A-B*K,B,C,0);
[y,t,x] = lsim(sys_cl,u,t,x0);
subplot(2,2,3);
plot(t,y);grid on;
title('Closed-Loop Response to Non-Zero Initial Condition')
xlabel('Time (sec)')
ylabel('Y=C*X')
subplot(2,2,4)
pzmap(sys_cl);grid on;
%% Checking the states 
disp('Checking the states of the system for the initial condition')
sys_ob=ss(A,B,eye(3),0);		%how???
disp('We need to check the response till 4 seconds')
t=0:0.01:4;
disp('Setting the initial state values at t=0 and computing the state trajectory values till t=4 seconds')
x=initial(sys_ob,[1;0;0],t);
%disp('isloating the state values into individual matrices')
x1=[1 0 0]*x';
x2=[0 1 0]*x';
x3=[0 0 1]*x';
disp('plotting the state trajectories till t=4 seconds')
subplot(3,2,1);
plot(t,x1);grid on;title('Open loop Response to initial condition');
ylabel('X1');
subplot(3,2,3);
plot(t,x2);grid on;ylabel('X2');
subplot(3,2,5);
plot(t,x3);grid on;ylabel('X3');



sys_cl=ss(A-B*K,eye(3),eye(3),eye(3));		%how???
disp('We need to check the response till 4 seconds')
t=0:0.01:4;
disp('Setting the initial state values at t=0 and computing the state trajectory values till t=4 seconds')
x=initial(sys_cl,[1;0;0],t);
%disp('isloating the state values into individual matrices')
x1=[1 0 0]*x';
x2=[0 1 0]*x';
x3=[0 0 1]*x';
disp('plotting the state trajectories till t=4 seconds')
subplot(3,2,2);
plot(t,x1);grid on;title('Closed loop Response to initial condition');
ylabel('X1');
subplot(3,2,4);
plot(t,x2);grid on;ylabel('X2');
subplot(3,2,6);
plot(t,x3);grid on;ylabel('X3');
%% check observability
Qo=[C;C*A;C*A*A];
disp('rank of observability matrix')
rank(Qo)
disp('check')
O = obsv(A,C);
rank(O)
%% Design observer
ob_poles=[-100 -101 -102]


% design observer by placing poles of A-LC 
L=place(A',C',ob_poles)'

% check poles of estimator-error dynamics
est_poles = eig(A - L*C)

Ahat = A;

Bhat = [B L]

Chat = eye(3);

Dhat=[0 0;0 0;0 0]
sys_ob = ss(Ahat,Bhat,Chat,Dhat);
lsim(sys_ob,zeros(size(t)),t,[x0 x0]);