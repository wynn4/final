clc;
clear all;

clear;

A = [0, 0, 1, 0;
    0, 0, 0, 1;
    0, -1, -0.1, 0.1;
    0, 11, 0.1, -0.11];

B = [0, 0;
    0, 0;
    0.1, 0;
    -0.1, 0];

%%
disp('_________part a___________')
B = B(:,1)

%lets measure cart position P
C = eye(4);

D = 0;
D = [0; 0; 0; 0];

sys = ss(A,B,C,D);

%desired poles
des_poles = [-1, -2, -1+0.5i, -1-0.5i]

%get the K which places the polse of A-BK to be the desired poles
K = place(A,B,des_poles)

%check to see that it worked
eig(A-B*K)

%%

disp('_________part b___________')

% create the feedback system
feedback_sys = ss([A-B*K],B,C,D);

t = 0:0.1:10;

% u(t) = zero since its already built in to feedback_sys
u = zeros(1,length(t));
x0 = [1, -0.2, 2, -0.1]';

% use lsim to generate data for plots
[Y, T, X] = lsim(feedback_sys,u,t,x0);

%back out what F(t) must have been
u_force = -K*X';


figure(1)
plot(T,X)
title('Question 2 Part b: State Values vs Time')
xlabel('time (s)')
ylabel('state value')
legend('P','theta','pdot','thetadot')

figure(2)
plot(T,u_force)
title('Question 2 Part b: Inuput Force vs Time')
xlabel('time (s)')
ylabel('Force (N)')

%%
disp('_________part c___________')

C = [1, 0, 0, 0];
D = 0;

feedback_sys_2 = ss(A-B*K,B,C,D);





