clc;
clear all;


%load the data file
load('pendulum.mat')

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
subplot(4,1,1)
plot(T,X(:,1))
title('Question 2 Part b: State Values vs Time')
ylabel('P')
subplot(4,1,2)
plot(T,X(:,2))
ylabel('theta')
subplot(4,1,3)
plot(T,X(:,3))
ylabel('Pdot')
subplot(4,1,4)
plot(T,X(:,4))
ylabel('thetadot')
xlabel('time (t)')

figure(2)
plot(T,u_force)
title('Question 2 Part b: Inuput Force vs Time')
xlabel('time (s)')
ylabel('Force (N)')

%%
disp('_________part c___________')

C = [1, 0, 0, 0];
D = [0];

feedback_sys_2 = ss(A-B*K,B,C,D);

W = .01:.01:100;
[mag, phase] = bode(feedback_sys_2, W);



disp('_________part d___________')
C = eye(4);
D = [0; 0; 0; 0];

feedback_sys_3 = ss(A-B*K,B,C,D);

tfinal = 50;
opt = stepDataOptions('StepAmplitude',1);
[Y_step,T_step] = step(feedback_sys_3, tfinal, opt);

figure(3)
subplot(4,1,1)
plot(T_step,Y_step(:,1))
title('Question 2 Part d: States vs Time in response to Step Input')
ylabel('P')
subplot(4,1,2)
plot(T_step,Y_step(:,2))
ylabel('theta')
subplot(4,1,3)
plot(T_step,Y_step(:,3))
ylabel('Pdot')
subplot(4,1,4)
plot(T_step,Y_step(:,4))
ylabel('thetadot')
xlabel('time (t)')

