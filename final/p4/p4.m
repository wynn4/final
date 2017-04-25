clc;
clear;
close all;

%load the data file
load('f16_long.mat')

disp('__________part a___________')

A = Along
B = Blong

r = [1000; 100; 10; 7];
q = 1;

%normalize Q1 by the equilibrium value of the corresponding states
Q1 = [1/500, 0, 0, 0;
    0, 1/degtorad(2.3), 0, 0;
    0, 0, 1/degtorad(17.2), 0;
    0, 0, 0, 1/0.5];

%Scale R1 according to Bryson's rule:
R1 = [1/5, 0;
      0, 1/degtorad(25)];
  
Q = q*Q1;
R_1 = r(1,1)*R1;
R_2 = r(2,1)*R1;
R_3 = r(3,1)*R1;
R_4 = r(4,1)*R1;

%compute optimal gain K
K_lqr_1 = lqr(A,B,Q,R_1);
K_lqr_2 = lqr(A,B,Q,R_2);
K_lqr_3 = lqr(A,B,Q,R_3);
K_lqr_4 = lqr(A,B,Q,R_4);

%create the LQR system
sys_lqr_1 = ss(A-B*K_lqr_1,B,eye(4),[0,0;0,0;0,0;0,0;]);
sys_lqr_2 = ss(A-B*K_lqr_2,B,eye(4),[0,0;0,0;0,0;0,0;]);
sys_lqr_3 = ss(A-B*K_lqr_3,B,eye(4),[0,0;0,0;0,0;0,0;]);
sys_lqr_4 = ss(A-B*K_lqr_4,B,eye(4),[0,0;0,0;0,0;0,0;]);


%create the open-loop system
sys = ss(A,B,eye(4),[0,0;0,0;0,0;0,0;]);

eigs_of_A = eig(A)

[Wn_lqr_1, Z_lqr_1] = damp(sys_lqr_1);
[Wn_lqr_2, Z_lqr_2] = damp(sys_lqr_2);
[Wn_lqr_3, Z_lqr_3] = damp(sys_lqr_3);
[Wn_lqr_4, Z_lqr_4] = damp(sys_lqr_4);


[Wn, Z] = damp(sys);

%plot damping ratio vs osc freq
figure(1)
plot(Wn(4,1),Z(4,1),'x',Wn_lqr_1(4,1),Z_lqr_1(4,1),'s',Wn_lqr_2(4,1),Z_lqr_2(4,1),'o',Wn_lqr_3(4,1),Z_lqr_3(4,1),'*',Wn_lqr_4(4,1),Z_lqr_4(4,1),'d')
title('Damping Ratio vs Natural Frequency')
xlabel('Wn (hz)')
ylabel('Damping Ratio')
legend('Open-Loop', 'LQR r = 1000', 'LQR r = 100', 'LQR r = 10', 'LQR r = 7','location','NW')


disp('__________part b___________')
R = [0.0355, 0; 0, .0001];

K_lqr = lqr(A,B,Q,R);

t_lqr = 0:0.01:15;
t_ol = 0:0.01:100;
u_lqr = zeros(2,length(t_lqr));
u_ol = zeros(2,length(t_ol));
x0 = [20, .01, -.01, .02];


sys_lqr = ss(A-B*K_lqr,B,eye(4),[0,0;0,0;0,0;0,0;]);

% lsim(sys_lqr,u,t,x0)

% use lsim to simulate the f-16 with LQR
[Y_lqr, T_lqr, X_lqr] = lsim(sys_lqr,u_lqr,t_lqr,x0);

% use lsim to simulate the open-loop f-16
[Y_ol, T_ol, X_ol] = lsim(sys,u_ol,t_ol,x0);

% back out what the input was
U = -K_lqr*X_lqr';

%plot u(t) vs time
figure(2)
plot(t_lqr,U(1,:),'-.',t_lqr,U(2,:))
title('P4 part b: Input U over time')
xlabel('time (s)')
ylabel('input')
legend('throttle','elevator')

%plot the states vs time for both LQR control and open-loop
figure(3)
subplot(4,1,1)
plot(T_lqr,X_lqr(:,1))
title('P4 part b: States under LQR over time')
ylabel('Airspeed')

subplot(4,1,2)
plot(T_lqr,X_lqr(:,2))
ylabel('alpha')

subplot(4,1,3)
plot(T_lqr,X_lqr(:,3))
ylabel('pitch angle')

subplot(4,1,4)
plot(T_lqr,X_lqr(:,4))
ylabel('pitch rate')
xlabel('time (s)')

figure(4)
subplot(4,1,1)
plot(T_ol,X_ol(:,1))
title('P4 part b: Open-Loop states over time')
ylabel('Airspeed')

subplot(4,1,2)
plot(T_ol,X_ol(:,2))
ylabel('alpha')

subplot(4,1,3)
plot(T_ol,X_ol(:,3))
ylabel('pitch angle')

subplot(4,1,4)
plot(T_ol,X_ol(:,4))
ylabel('pitch rate')
xlabel('time (s)')


disp('__________part c___________')
% only measure theta and alpha
C = [0, 1, 0, 0; 0, 0, 1, 0];
% noise = 1e-5*randn(size(t_lqr));
W_noise = [1, 0; 0, 1e-5];
W_disturbance = Blong*(1e-4*eye(2))*Blong';

% Kalman filter design
Kf = (lqr(A',C',W_disturbance, W_noise))'

%create the Kalman filter ss system
%sysKF = ss(A-Kf*C,[B Kf],eye(4),0*[B Kf]);  % Kalman filter estimator

sys = ss(A,B,C,[0, 0; 0, 0]);

reg_sys = reg(sys,K_lqr,Kf)

%reg_sys = zpk(reg_sys)

disp('__________part d___________')

opt = stepDataOptions('StepAmplitude',10);
[Y_step,T_step] = step(reg_sys, opt);

figure(5)
plot(T_step,Y_step(:,:,2))
title('Step Response of Compensator system')
xlabel('time (s)')
ylabel('output')
legend('delta phi', 'delta alpha','location','NW')

