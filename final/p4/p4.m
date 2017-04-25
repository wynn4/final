clc;
clear;
close all;

load('f16_long.mat')

A = Along
B = Blong

r = [1000; 100; 10; 7];
q = 1;

Q1 = [1/500, 0, 0, 0;
    0, 1/degtorad(2.3), 0, 0;
    0, 0, 1/degtorad(17.2), 0;
    0, 0, 0, 1/0.5];

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

[Wn_lqr_1, Z_lqr_1] = damp(sys_lqr_1);
[Wn_lqr_2, Z_lqr_2] = damp(sys_lqr_2);
[Wn_lqr_3, Z_lqr_3] = damp(sys_lqr_3);
[Wn_lqr_4, Z_lqr_4] = damp(sys_lqr_4);


[Wn, Z] = damp(sys);

%plot damping ratio vs osc freq
plot(Wn,Z,Wn_lqr_1,Z_lqr_1,Wn_lqr_2,Z_lqr_2,Wn_lqr_3,Z_lqr_3,Wn_lqr_4,Z_lqr_4)
legend('Open-Loop', 'LQR r = 1000', 'LQR r = 100', 'LQR r = 10', 'LQR r = 7')


  
  