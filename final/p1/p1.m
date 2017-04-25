clc;
clear;

%load the data file
load('pendulum.mat')

%problem 1a
disp('__________part a_______________')


B_tau_only = [0, 0;
              0, 0;
              0, -0.1;
              0, 1.1];
          
%compute controllabiltiy matrix and check rank
if rank(ctrb(A,B_tau_only)) ~= 4
    disp('not controllable using only Tau')
    controllability_matrix = ctrb(A,B_tau_only)
else
    ctrb(A,B_tau_only)
end

%get the e-vals of A
eigs = eig(A)

%use PBH test to check if stabilizable
test_eig1 = rank([A - eigs(1,1)*eye(4) B_tau_only])
test_eig2 = rank([A - eigs(2,1)*eye(4) B_tau_only])
test_eig3 = rank([A - eigs(3,1)*eye(4) B_tau_only])
test_eig4 = rank([A - eigs(4,1)*eye(4) B_tau_only])

%problem 1b
disp('__________part b_______________')



C1 = [1, 0, 0, 0];
C2 = [0, 1, 0, 0];

%check the controllability of each to see if rank = n = 4
rank(obsv(A,C1))
rank(obsv(A,C2))

%check the detectability for each e-value
rank([A-eigs(1,1)*eye(4); C2])
rank([A-eigs(2,1)*eye(4); C2])
rank([A-eigs(3,1)*eye(4); C2])
rank([A-eigs(4,1)*eye(4); C2])

%problem 1c
B = B(:,1);
C = [0, 0, 1, 0; 0, 1, 0, 0];
D = [0;0];
disp('__________part c_______________')

sys = ss(A,B,C,D);

%check if observable
if rank(obsv(A,C)) ~= 4,
    disp('not observable')
else
    rank(obsv(A,C))
end

%not observable, so lets get the transmission zeros
transmission_zeros = tzero(minreal(sys))

%see where G(S) looses rank
syms s
s = 1;
G = C*inv(s*eye(4)-A)*B + D;

disp('__________part d_______________')

C = eye(4);
sys = ss(A,B,C,[0; 0; 0; 0]);
t = 0:0.01:100;
u = zeros(1,length(t));
x0 = [0, 0, 0, 0]';
[Y, T, X] = lsim(sys,u,t,x0);

figure(1)
subplot(4,1,1)
plot(T,X(:,1))
title('Question 1 Part d: State Values vs Time')
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

