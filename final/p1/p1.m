clc;
clear;

A = [0, 0, 1, 0;
    0, 0, 0, 1;
    0, -1, -0.1, 0.1;
    0, 11, 0.1, -0.11];

B = [0, 0;
    0, 0;
    0.1, -0.1;
    -0.1, 1.1];

%problem 1a
disp('part a')

B_tau_only = [0, 0;
              0, 0;
              0, -0.1;
              0, 1.1];
          
%compute controllabiltiy matrix and check rank
if rank(ctrb(A,B_tau_only)) ~= 4
    disp('not controllable using only Tau')
    ctrb(A,B_tau_only)
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
disp('part b')


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
C = [0, 0, 1, 0; 0, 1, 0, 0];
disp('part c')

sys = ss(A,B,C,[0,0;0,0]);