%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script designs the controller for the helicopter models
% 
% This file is for use within the course "Practice of Concepts of
% Automatic Control" at the University of Stuttgart, held by F.
% Allgoewer.
% 
% Written by Y. Wang, J. Pfingsten and Y. Assanov, Dec. 2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Call for other scripts
Linearize
Motor_Chara
Param

%% Controller Design - by LQR with Integrator
Q= diag([5, 5.1, 5.2, 5.3, 5.4, 5.5, 10.1 ,5.7, 5.8]);
R = diag([1,1.2]);

[K,~,eigs_ctrl] = lqi(ss(A,B,C,D),Q,R);
K = K(:,1:8);

%% Observer desgin - by Pole Placing

max_eig_ctrl = max(abs(real(eigs_ctrl)));
L = place(A',C', -1*[1.05, 1.1, 1.2, 1.3, 1.4, 1.5]*max_eig_ctrl)';

%% Setup for Simulation
% Limits on voltage and force
ulimit = 4;
Flimit = ulimit.^2*(a_F+a_B)/2;

% Swtich among the Variant Subsystem
System.Typ = struct;
System.Typ = 'linear';  %linear, nonlinear, real
