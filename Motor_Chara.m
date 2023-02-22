%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script calculate the coefficient to transfer motor voltage to 
% motor force.

% Variables for other scripts: 
% a_F, a_B: Voltage to Force coefficient of front and back motor
% respectively
% 
% This file is for use within the course "Practice of Concepts of
% Automatic Control" at the University of Stuttgart, held by F.
% Allgoewer.
% 
% Written by Y. Wang, J. Pfingsten and Y. Assanov, Dec. 2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Motor Characteristics
% Measured voltages and force from weighting scale
U = [0 0.5 1 1.5 2 2.4 2.8 3.2 3.6 4];    % [V]
mass_F = [0 0 5 10 19 27 37 53 63 85];    % [g]
mass_B = [0 0 5 12 22 33 47 64 82 103];   % [g]

F_F = mass_F*9.81*1e-3;
F_B = mass_B*9.81*1e-3;

hold on;
fun = @(a,U)a*U.^2;
x0 = 0;
a_F = lsqcurvefit(fun,x0,U,F_F);
a_B = lsqcurvefit(fun,x0,U,F_B);

clearvars -except a_F a_B A B C D u_s x_s y_s x_0 y_0 