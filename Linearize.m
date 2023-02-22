%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script is to linearize the helicopter model.
% 
% Variables for other scripts: 
% A, B, C, D: Linear system matrices
% x_s, y_s: States and outputs at the equilibrium point
% x_0, y_0: States and outputs at the initial point
% 
% This file is for use within the course "Practice of Concepts of
% Automatic Control" at the University of Stuttgart, held by F.
% Allgoewer.
% 
% Written by Y. Wang, J. Pfingsten and Y. Assanov, Dec. 2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Definition of symbolic variables
% As Inputs:
% Motor forces, Motor Voltages
syms F_F F_B F_sum F_delta
syms U_F U_B
syms F_F_0 F_B_0

% As States:
syms alpha alpha_dot beta beta_dot gamma gamma_dot

% As Parameters
% Moments of Inerita
syms J_x J_y J_z
% Moments
syms M_Mot_gamma M_N M_H M_W M_B M_J_S M_F 

%% Parameters
Param

%% Calculation
% Moment of inertia
J_x = m_H*l_H^2/12 + m_H*l_3^2 + m_N*(l_4+l_5*cos(phi))^2*cos(beta)^2 + ...
    m_W*(l_4+l_6*cos(phi))^2*cos(beta)^2 + (m_J+m_S)*l_7^2*cos(beta)^2 + ...
    (m_F+m_B)*(l_7^2*cos(beta)^2+l_2^2*cos(gamma)^2);


J_y = m_H*l_H^2/12 + l_H*(l_3^2 + l_8^2) + m_N*((l_4+l_5*cos(phi))^2+...
    (l_8-l_5*sin(phi))^2) + m_W*((l_4+l_6*cos(phi))^2+...
    (l_8-l_6*sin(phi))^2) + (m_J+m_S+m_F+m_B)*(l_7^2+l_8^2);

J_z = m_S*l_1^2/12 + (m_F+m_B)*l_2^2;

% Moments
M_Mot_alpha = -l_7*(F_F + F_B)*cos(beta)*sin(gamma);
M_Mot_beta  = l_7*(F_F + F_B)*cos(gamma);
M_Mot_gamma = l_2*(F_F - F_B);
M_N  = m_N*g*((l_4+l_5*cos(phi))*cos(beta)-l_8*sin(beta));
M_H  = -m_H*g*(l_3*cos(beta)+l_8*sin(beta));
M_W  = m_W*g*((l_4+l_6*cos(phi))*cos(beta)-l_8*sin(beta));
M_B  = -m_B*g*(l_7*cos(beta)+l_8*sin(beta));
M_J_S = -(m_J+m_S)*g*(l_7*cos(beta)+l_8*sin(beta));
M_F  = -m_F*g*(l_7*cos(beta)+l_8*sin(beta)); % kleiner Fehler korrigiert


%% Linearization
% Second order derivatives of the angles
alpha_ddot = M_Mot_alpha/J_x;
beta_ddot = (M_Mot_beta+M_N+M_H+M_W+M_B+M_J_S+M_F)/J_y;
gamma_ddot = M_Mot_gamma/J_z;

x = [alpha;beta;gamma;alpha_dot;beta_dot;gamma_dot];
u = [F_F;F_B];

% Equilibrium point
x_s = [0;-phi;0;0;0;0];

% Equations to be solved: the second order derivative should be zero at the
% equilibrium point
beta_dd_eqn = subs(beta_ddot, [x;F_F;F_B], [x_s;F_sum/2;F_sum/2]) == 0;
 
F_sum_stat  = solve(beta_dd_eqn, F_sum);

F_F_stat = F_sum_stat / 2;
F_B_stat = F_sum_stat / 2; 
u_s = double([F_F_stat;F_B_stat]);

% Linearize the function x_dot = f(x,u) with Jacobian matrices
f = [alpha_dot; beta_dot; gamma_dot; alpha_ddot; beta_ddot; gamma_ddot];

A_lin = jacobian(f, x);
A = double(subs(A_lin, [x;u], [x_s;u_s]));

B_lin = jacobian(f, u);
B = double(subs(B_lin, [x;u], [x_s;u_s]));

C = [eye(3), zeros(3,3)];

D = zeros(3,2);
%% Other Results
% Outputs at equilibrium point
y_s = C*x_s;

% Initial point
x_0 = [0;deg2rad(-27);0;0;0;0];
y_0 = C*x_0;

%% Clear Unecessary Variables
clearvars -except a_F a_B A B C D u_s x_s y_s x_0 y_0 
