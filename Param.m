%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script gives all necessary parameters of the helicopter
% 
% This file is for use within the course "Practice of Concepts of
% Automatic Control" at the University of Stuttgart, held by F.
% Allgoewer.
% 
% Written by Y. Wang, J. Pfingsten and Y. Assanov, Dec. 2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parameters
% Length
l_1 = 495e-3;       % Length of the rail (m)
l_2 = 177.5e-3;     % Distance to the motor center of gravity (m)
l_3 = 210e-3;       % Distance to the main arm center of gravity (m)
l_4 = 228.5e-3;     % Distance to main/auxiliary arm junction (m)
l_5 = 135.5e-3;     % Distance to secondary arm center of gravity (m)
l_6 = 262e-3;       % Distance to counterweight center of gravity (m)
l_7 = 655e-3;       % Distance to rail (m)
l_8 = 51.5e-3;      % Distance from suspension point to main arm (m)
l_H = 915e-3;       % Length of main arm (m)

% Mass
m_F = 0.487;        % Mass of front propeller/motor (kg)
m_B = 0.487;        % Mass of back propeller/motor (kg)
m_S = 0.322;        % Mass of the rail (kg)
m_H = 0.377;        % Mass of the main arm (kg)
m_N = 0.138;        % Mass of the secondary arm (kg)
m_W = 1.918;        % Mass of counterweight (kg)
m_J = 0.202;        % Mass of joint block 2/3 (kg)

% others
phi = deg2rad(15);  % Elevation angle at the equilibrium point
g   = 9.81;         % Gravity
