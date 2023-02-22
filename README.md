# Praktikum-KRT
This Project is for use of the course "Praktikum Konzepte der Regelungstechnik" or in English "Practice of Concepts of the Control Technique" at Institute for Systems Theory and Automatic Control of University of Stuttgart, Germany, held by Professor F. Allgoewer. The task is to design a controller for a helicopter model so that it travels along given trajectory.

## Usage

1. Extract the file in a folder.
2. Open MATLAB and set current folder to where you have just extract the file.
3. Run the script Controller.m, and open the model Helicopter.slx.
4. Simulate the model by clicking Run button.

## Features
We have applied the LQR control algorithms for that QUANSER 3-DOF Helicopter Model. As a preparation we first established the motion equations of the helicopter, and linearize it as a state-space model (A,B,C,D). The LQR state-feedback K can then be calculated by MATLAB function lqr() while the control input is given as u = -Kx. For manipulating the helicopter, the Simulink interface QUARC has been used, by which the helicopter is drived through QUANSER Q8-USB Data Acquisition Board according to signals in Simulink. 

## Contributions
The scripts are written by me, Y. Wang, and my colleagues J. Pfingsten and Y. Assanov.

## Acknowledgements

This project uses MATLAB software by MathWorks. MATLAB is a registered trademark of The MathWorks, Inc. For more information about MATLAB and to obtain a license, please visit [mathworks.com](https://www.mathworks.com/).

This project also uses QUANSER 3-DOF Helicopter Model, QUANSER Q8-USB Data-Acquisiton Board, QUANSER VoltPAQ-X4 Amplifier, as well as QUANSER QUARC Real-Time Control Software, which are copyrights hold by Quanser, Inc. For more information please visit [www.quanser.com](https://www.quanser.com).
