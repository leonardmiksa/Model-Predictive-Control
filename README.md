# Model-Predictive-Control

In this project, a Model Predictive Control algorithm is implemented in MATLAB.
This work was done within the labs in the Control of Mechatronic Systems course on my own proposal, and approved and mentored by prof. Ervin Kamenar. 
The idea was to develop the MPC algorithm for pneumatic artificial muscle which would be used in a robotic exoskeleton for stroke rehabilitation patients.
This project was intended to be a starting point on which future generations of students would continue to work and explore.

Key features:
- Model Predictive Control theory - parameters and principles
- code development in MATLAB (a custom-written code from scratch, without MPC toolbox)
- testing the algorithm on a state-space model of a separately excited DC motor with constant excitation, and pneumatic artificial muscle
- implementation of input constraints and state constraints; both are hard constraints and soft constraints are not implemented due to the scope of course
- comparison with LQR (used for validation of the results)

Tools used:
- MATLAB (without Simulink)

Due to the complexity and extensiveness of the topic, the algorithm was not implemented on real hardware, but all simulations were ran successfully and provided valid results, including reference tracking, input constraints, and state constraints.

> 📎 Full project report (in English) is available in the provided PDF file.


Author:
Leonard Mikša,
MSc Electrical Engineering student,
Email: leonardmiksa@gmail.com
