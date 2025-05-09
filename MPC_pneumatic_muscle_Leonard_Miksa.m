clc;
clear all;
%Leonard Mikša

%% LQR

%Initial conditions:
x0 = [0;  %Initial linear displacement
      0;  %Initial linear speed of expansion/contraction
      0]; %Initial air flow
  
% Parameters of the system:                   
m = 0.03;                                       %Muscle mass
b = 0.5;                                        %Friction coefficient
k = 0.04;                                       %Spring constant                    
K = 0.6;                                        %Gain of the pneumatic valve
tau = 0.05;                                     %Time constant of the pneumatic valve
gama = 2000;                                    %Proportionality constant
A = 0.000314;                                   %Cross-section area of the muscle

%State-space matrices:
A = [0      1       0;
     -k/m   -b/m    gama*A/m;
     0      0       -1/tau];

B = [0;
     0;
     K/tau];

C = [1  0  0];

D = [0];

%Discretization parameters:
Ts = 0.02;                                      %Sampling time 
sys_cont = ss(A, B, C, D);                      %Continuous-time system
sys_disc = c2d(sys_cont, Ts, 'zoh');            %Discretization

%Extracting discrete-time matrices:
Ad = sys_disc.A;
Bd = sys_disc.B;
Cd = sys_disc.C;
Dd = sys_disc.D;

%Discrete-time LQR:
Q = [10000    0    0;             %Higher Q1 means more aggresive control of the displacement (but with bigger air consumption)
     0    10    0;            
     0    0    100]           
R = 1;  
Ksystem = dlqr(Ad, Bd, Q, R);     %Discrete-time LQR gain

%Closed-loop system:
sys_cl = ss(Ad - Bd * Ksystem, Bd, Cd, Dd, Ts);

%DC gain for reference tracking:
Kdc = dcgain(sys_cl);
Kr = 1/Kdc;
sys_cl = ss(Ad - Bd * Ksystem, Bd * Kr, Cd, Dd, Ts);

%Multiple input references (comment unnecessary):
t = 0:Ts:5;
ref = zeros(size(t));
ref(t >= 1) = 1;
% ref(t >= 1.14) = 0.25; 
% ref(t >= 1.28) = 0.7; 
% ref(t >= 1.42) = 0; 
% ref(t >= 1.56) = -0.5;
% ref(t >= 1.7) = -1; 
% ref(t >= 1.84) = -0.25;
% ref(t >= 1.98) = -0.75; 
% ref(t >= 2.12) = 0.5;
% ref(t >= 2.26) = 0; 

%System response:
[y, t, xlqr] = lsim(sys_cl, ref, t, x0);








%Leonard Mikša - MPC controlled pneumatic artifical muscle

%% State-space model of the DC motor  
%Parameters of the system:                
m = 0.03;                                             %Muscle mass 
b = 0.5;                                              %Friction coefficient 
k = 0.04;                                             %Spring constant                           
K = 0.6;                                              %Gain of the pneumatic valve 
tau = 0.05;                                           %Time constant of the pneumatic valve 
gama = 2000;                                          %Proportionality constant
A = 0.000314;                                         %Cross-section area of the muscle

%State-space matrices:
A = [0      1       0;
     -k/m   -b/m    gama*A/m;
     0      0       -1/tau]
 
B = [0;
     0;
     K/tau]
 
C = [1    0    0]

D = [0]

%Simulation setup:
steps = 250; 
sim_time = 5;                  %Total simulation time (seconds)
Ts = sim_time / steps;         %Sampling time

[Ad, Bd] = c2d(A, B, Ts);      %Discretization

%% MPC parameters
prediction_horizon = 35;       %Prediction horizon (p)
control_horizon = 6;           %Control horizon (m)

%Regulation matrices:
Q = [8000    0    0;           
     0    10    0;               
     0    0    100]             
R = 1e-9;        

n_x = length(Q);                %Number of states
p = prediction_horizon;
m = control_horizon;

%% Initial conditions
x0 = [0; 0; 0];                 %Initial state of the system (can be replaced with x0 = zeros(:, 1))
x = x0;                         %Current state
u_prev = 0;                     %Previous input

%% Simulation setup
x_trajectory = zeros(n_x, steps);           %Store states for plotting
u_trajectory = zeros(1, steps);             %Store inputs for plotting

%% Optimization setup
%Quadratic cost matrices:
Q_bar = kron(eye(p), Q);                    
R_bar = kron(eye(m), R);                    
H = blkdiag(Q_bar, R_bar);                  %Quadratic cost matrix

%Prediction matrices:
Phi = zeros(n_x*p, n_x);                      %State prediction matrix
Gamma = zeros(n_x*p, m);                      %Control influence matrix

%Filling the prediction matrices
for i = 1:p
    Phi(n_x*i-2 : n_x*i, :) = Ad^i;                      
    for j = 1:min(i, m)
        Gamma(n_x*i - 2:n_x*i, j) = Ad^(i - j) * Bd;
    end
end


%Constraints:
U_max = 6000;                                        %Maximum input
U_min = -6000;                                       %Minimum input
delta_U_max = 200;                                   %Maximum change in input
x_max = [1; 1000; 1000];                             %Maximum state values
x_min = [-1; -1000; -1000];                          %Minimum state values

predicted_states = zeros(n_x, p, steps);             %3D array to store predicted states
control_inputs = zeros(m, steps);                    %Store control inputs

u_opt = zeros(m, 1);                                 %Optimal control vector initialization


%% MPC Loop
for k = 1:steps    
    current_time = k * Ts;
    
    %% IMPORTANT!
    %Following reference variables are named randomly
    %Additional letters don't have any specific meaning
    reft = zeros(n_x * p, 1);
    
    %Sequence of step reference changes:
    if current_time >= 1
        reft = 1 * ones(n_x * p, 1);  
    end
%     if current_time >= 1.14
%         reft = 0.25 * ones(n_x * p, 1);  
%     end
%     if current_time >= 1.28
%         reft = 0.7 * ones(n_x * p, 1);  
%     end
%     if current_time >= 1.42
%         reft = 0 * ones(n_x * p, 1);  
%     end
%     if current_time >= 1.56
%         reft = -0.5 * ones(n_x * p, 1);  
%     end
%     if current_time >= 1.7
%         reft = -1 * ones(n_x * p, 1);  
%     end
%     if current_time >= 1.84
%         reft = -0.25 * ones(n_x * p, 1);  
%     end
%     if current_time >= 1.98
%         reft = -0.75 * ones(n_x * p, 1);  
%     end
%     if current_time >= 2.12
%         reft = 0.5 * ones(n_x * p, 1);  
%     end
%     if current_time >= 2.26
%         reft = 0 * ones(n_x * p, 1);  
%     end
    
    
    %% Loading reference from the sEMG_simulation.m 
    %Comment if not needed
%     ref = load('fft_smoothed_data.mat');
%     ref = ref.smoothed_emg;
%     len = length(ref);
% 
%     %Vector manipulation; necessary for quadprog argument definition
%     new_len = 250;
%     indices = round(linspace(1, len, new_len));
%     refs = ref(indices)';
%     time_steps = length(refs); 
%     refk = zeros(250, time_steps);
%     refk(:, 1) = refs(k) * ones(250, 1);
%     refk = refk(:, 1);
%     indices = round(linspace(1, 250, 120));
%     refkk = refk(indices);
    
    
    

    %% MPC setup for quadprog
    x_pred = Phi * x + Gamma * u_opt;                       %(n_x*p) X (n_x) + (n_x*p) X (m*n_u)
    predicted_states(:, :, k) = reshape(x_pred, [n_x, p]);

    %Defining quadratic cost matrix H:
    H = Gamma' * Q_bar * Gamma + R_bar;                     %m X m

    %Defining linear cost vector f:
    f = (Gamma' * Q_bar * (Phi * x - reft))';               %HERE REPLACE THE reft VARIABLE WITH ANOTHER REFERENCE IF NEEDED!
                                                            %refkk FOR IMPORTED, reft FOR CUSTOMLY DEFINED
                                                            
    %Adjust bounds for states:
    b_x_upper = repmat(x_max, p, 1) - Phi * x;              %Adjusted upper state bounds (for state constraints)
    b_x_lower = repmat(x_min, p, 1) - Phi * x;              %Adjusted lower state bounds (for state constraints)

    %Combining constraints:
    F_total = [Gamma; -Gamma; eye(m); -eye(m)];             %Input constraints
    F_x = eye(n_x * p);                                     
    F_total = [F_x * Gamma; -F_x * Gamma; eye(m); -eye(m)]; %State constraints

    b_total = [b_x_upper; -b_x_lower; U_max * ones(m, 1); -U_min * ones(m, 1)];
    

    %% Solving the quadratic programming problem:
    %options = optimset('Display', 'off');
    u_opt = quadprog(H, f, F_total, b_total, [], [], [], [], []);  

    control_inputs(:, k) = u_opt(1:m);                      %Storing control inputs for the horizon
    u = u_opt(1);                                           %Extracting the first control input
    x = Ad * x + Bd * u;                                    %Applying the control input and update the state

    %Saving trajectories:
    x_trajectory(:, k) = x;
    u_trajectory(k) = u;

    %Updating previous input:
    u_prev = u;
    
    disp('State Prediction Error:');
    disp(norm(Phi * x - ref));
    disp('Current step:');
    disp(k);
end

time = (0:steps-1) * Ts + 2*Ts;

%% Extracting response parameters:
displacement = x_trajectory(1, :);    %Extracting the first row of response matrix (displacement)

steady_state_error = abs(displacement(end) - ref(end));  %Calculating tracking error
overshoot = 100*( (max(displacement) - displacement(end)) / displacement(end) ); %Calculating overshoot [%]
%Calculating rising time:
%Rising edge detection
step_idx = find(diff(ref) ~= 0, 1, 'first') + 1;
step_time = t(step_idx);
%Step value:
step_size = ref(end) - ref(step_idx - 1);           %Final value - initial value
%Thresholds:
lower_thresh = ref(step_idx - 1) + 0.1 * step_size;
upper_thresh = ref(step_idx - 1) + 0.9 * step_size;

%Finding where displacement crosses 10% and 90% AFTER step:
idx_start = find(displacement(step_idx:end) >= lower_thresh, 1, 'first') + step_idx - 1;
idx_end   = find(displacement(step_idx:end) >= upper_thresh, 1, 'first') + step_idx - 1;

%Computing rise time:
if ~isempty(idx_start) && ~isempty(idx_end)
    rise_time = time(idx_end) - time(idx_start);
else
    rise_time = NaN;
end


%% Plotting results
time = (0:steps-1) * Ts + 2*Ts;
%xlqr can be replaced with reference signal; xlqr left for comparison
figure;
subplot(3, 1, 1);
plot(time, x_trajectory(1, :), 'b', 'LineWidth', 2, 'DisplayName', 'Linear displacement MPC');      %1st state variable
hold on;
plot(t, xlqr(:, 1), 'r--', 'LineWidth', 1, 'DisplayName', 'Linear displacement LQR'); 
title('Linear displacement response');
ylabel('x_1 [m]');
xlim([0, 5]);
%ylim([0, 0.6]);
grid on;
legend;

subplot(3, 1, 2);
plot(time, x_trajectory(2, :), 'g', 'LineWidth', 2, 'DisplayName', 'Linear speed MPC');             %2nd state variable
hold on;
plot(t, xlqr(:, 2), 'k--', 'LineWidth', 1, 'DisplayName', 'Linear speed LQR'); 
title('Linear speed response');
ylabel('x_2 [m/s]');
xlim([0, 5]);
grid on;
legend;

subplot(3, 1, 3);
plot(time, x_trajectory(3, :), 'm', 'LineWidth', 2, 'DisplayName', 'Air flow MPC');                 %3rd state variable
hold on;
plot(t, xlqr(:, 3), 'c--', 'LineWidth', 1, 'DisplayName', 'Air flow LQR'); 
title('Air flow response');
xlabel('Time [s]');
ylabel('x_3 [m^3/s]');
xlim([0, 5]);
grid on;
legend;
