%Task 3
G = tf(1088.64, [1 72 5184 0 0]);
t = 0:0.01:100;
figure;
impulse(G, t);
title('Impulse Response of Open-loop Transfer Function G(s)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

figure;
step(G, t);
title('Step Response of Open-loop Transfer Function G(s)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

% Task 6
Sys = tf(1088.64, [1 72 5184 0 1088.64]);
tnew = 0:0.01:1000;

figure;
impulse(Sys, tnew);
title('Impulse Response of Closed-loop Transfer Function');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

figure;
step(Sys, tnew);
title('Step Response of Closed-loop Transfer Function');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

%Task 8
figure;
rlocus(G);
title('Root Locus of the System');
grid on;

%Task 9
figure;
margin(G);
title('Bode Plot with Gain and Phase Margins');
grid on;

% Task 11
kd = 11.49; % Derivative gain
kp = 31.83; % Proportional gain
PD_controller = tf([kd kp], 1); % PD controller transfer function
Lag_for_PD = tf([1 0.03], [1 0.002]);
PD_Lag = PD_controller * Lag_for_PD;

% Closed-loop transfer function
T = feedback(G * PD_Lag, 1);

%Input: Step 
u_step = ones(size(t));

% Input: Ramp
u_ramp = t; 
y_ramp = lsim(T, u_ramp, t);

% Input: Parabolic
u_para = 0.5 * t.^2;
y_para = lsim(T, u_para, t);

% Plot Step Response
figure;
plot(t, u_step, 'b', 'LineWidth', 1.5); hold on;
step(T, t);
title('Step Input Response');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Input', 'Output');
grid on;

% Plot Ramp Response
figure;
plot(t, u_ramp, 'b', 'LineWidth', 1.5); hold on;
plot(t, y_ramp, 'r', 'LineWidth', 1.5);
title('Ramp Input Response');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Input', 'Output');
grid on;

% Plot Parabolic Response
figure;
plot(t, u_para, 'b', 'LineWidth', 1.5); hold on;
plot(t, y_para, 'r', 'LineWidth', 1.5);
title('Parabolic Input Response');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Input', 'Output');
grid on; 

% Calculate Steady-State Errors
syms s t_s
G_open_loop = 1088.64 / (s^4 + 72*s^3 + 5184*s^2);
PD_Lag_sys = (11.49*s + 31.83) * (s + 0.03) / (s + 0.002);
rt1 = 1*t_s/t_s;
rt2 = t_s;
rt3 = 0.5*t_s^2;
r1 = laplace(rt1);
r2 = laplace(rt2);
r3 = laplace(rt3);

% Step Error
e_step = limit(s*r1 / (1+ G_open_loop*PD_Lag_sys),0);

% Ramp Error
e_ramp = limit(s*r2 / (1+ G_open_loop*PD_Lag_sys),0);

% Parabolic Error
e_parabolic = limit(s*r3 / (1+ G_open_loop*PD_Lag_sys),0);

disp('Steady-State Errors:');
disp(['Step Input Error: ', char(e_step)]);
disp(['Ramp Input Error: ', char(e_ramp)]);
disp(['Parabolic Input Error: ', char(e_parabolic)]);

% Task 13 (Change the value of kd and kp)
kd = 20; % Derivative gain
kp = 31.83; % Proportional gain
PD_controller = tf([kd kp], 1); % PD controller transfer function
Lag_for_PD = tf([1 0.03], [1 0.002]);
PD_Lag = PD_controller * Lag_for_PD;

% Closed-loop transfer function
T = feedback(G * PD_Lag, 1);

%Input: Step 
u_step = ones(size(t));

% Input: Ramp
u_ramp = t; 
y_ramp = lsim(T, u_ramp, t);

% Input: Parabolic
u_para = 0.5 * t.^2;
y_para = lsim(T, u_para, t);

% Plot Step Response
figure;
plot(t, u_step, 'b', 'LineWidth', 1.5); hold on;
step(T, t);
title('Step Input Response');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Input', 'Output');
grid on;

% Plot Ramp Response
figure;
plot(t, u_ramp, 'b', 'LineWidth', 1.5); hold on;
plot(t, y_ramp, 'r', 'LineWidth', 1.5);
title('Ramp Input Response');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Input', 'Output');
grid on;

% Plot Parabolic Response
figure;
plot(t, u_para, 'b', 'LineWidth', 1.5); hold on;
plot(t, y_para, 'r', 'LineWidth', 1.5);
title('Parabolic Input Response');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Input', 'Output');
grid on;

% Task 15 
figure;
rlocus(G * PD_Lag);
title('Root Locus with PD Lag');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;

% Task 16
figure;
margin(G * PD_Lag);
title('Bode Plot with PD Lag Controller');
grid on;

% Task 18
kc = 63.4;
Lead = tf([kc 1.35*kc] , [1 5.15]);
Lag = tf([1 0.029] , [1 0.001]);
Lead_Lag_Compensator = Lead * Lag;
TF = feedback(G * Lead_Lag_Compensator, 1);

%Input: Step 
u_step = ones(size(t));

% Input: Ramp
u_ramp = t; 
y_ramp = lsim(TF, u_ramp, t);

% Input: Parabolic
u_para = 0.5 * t.^2;
y_para = lsim(TF, u_para, t);

% Calculate Steady-State Errors
syms s t_s
G_open_loop = 1088.64 / (s^4 + 72*s^3 + 5184*s^2);
Lead_sys = (63.4*s + 85.59) / (s + 5.15); 
Lag_sys = (s + 0.029) / (s + 0.001);
Lead_Lag_sys = Lead_sys * Lag_sys; 
rt1 = 1*t_s/t_s;
rt2 = t_s;
rt3 = 0.5*t_s^2;
r1 = laplace(rt1);
r2 = laplace(rt2);
r3 = laplace(rt3);

% Step Error
e_step = limit(s*r1 / (1+ G_open_loop*Lead_Lag_sys),0);

% Ramp Error
e_ramp = limit(s*r2 / (1+ G_open_loop*Lead_Lag_sys),0);

% Parabolic Error
e_parabolic = limit(s*r3 / (1+ G_open_loop*Lead_Lag_sys),0);

disp('Steady-State Errors:');
disp(['Step Input Error: ', char(e_step)]);
disp(['Ramp Input Error: ', char(e_ramp)]);
disp(['Parabolic Input Error: ', char(e_parabolic)]);

% Plot Step Response
figure;
plot(t, u_step, 'b', 'LineWidth', 1.5); hold on;
step(TF, t);
title('Step Input Response');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Input', 'Output');
grid on;

% Plot Ramp Response
figure;
plot(t, u_ramp, 'b', 'LineWidth', 1.5); hold on;
plot(t, y_ramp, 'r', 'LineWidth', 1.5);
title('Ramp Input Response');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Input', 'Output');
grid on;

% Plot Parabolic Response
figure;
plot(t, u_para, 'b', 'LineWidth', 1.5); hold on;
plot(t, y_para, 'r', 'LineWidth', 1.5);
title('Parabolic Input Response');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Input', 'Output');
grid on;

% Task 20 (Change kc and shift zeros and poles) 
kc = 80;
Lead = tf([kc 0.29*kc] , [1 4.97]);
Lag = tf([1 0.029] , [1 0.001]);
Lead_Lag_Compensator = Lead * Lag;
TF = feedback(G * Lead_Lag_Compensator, 1);

%Input: Step 
u_step = ones(size(t));

% Input: Ramp
u_ramp = t; 
y_ramp = lsim(TF, u_ramp, t);

% Input: Parabolic
u_para = 0.5 * t.^2;
y_para = lsim(TF, u_para, t);

% Plot Step Response
figure;
plot(t, u_step, 'b', 'LineWidth', 1.5); hold on;
step(TF, t);
title('Step Input Response');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Input', 'Output');
grid on;

% Plot Ramp Response
figure;
plot(t, u_ramp, 'b', 'LineWidth', 1.5); hold on;
plot(t, y_ramp, 'r', 'LineWidth', 1.5);
title('Ramp Input Response');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Input', 'Output');
grid on;

% Plot Parabolic Response
figure;
plot(t, u_para, 'b', 'LineWidth', 1.5); hold on;
plot(t, y_para, 'r', 'LineWidth', 1.5);
title('Parabolic Input Response');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Input', 'Output');
grid on;

% Task 22 
figure;
rlocus(G * Lead_Lag_Compensator);
title('Root Locus with Lead Lag Compensator');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;

% Task 23
figure;
margin(G * Lead_Lag_Compensator);
title('Bode Plot with Lead Lag Compensator');
grid on;