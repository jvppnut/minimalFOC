%% foc_velocity_verify.m
% Closed-loop verification of the velocity control loop: simulates the
% velocity PI controller + identified mechanical plant together, driven by
% the SAME omega_ref trace logged during an actual closed-loop velocity-mode
% experiment (CSV from foc_scope.py's Save CSV, run in Velocity mode), and
% compares the simulated omega_mech against the measured one.
%
% This mirrors FOC_VelocityCtrlComputation + FOC_PID_Update exactly
% (foc.c / core/math/foc_pid.c), including the two-stage anti-windup clamp
% (integrator clamped, then P+I clamped again):
%   error        = omega_ref - omega_sim
%   p_out        = Kp * error
%   integrator   = clamp(integrator + Ki*error, -i_lim, i_lim)
%   i_q_ref_sim  = clamp(p_out + integrator, -i_lim, i_lim)
%   domega/dt    = (Kt*i_q_ref_sim - Dm*omega_sim - Tc*sign(omega_sim)) / J
%
% The current loop itself is NOT simulated (i_q_ref_sim drives the plant
% directly through Kt) — its ~1 ms settling time was already established as
% negligible against these mechanical time constants during mechanical ID
% (tools/foc_mech_id.m), so this checks the velocity-loop design + mechanical
% model together, not the current loop.
%
% Requires only base MATLAB.

clear; clc;

%% ---- User settings ----
csv_file = 'velocity_data2.csv';   % path to CSV exported from foc_scope.py (Velocity mode run)

% Mechanical parameters — from tools/foc_mech_id.m identification
J  = 6.498115558493604e-5;   % kg*m^2
Dm = 9.410621428371238e-5;   % N*m*s/rad
Tc = 0.013139614148598;      % N*m (Coulomb friction)

% Motor/electrical — must match foc_config.h
pole_pairs = 14;
lambda_pm  = 0.0055;              % Wb
Kt = 1.5 * pole_pairs * lambda_pm;  % Nm/A

% Velocity loop design — must match foc_config.h FOC_VELOCITY_WN/ZETA
wn   = 200.0;   % rad/s
zeta = 1.0;

% Velocity PID output clamp — must match main.c's i_lim for foc_pid_speed
% i_lim = 2.0 * 0.7;   % A
i_lim = 6.0 * 0.7;   % A
% i_lim = 20;

%% ---- Derived gains (mirrors the formula in foc_config.h / main.c) ----
Kp_w      = (2*zeta*wn*J - Dm) / Kt;
Ki_w_cont = J * wn^2 / Kt;

%% ---- Load experimental data ----
data      = readtable(csv_file);
t         = data.t;
omega     = data.omega_mech;
omega_ref = data.omega_ref;

Ts   = mean(diff(t));   % foc_scope.py increments t by exactly 1/SAMPLE_RATE per frame
Ki_w = Ki_w_cont * Ts;  % discrete (forward-Euler) scaling, same convention as main.c

fprintf('Kp_w = %.6g, Ki_w (discrete) = %.6g, i_lim = %.3g A\n', Kp_w, Ki_w, i_lim);

%% ---- Closed-loop simulation ----
[omega_sim, iq_ref_sim] = simulate_velocity_loop(omega_ref, omega(1), Ts, ...
    Kp_w, Ki_w, i_lim, J, Dm, Tc, Kt);

%% ---- Fit quality ----
resid = omega - omega_sim;
RMSE  = sqrt(mean(resid.^2));
R2    = 1 - sum(resid.^2) / sum((omega - mean(omega)).^2);
fprintf('Velocity loop verification: RMSE = %.3f rad/s, R^2 = %.4f\n', RMSE, R2);

%% ---- Plots ----
figure('Name', 'Velocity Loop Verification');

subplot(2,1,1);
plot(t, omega_ref, 'k--', 'LineWidth', 1.0); hold on;
plot(t, omega, 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0);
plot(t, omega_sim, 'g-', 'LineWidth', 1.2);
xlabel('t (s)'); ylabel('\omega_{mech} (rad/s)');
legend('\omega_{ref}', 'measured', 'simulated', 'Location', 'best');
title(sprintf('Velocity closed-loop: measured vs. simulated \x2014 RMSE=%.3f rad/s, R^2=%.4f', RMSE, R2));

subplot(2,1,2);
plot(t, data.i_q_ref, 'k'); hold on;
plot(t, iq_ref_sim, 'r');
xlabel('t (s)'); ylabel('i_{q,ref} (A)');
legend('measured (real velocity-loop output)', 'simulated', 'Location', 'best');
title('Velocity PI output (i_{q,ref}): measured vs. simulated');

%% ---- Local functions ----
function [omega_sim, iq_ref_sim] = simulate_velocity_loop(omega_ref, omega0, Ts, Kp, Ki, i_lim, J, Dm, Tc, Kt)
% Forward-simulates the velocity PI controller (with the same two-stage
% anti-windup clamp as FOC_PID_Update) driving the mechanical plant through
% Kt, using the logged omega_ref as the only external input.
    N = length(omega_ref);
    omega_sim  = zeros(N, 1);
    iq_ref_sim = zeros(N, 1);
    omega_sim(1) = omega0;
    integrator = 0.0;

    for n = 1:N-1
        err   = omega_ref(n) - omega_sim(n);
        p_out = Kp * err;
        integrator = min(max(integrator + Ki*err, -i_lim), i_lim);
        iq_ref_sim(n) = min(max(p_out + integrator, -i_lim), i_lim);

        domega = (Kt*iq_ref_sim(n) - Dm*omega_sim(n) - Tc*sign(omega_sim(n))) / J;
        omega_sim(n+1) = omega_sim(n) + domega * Ts;
    end

    % Compute i_q_ref for the final sample too (loop above only goes to N-1)
    err = omega_ref(N) - omega_sim(N);
    integrator = min(max(integrator + Ki*err, -i_lim), i_lim);
    iq_ref_sim(N) = min(max(Kp*err + integrator, -i_lim), i_lim);
end
