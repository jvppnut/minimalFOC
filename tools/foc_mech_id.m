%% foc_mech_id.m
% Least-squares identification of mechanical parameters (J, Dm, Tc) from a
% logged +/- i_q square-wave test (CSV exported from foc_scope.py's Save CSV),
% with a forward simulation of the fitted model overlaid on the measured
% velocity to sanity-check the fit.
%
% Model:
%   T(t) = J * domega/dt + Dm * omega + Tc * sign(omega)
%   T(t) = Kt * i_q(t)      (current loop settles in ~1 ms, far faster than
%                             mechanical dynamics, so i_q(t) is ~= applied
%                             torque/Kt at all times)
%
% No near-zero-speed exclusion: the Coulomb term Tc*sign(omega) is assumed
% to dominate friction near rest, so the whole trajectory (including every
% zero-crossing of a +/- square wave) is used in the fit as-is.
%
% Requires only base MATLAB (polyfit/polyval/polyder, lsqnonneg) - no toolboxes.

clear; clc;

%% ---- User settings ----
csv_file  = 'id_data.csv';   % path to CSV exported from foc_scope.py
Kt        = 0.1155;          % Nm/A — Kt = 1.5 * pole_pairs * lambda_pm
sg_window = 41;               % local-fit window (samples, odd) — tune to noise level
sg_order  = 3;                % local polynomial order (2 = quadratic, 3 = cubic)

%% ---- Load data ----
data  = readtable(csv_file);
t     = data.t;
omega = data.omega_mech;
iq    = data.i_q_ref;

Ts = mean(diff(t));   % foc_scope.py increments t by exactly 1/SAMPLE_RATE per
                       % frame, so this is an exact, non-jittery sample period

%% ---- Noncausal Savitzky-Golay smoothing + derivative ----
% Fits a local polynomial to a window CENTERED on each sample (using both
% past and future data — noncausal, fine here since this is offline/logged
% data) and takes the derivative directly from the fitted polynomial's
% slope term, rather than smoothing first and then finite-differencing the
% result. This avoids the extra noise amplification that a separate
% smooth-then-gradient cascade adds, and was needed because domega_dt was
% coming out noisy and near-zero with the smooth+gradient approach.
[omega_smooth, domega_dt] = sgolay_fit_and_diff(omega, Ts, sg_window, sg_order);

%% ---- Torque from i_q_ref ----
% Using the reference rather than measured i_q: A/B testing the two showed
% the same transient mismatch either way (current-loop rise time is ~1 ms,
% negligible against the ~100+ ms mechanical time constants here), so
% i_q_ref is preferred — it's noise-free, whereas measured i_q carries ADC/
% current-sensing noise straight into the simulation's forcing input.
% Ld == Lq for this motor (surface PM), so the reluctance term (Ld-Lq)*id*iq
% is exactly zero regardless of id — Kt*iq is the exact torque model here.
Torque = Kt * iq;

%% ---- Least-squares fit: T = J*domega/dt + Dm*omega + Tc*sign(omega) ----
% J, Dm, Tc are all physically non-negative (inertia, viscous/Coulomb
% friction magnitudes) — use lsqnonneg (base MATLAB, no toolbox) instead of
% the unconstrained backslash so noisy/ill-conditioned data can't fit a
% negative "inertia" or friction coefficient.
A = [domega_dt, omega_smooth, sign(omega_smooth)];
x = lsqnonneg(A, Torque);    % [J; Dm; Tc], all >= 0
J_fit  = x(1);
Dm_fit = x(2);
Tc_fit = x(3);

fprintf('Fitted mechanical parameters:\n');
fprintf('  J  = %.6g kg*m^2\n', J_fit);
fprintf('  Dm = %.6g N*m*s/rad\n', Dm_fit);
fprintf('  Tc = %.6g N*m  (Coulomb friction)\n', Tc_fit);
for name_val = {{'J', J_fit}, {'Dm', Dm_fit}, {'Tc', Tc_fit}}
    nm = name_val{1}{1}; v = name_val{1}{2};
    if v == 0
        fprintf('  NOTE: %s hit the >=0 constraint boundary — unconstrained fit wanted it negative.\n', nm);
    end
end

if Dm_fit > 0
    tau_mech = J_fit / Dm_fit;
    fprintf('  tau_mech = J/Dm = %.6g s\n', tau_mech);
else
    fprintf('  tau_mech = J/Dm undefined (Dm fit to 0 — Coulomb-dominated friction)\n');
end

%% ---- Regression fit quality (instantaneous torque balance) ----
Torque_pred = A * x;
resid  = Torque - Torque_pred;
SS_res = sum(resid.^2);
SS_tot = sum((Torque - mean(Torque)).^2);
R2 = 1 - SS_res/SS_tot;
fprintf('  Torque-balance R^2 = %.4f\n', R2);

%% ---- Forward-simulate the fitted model against the measured input ----
% Integrates domega/dt = (T - Dm*omega - Tc*sign(omega)) / J using the
% measured Kt*i_q as the forcing input, starting from the measured initial
% speed. This is a stronger check than the regression R^2 above: any bias
% or drift in J/Dm/Tc accumulates over time here instead of averaging out
% sample-by-sample.


omega_sim = simulate_mech(Torque, Ts, omega(1), J_fit, Dm_fit, Tc_fit);

sim_resid = omega - omega_sim;
sim_RMSE  = sqrt(mean(sim_resid.^2));
sim_R2    = 1 - sum(sim_resid.^2) / sum((omega - mean(omega)).^2);
fprintf('  Simulation vs measured: RMSE = %.3f rad/s, R^2 = %.4f\n', sim_RMSE, sim_R2);

%% ---- Diagnostic plots ----
figure('Name', 'Mechanical ID');
grid on;
subplot(2,2,1);
plot(t, omega, 'Color', [0.6 0.6 0.6]); hold on;
plot(t, omega_smooth, 'b', 'LineWidth', 1.0);
plot(t, omega_sim, 'g--', 'LineWidth', 1.2);
xlabel('t (s)'); ylabel('\omega_{mech} (rad/s)');
legend('raw', 'smoothed', 'simulated (fit)', 'Location', 'best');
title('Mechanical velocity: measured vs. simulated');

subplot(2,2,2);
plot(t, domega_dt, 'm');
xlabel('t (s)'); ylabel('d\omega/dt (rad/s^2)');
title('Angular acceleration (smoothed derivative)');

subplot(2,2,3);
plot(t, Torque, 'k.'); hold on;
plot(t, Torque_pred, 'r.');
xlabel('t (s)'); ylabel('Torque (N*m)');
legend('T = K_t i_q (measured)', 'model fit', 'Location', 'best');
title(sprintf('Torque balance fit \x2014 R^2 = %.4f', R2));

subplot(2,2,4);
plot(t, sim_resid, 'Color', [0.85 0.33 0.1]);
xlabel('t (s)'); ylabel('\omega - \omega_{sim} (rad/s)');
title(sprintf('Simulation residual \x2014 RMSE = %.3f rad/s, R^2 = %.4f', sim_RMSE, sim_R2));

%% ---- Validation against independent datasets ----
% Cross-checks the fitted J/Dm/Tc against separate logs in ValidationData/
% (sitting next to this script) that were NOT used for the fit above —
% including runs that didn't go well as identification attempts themselves,
% but are still valid experimental omega_mech/i_q_ref traces to simulate
% against and see how the fitted model generalizes.
script_dir = fileparts(mfilename('fullpath'));
val_dir    = fullfile(script_dir, 'ValidationData');
val_files  = dir(fullfile(val_dir, 'foc_log*.csv'));

if isempty(val_files)
    fprintf('\nNo validation files found in %s\n', val_dir);
else
    fprintf('\n--- Validation against %d dataset(s) in ValidationData/ ---\n', numel(val_files));
    n_val  = numel(val_files);
    n_cols = ceil(sqrt(n_val));
    n_rows = ceil(n_val / n_cols);
    figure('Name', 'Mechanical ID — Validation');

    for i = 1:n_val
        vfile  = fullfile(val_files(i).folder, val_files(i).name);
        vdata  = readtable(vfile);
        vt     = vdata.t;
        vomega = vdata.omega_mech;
        vTorque = Kt * vdata.i_q_ref;
        vTs    = mean(diff(vt));

        vomega_sim = simulate_mech(vTorque, vTs, vomega(1), J_fit, Dm_fit, Tc_fit);

        vresid = vomega - vomega_sim;
        vRMSE  = sqrt(mean(vresid.^2));
        vR2    = 1 - sum(vresid.^2) / sum((vomega - mean(vomega)).^2);
        fprintf('  %s: RMSE = %.3f rad/s, R^2 = %.4f\n', val_files(i).name, vRMSE, vR2);

        subplot(n_rows, n_cols, i);
        plot(vt, vomega, 'Color', [0.6 0.6 0.6]); hold on;
        plot(vt, vomega_sim, 'g--', 'LineWidth', 1.0);
        xlabel('t (s)'); ylabel('\omega_{mech} (rad/s)');
        legend('measured', 'simulated', 'Location', 'best');
        title(sprintf('%s \x2014 RMSE=%.2f rad/s, R^2=%.3f', val_files(i).name, vRMSE, vR2), ...
            'Interpreter', 'none');
    end
end

%% ---- Local functions ----
function omega_sim = simulate_mech(Torque, Ts, omega0, J, Dm, Tc)
% Forward-simulates domega/dt = (T - Dm*omega - Tc*sign(omega)) / J against
% a given torque time series, via forward Euler at the series' own Ts
% (stable/accurate here since Ts is far smaller than the mechanical time
% constant of interest).
    N = length(Torque);
    omega_sim = zeros(N, 1);
    omega_sim(1) = omega0;
    for n = 1:N-1
        domega = (Torque(n) - Dm*omega_sim(n) - Tc*sign(omega_sim(n))) / J;
        omega_sim(n+1) = omega_sim(n) + domega * Ts;
    end
end
function [y_smooth, dydt] = sgolay_fit_and_diff(y, Ts, window, order)
% Noncausal Savitzky-Golay smoothing + derivative.
%
% For each sample k, fits a polynomial of the given order to the samples
% in a window CENTERED on k (shrinking near the edges), using local time
% offsets so t=0 is always the window center. The smoothed value and the
% derivative both come from that one fitted polynomial (evaluated, and
% differentiated-then-evaluated, at t=0) rather than from a separate
% smoothing pass followed by finite differencing — this is the standard
% Savitzky-Golay differentiator, just built from polyfit/polyval/polyder
% instead of the Signal Processing Toolbox's sgolay().
    n = length(y);
    half = floor(window/2);
    y_smooth = zeros(n, 1);
    dydt     = zeros(n, 1);
    for k = 1:n
        i0  = max(1, k-half);
        i1  = min(n, k+half);
        idx = (i0:i1)';
        tloc = (idx - k) * Ts;             % local time axis, centered at 0
        p  = polyfit(tloc, y(idx), order);
        y_smooth(k) = polyval(p, 0);
        dp = polyder(p);
        dydt(k) = polyval(dp, 0);
    end
end
