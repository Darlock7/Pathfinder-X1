function propOut = propulsionAnalysis(propIn)
% propulsionAnalysis
%
% Purpose:
%   Propulsion analysis for a single motor/prop combination.
%
% Modes (selected automatically):
%   1) Preliminary  — usePrelimModel=true : linear T/T0 = 1 - V/V0
%   2) Dat-file     — propIn.datFile set  : reads J,CT,CP from a generic
%                     .dat file (UIUC/APC download) and solves torque balance
%   3) APC PER3     — fallback             : existing PER3_*.txt parser
%
% Class definitions (propulsion slides):
%   J   = V / (n*D)                 advance ratio              [-]
%   CT  = T / (rho * n^2 * D^4)    thrust coefficient         [-]
%   CP  = P / (rho * n^3 * D^5)    power coefficient          [-]
%   CQ  = Q / (rho * n^2 * D^5)    torque coeff = CP/(2*pi)   [-]
%   eta = T*V / (Q*omega) = J*CT/CP propulsive efficiency      [-]
%
% Three plots generated (matching class slide 15):
%   1) Thrust T [N] vs flight speed V [m/s]
%   2) Current I [A] vs flight speed V [m/s]
%   3) Non-dimensional: eta, CT, CP vs advance ratio J

%% ── Unpack inputs ─────────────────────────────────────────────────────────
rho    = propIn.rho;
KV     = propIn.KV;
Rm     = propIn.Rm;
I0     = propIn.I0;
Vbat   = propIn.Vbat;

I_max  = inf;
if isfield(propIn,'I_max'), I_max = propIn.I_max; end

propName  = propIn.propName;
D_in      = propIn.D_in;
pitch_in  = propIn.pitch_in;
V_vec     = propIn.V_vec_mps(:);
Nv        = numel(V_vec);
D_m       = D_in * 0.0254;          % [m]

usePrelimModel = true;
if isfield(propIn,'usePrelimModel'), usePrelimModel = propIn.usePrelimModel; end

%% ── Motor constants ───────────────────────────────────────────────────────
Kt = 60 / (2*pi*KV);    % torque constant  [N*m/A]
Ke = Kt;                % back-EMF const   [V*s/rad]

Im_fn  = @(omega) max((Vbat - Ke*omega)/Rm, I0);          % current [A]
Qm_fn  = @(omega) Kt * (((Vbat - Ke*omega)/Rm) - I0);     % motor torque [N*m]

%% ══════════════════════════════════════════════════════════════════════════
%  MODE 1 — Preliminary linear model
%% ══════════════════════════════════════════════════════════════════════════
if usePrelimModel
    T_static_N = propIn.T_static_N;
    V0_mps     = propIn.V0_mps;

    T_vec_N = T_static_N * max(0, 1 - V_vec./V0_mps);

    pitch_m   = pitch_in * 0.0254;
    n_rps_est = V0_mps / max(pitch_m,1e-6);
    omega_est = 2*pi*n_rps_est;
    I_est     = max((Vbat - Ke*omega_est)/Rm, I0);

    I_vec_A      = I_est * ones(size(V_vec));
    I_vec_A(T_vec_N <= 0) = I0;
    P_elec_vec_W = Vbat .* I_vec_A;
    P_prop_vec_W = T_vec_N .* V_vec;

    eta_vec = nan(size(V_vec));
    ok = P_elec_vec_W > 0;
    eta_vec(ok) = P_prop_vec_W(ok) ./ P_elec_vec_W(ok);

    propOut = packOutput('preliminary', propName, KV, Rm, I0, Vbat, I_max, ...
        D_in, pitch_in, V_vec, T_vec_N, I_vec_A, P_elec_vec_W, P_prop_vec_W, eta_vec, ...
        nan(Nv,1), nan(Nv,1), nan(Nv,1), nan(Nv,1), nan(Nv,1), nan(Nv,1));

    printSummary(propOut, 'PRELIMINARY MODEL');
    plotThrust(V_vec, T_vec_N, propName);
    plotCurrent(V_vec, I_vec_A, I_max, propName);
    return;
end

%% ── Common vector initialisation ──────────────────────────────────────────
T_vec_N      = nan(Nv,1);
I_vec_A      = nan(Nv,1);
P_elec_vec_W = nan(Nv,1);
P_prop_vec_W = nan(Nv,1);
rpm_vec      = nan(Nv,1);
J_vec        = nan(Nv,1);
CT_vec       = nan(Nv,1);
CP_vec       = nan(Nv,1);
eta_prop_vec = nan(Nv,1);
Qp_vec_Nm    = nan(Nv,1);

omega_guess = 9000 * 2*pi/60;   % initial guess ~9000 RPM

%% ══════════════════════════════════════════════════════════════════════════
%  MODE 2 — Generic .dat file (UIUC / APC download)
%% ══════════════════════════════════════════════════════════════════════════
if isfield(propIn,'datFile') && ~isempty(propIn.datFile)

    [J_dat, CT_dat, CP_dat] = parsePropDatFile(propIn.datFile);
    CQ_dat = CP_dat / (2*pi);

    F_CT = @(J) max(0, interp1(J_dat, CT_dat, J, 'linear','extrap'));
    F_CP = @(J) max(0, interp1(J_dat, CP_dat, J, 'linear','extrap'));
    F_CQ = @(J) max(0, interp1(J_dat, CQ_dat, J, 'linear','extrap'));

    for i = 1:Nv
        Vinf = V_vec(i);
        try
            balFun = @(omega) localBalanceDat(omega, Vinf, D_m, rho, Qm_fn, F_CQ);
            omega_sol = fzero(balFun, omega_guess);
            n_sol = omega_sol / (2*pi);
            if ~isfinite(n_sol) || n_sol <= 0, continue; end

            J_sol  = Vinf / (n_sol * D_m);
            CT_sol = F_CT(J_sol);
            CP_sol = F_CP(J_sol);
            T_sol  = CT_sol * rho * n_sol^2 * D_m^4;
            Qp_sol = (CP_sol/(2*pi)) * rho * n_sol^2 * D_m^5;
            I_sol  = Im_fn(omega_sol);

            if ~isfinite(T_sol), continue; end

            rpm_vec(i)      = 60 * n_sol;
            J_vec(i)        = J_sol;
            CT_vec(i)       = CT_sol;
            CP_vec(i)       = CP_sol;
            T_vec_N(i)      = max(T_sol, 0);
            Qp_vec_Nm(i)    = Qp_sol;
            I_vec_A(i)      = I_sol;
            P_elec_vec_W(i) = Vbat * I_sol;
            P_prop_vec_W(i) = T_sol * Vinf;

            if J_sol > 0 && CP_sol > 1e-9
                eta_prop_vec(i) = J_sol * CT_sol / CP_sol;
            end
            omega_guess = omega_sol;
        catch
        end
    end

    modeLabel = sprintf('DAT_%s', propName);

%% ══════════════════════════════════════════════════════════════════════════
%  MODE 3 — APC PER3 format
%  Reads CT and CP from columns 4-5 (present in both 8-col and 15-col files).
%  Density correction is handled correctly via CT/CP definitions — no
%  rho_ratio approximation needed.
%% ══════════════════════════════════════════════════════════════════════════
else
    % Accept any PER3 file via propIn.apcFile, otherwise fall back to
    % the two shipped legacy .txt files.
    if isfield(propIn,'apcFile') && ~isempty(propIn.apcFile)
        filename = propIn.apcFile;
    elseif isfield(propIn,'propName') && contains(lower(string(propIn.propName)),'mr')
        filename = 'PER3_10x45MR.txt';
    else
        filename = 'PER3_10x47SF.txt';
    end

    txt   = fileread(filename);
    lines = splitlines(string(txt));

    % Collect (n, J, CT, CP) tuples across all RPM sections
    RPM_all = []; n_all = []; J_all = []; CT_all = []; CP_all = [];
    currentRPM = NaN;

    for k = 1:length(lines)
        line = strtrim(lines(k));
        tokRPM = regexp(line,'PROP RPM =\s*([0-9]+)','tokens');
        if ~isempty(tokRPM)
            currentRPM = str2double(tokRPM{1}{1});
            continue;
        end
        if isnan(currentRPM), continue; end
        nums = sscanf(line,'%f');
        % Columns: 1=V(mph) 2=J 3=Pe 4=CT 5=CP  (both 8-col and 15-col formats)
        if numel(nums) >= 5
            J_v  = nums(2);
            CT_v = nums(4);
            CP_v = nums(5);
            if isfinite(J_v) && J_v >= 0 && isfinite(CT_v) && CT_v >= 0 && ...
               isfinite(CP_v) && CP_v > 0
                RPM_all(end+1,1) = currentRPM;  %#ok<AGROW>
                n_all(end+1,1)   = currentRPM/60;
                J_all(end+1,1)   = J_v;
                CT_all(end+1,1)  = CT_v;
                CP_all(end+1,1)  = CP_v;
            end
        end
    end

    if isempty(n_all)
        error('No valid CT/CP data parsed from: %s', filename);
    end

    % 2D interpolants over (n [rev/s], J) — matches professor''s Python approach
    F_CT_apc = scatteredInterpolant(n_all, J_all, CT_all, 'natural','nearest');
    F_CP_apc = scatteredInterpolant(n_all, J_all, CP_all, 'natural','nearest');

    for i = 1:Nv
        Vinf = V_vec(i);
        try
            % Torque balance: Qm(omega) = CQ(n,J) * rho * n^2 * D^5
            balFun = @(omega) localBalanceAPC(omega, Vinf, D_m, rho, Qm_fn, F_CP_apc);
            omega_sol = fzero(balFun, omega_guess);
            n_sol    = omega_sol / (2*pi);
            if ~isfinite(n_sol) || n_sol <= 0, continue; end

            J_sol  = Vinf / (n_sol * D_m);
            CT_sol = max(0, F_CT_apc(n_sol, J_sol));
            CP_sol = max(0, F_CP_apc(n_sol, J_sol));
            T_sol  = CT_sol * rho * n_sol^2 * D_m^4;
            Qp_sol = (CP_sol/(2*pi)) * rho * n_sol^2 * D_m^5;
            I_sol  = Im_fn(omega_sol);

            if ~isfinite(T_sol) || ~isfinite(Qp_sol), continue; end

            rpm_vec(i)      = 60 * n_sol;
            J_vec(i)        = J_sol;
            CT_vec(i)       = CT_sol;
            CP_vec(i)       = CP_sol;
            T_vec_N(i)      = T_sol;
            Qp_vec_Nm(i)    = Qp_sol;
            I_vec_A(i)      = I_sol;
            P_elec_vec_W(i) = Vbat * I_sol;
            P_prop_vec_W(i) = T_sol * Vinf;

            if J_sol > 0 && CP_sol > 1e-9
                eta_prop_vec(i) = J_sol * CT_sol / CP_sol;
            end
            omega_guess = omega_sol;
        catch
        end
    end

    modeLabel = sprintf('APC_%s', strrep(filename,'.txt',''));
end

%% ── Pack outputs ──────────────────────────────────────────────────────────
eta_elec = P_prop_vec_W ./ P_elec_vec_W;   % electrical efficiency = TV/(Vbat*I)

propOut = packOutput(modeLabel, propName, KV, Rm, I0, Vbat, I_max, ...
    D_in, pitch_in, V_vec, T_vec_N, I_vec_A, P_elec_vec_W, P_prop_vec_W, eta_elec, ...
    rpm_vec, J_vec, CT_vec, CP_vec, eta_prop_vec, Qp_vec_Nm);

I_valid = I_vec_A(isfinite(I_vec_A));
I_peak  = max(I_valid);
propOut.I_peak_A = I_peak;

printSummary(propOut, modeLabel);
if I_peak > I_max
    fprintf('*** WARNING: peak current %.1f A exceeds limit %.1f A ***\n', I_peak, I_max);
elseif I_peak > 40
    fprintf('*** WARNING: peak current %.1f A exceeds 40 A hard limit ***\n', I_peak);
else
    fprintf('Current check: OK  (peak = %.1f A <= %.1f A limit)\n', I_peak, I_max);
end
fprintf('============================================================\n\n');

%% ── Plot 1: Thrust vs Speed ───────────────────────────────────────────────
plotThrust(V_vec, T_vec_N, propName);

%% ── Plot 2: Current vs Speed ──────────────────────────────────────────────
plotCurrent(V_vec, I_vec_A, I_max, propName);

%% ── Plot 3: Non-dimensional performance (eta, CT, CP vs J) ───────────────
% Matches class slide 15 layout
valid = isfinite(J_vec) & isfinite(CT_vec) & isfinite(CP_vec);
if any(valid)
    figure('Name', sprintf('Prop Coefficients — %s', propName), ...
           'NumberTitle','off','Color','w');

    subplot(1,3,1);
    eta_ok = valid & isfinite(eta_prop_vec);
    if any(eta_ok)
        plot(J_vec(eta_ok), eta_prop_vec(eta_ok), 'g', 'LineWidth', 2);
    end
    grid on; box on;
    xlabel('Advance Ratio  J = V/(nD)  [-]');
    ylabel('\eta_{prop}  [-]');
    title('Propulsive Efficiency  \eta = J \cdot C_T / C_P');
    ylim([0 1]);

    subplot(1,3,2);
    plot(J_vec(valid), CT_vec(valid), 'b', 'LineWidth', 2);
    grid on; box on;
    xlabel('Advance Ratio  J = V/(nD)  [-]');
    ylabel('C_T = T / (\rho n^2 D^4)  [-]');
    title('Thrust Coefficient  C_T');

    subplot(1,3,3);
    plot(J_vec(valid), CP_vec(valid), 'r', 'LineWidth', 2);
    grid on; box on;
    xlabel('Advance Ratio  J = V/(nD)  [-]');
    ylabel('C_P = P / (\rho n^3 D^5)  [-]');
    title('Power Coefficient  C_P');

    sgtitle(sprintf('Propeller Performance — %s', propName), 'Interpreter','none');
end

end

%% ══════════════════════════════════════════════════════════════════════════
%  LOCAL FUNCTIONS
%% ══════════════════════════════════════════════════════════════════════════


function val = localBalanceAPC(omega, Vinf, D_m, rho, Qm_fn, F_CP_apc)
    % Torque balance using CP from PER3 file: Qm = CQ * rho * n^2 * D^5
    % CQ = CP / (2*pi)
    n = omega / (2*pi);
    if ~isfinite(n) || n <= 0, val = NaN; return; end
    J = Vinf / (n * D_m);
    if ~isfinite(J),            val = NaN; return; end
    CQ_val = max(0, F_CP_apc(n, J)) / (2*pi);
    Q_prop = CQ_val * rho * n^2 * D_m^5;
    val = Qm_fn(omega) - Q_prop;
end

function val = localBalanceDat(omega, Vinf, D_m, rho, Qm_fn, F_CQ)
    % Torque balance using CQ from dat file:  Qm = CQ * rho * n^2 * D^5
    n = omega / (2*pi);
    if ~isfinite(n) || n <= 0, val = NaN; return; end
    J = Vinf / (n * D_m);
    if ~isfinite(J),            val = NaN; return; end
    Q_prop = F_CQ(J) * rho * n^2 * D_m^5;
    val = Qm_fn(omega) - Q_prop;
end

function [J_out, CT_out, CP_out] = parsePropDatFile(filename)
    % Parse a propeller dat file with columns: J, CT, CP [, eta, ...]
    % Skips header/comment lines. Handles UIUC and similar formats.
    data = [];
    fid = fopen(filename,'r');
    if fid < 0
        error('Cannot open propeller file: %s', filename);
    end
    while ~feof(fid)
        raw = strtrim(fgetl(fid));
        if isempty(raw) || raw(1)=='%' || raw(1)=='#', continue; end
        nums = sscanf(raw,'%f');
        if numel(nums) >= 3 && nums(1) >= 0 && nums(1) <= 2.0
            data(end+1,:) = nums(1:3);  %#ok<AGROW>
        end
    end
    fclose(fid);

    if isempty(data)
        error('No valid J,CT,CP data found in: %s', filename);
    end

    [J_out, idx] = unique(data(:,1));
    CT_out = data(idx,2);
    CP_out = data(idx,3);
end

function propOut = packOutput(mode, propName, KV, Rm, I0, Vbat, I_max, ...
        D_in, pitch_in, V_vec, T_vec_N, I_vec_A, P_elec, P_prop, eta_vec, ...
        rpm_vec, J_vec, CT_vec, CP_vec, eta_prop_vec, Qp_vec_Nm)

    propOut = struct();
    propOut.mode         = mode;
    propOut.propName     = propName;
    propOut.KV           = KV;
    propOut.Rm           = Rm;
    propOut.I0           = I0;
    propOut.Vbat         = Vbat;
    propOut.I_max        = I_max;
    propOut.D_in         = D_in;
    propOut.pitch_in     = pitch_in;
    propOut.V_vec_mps    = V_vec;
    propOut.T_vec_N      = T_vec_N;
    propOut.I_vec_A      = I_vec_A;
    propOut.P_elec_vec_W = P_elec;
    propOut.P_prop_vec_W = P_prop;
    propOut.eta_vec      = eta_vec;       % electrical: TV/(Vbat*I)
    propOut.rpm_vec      = rpm_vec;
    propOut.J_vec        = J_vec;
    propOut.CT_vec       = CT_vec;
    propOut.CP_vec       = CP_vec;
    propOut.eta_prop_vec = eta_prop_vec;  % propulsive: J*CT/CP
    propOut.Qp_vec_Nm    = Qp_vec_Nm;
    propOut.T_static_N   = T_vec_N(1);
    propOut.I_static_A   = I_vec_A(1);
    [propOut.T_max_N, idx] = max(T_vec_N);
    propOut.V_at_Tmax    = V_vec(idx);
    propOut.I_peak_A     = max(I_vec_A(isfinite(I_vec_A)));
end

function printSummary(p, modeLabel)
    fprintf('\n============================================================\n');
    fprintf('PROPULSION ANALYSIS SUMMARY\n');
    fprintf('Mode: %s\n', modeLabel);
    fprintf('============================================================\n');
    fprintf('Propeller        = %s\n',     string(p.propName));
    fprintf('Battery voltage  = %.3f V\n', p.Vbat);
    fprintf('Motor KV         = %.1f RPM/V\n', p.KV);
    fprintf('Motor Rm         = %.4f ohm\n',   p.Rm);
    fprintf('Motor I0         = %.3f A\n',     p.I0);
    fprintf('Static thrust    = %.3f N\n',     p.T_static_N);
    fprintf('Static current   = %.3f A\n',     p.I_static_A);
    fprintf('Current limit    = %.3f A\n',     p.I_max);
end

function plotThrust(V_vec, T_vec_N, propName)
    figure('Name', sprintf('Thrust vs Speed — %s', propName), ...
           'NumberTitle','off','Color','w');
    plot(V_vec, T_vec_N, 'LineWidth', 2);
    grid on; box on;
    xlabel('Flight Speed  V_{\infty}  [m/s]');
    ylabel('Thrust  T  [N]');
    title(sprintf('Thrust vs Flight Speed — %s', propName), 'Interpreter','none');
end

function plotCurrent(V_vec, I_vec_A, I_max, propName)
    figure('Name', sprintf('Current Draw — %s', propName), ...
           'NumberTitle','off','Color','w');
    plot(V_vec, I_vec_A, 'LineWidth', 2); hold on;
    if isfinite(I_max)
        yline(I_max, '--', 'LineWidth', 1.5);
        legend('Current draw','Current limit','Location','best');
    end
    grid on; box on;
    xlabel('Flight Speed  V_{\infty}  [m/s]');
    ylabel('Current  I  [A]');
    title(sprintf('Current vs Flight Speed — %s', propName), 'Interpreter','none');
end
