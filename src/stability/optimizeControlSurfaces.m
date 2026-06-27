function csOptOut = optimizeControlSurfaces(csOptIn)
% optimizeControlSurfaces
%
% Purpose:
%   CMA-ES optimizer for elevon + rudder geometry. Minimizes turn radius
%   (= maximizes maneuvering authority) subject to roll rate, trim, and
%   yaw authority constraints.
%
% Design variables (4):
%   x(1) = cs_chord_frac   [-]   elevon chord fraction
%   x(2) = eta_cs_start    [-]   elevon inboard span station
%   x(3) = delta_eta       [-]   elevon span width (eta_end = start + delta_eta)
%   x(4) = rudder_cf       [-]   rudder chord fraction on vertical fin
%
% Inputs (csOptIn struct):
%   .dynIn           baseline dynIn struct (control surface fields updated per sample)
%   .CL_trim         [-]    cruise lift coefficient
%   .CLmax           [-]    airfoil stall CL
%   .V_mps           [m/s]  cruise speed
%   .rho_kgm3        [kg/m³]
%   .S_ref_m2        [m²]
%   .b_m             [m]
%   .mass_kg         [kg]
%   .x0              [4x1]  initial guess (default: [0.25; 0.60; 0.30; 0.30])
%   .sigma0          initial step size (default 0.05)
%   .lb              [4x1]  lower bounds (default [0.15; 0.30; 0.15; 0.20])
%   .ub              [4x1]  upper bounds (default [0.45; 0.70; 0.45; 0.50])
%   .delta_e_max     [deg]  max elevon deflection    (default 20)
%   .delta_r_max     [deg]  max rudder deflection    (default 25)
%   .p_ss_min_dps    [deg/s] minimum roll rate — 48° bank in 1 s × 1.5 margin (default 70)
%   .R_min_max_m     [m]    hard max turn radius at cruise — mission lap geometry (default 36.5)
%   .de_trim_max_deg [deg]  max allowable trim deflection (default 15)
%   .eta_end_max     [-]    max allowable eta_cs_end  (default 0.95)
%   .maxGen          maximum CMA-ES generations      (default 100)
%   .lambda          population size (0 = Hansen default) (default 0)
%   .tolSigma        step-size convergence tolerance  (default 1e-4)
%   .tolFun          objective range tolerance        (default 1e-3)
%   .verbose         print interval in generations    (default 5)
%
% Outputs (csOptOut struct):
%   .xBest      [4x1]  best parameter vector
%   .JBest      best objective value
%   .best       struct: chord_frac, eta_start, eta_end, rudder_cf,
%                       R_min, de_trim, p_ss, Cn_ratio
%   .history    struct array per generation

    % ---- defaults ----
    if ~isfield(csOptIn,'x0'),              csOptIn.x0              = [0.25; 0.60; 0.30; 0.30]; end
    if ~isfield(csOptIn,'sigma0'),          csOptIn.sigma0          = 0.05;  end
    if ~isfield(csOptIn,'lb'),              csOptIn.lb              = [0.15; 0.30; 0.15; 0.20]; end
    if ~isfield(csOptIn,'ub'),              csOptIn.ub              = [0.45; 0.70; 0.45; 0.50]; end
    if ~isfield(csOptIn,'delta_e_max'),     csOptIn.delta_e_max     = 20;    end
    if ~isfield(csOptIn,'delta_r_max'),     csOptIn.delta_r_max     = 25;    end
    if ~isfield(csOptIn,'p_ss_min_dps'),    csOptIn.p_ss_min_dps    = 70;    end  % 48° bank in 1 s × 1.5 margin
    if ~isfield(csOptIn,'R_min_max_m'),    csOptIn.R_min_max_m     = 36.5;  end  % mission hard cap at cruise speed
    if ~isfield(csOptIn,'de_trim_max_deg'), csOptIn.de_trim_max_deg = 15;    end
    if ~isfield(csOptIn,'eta_end_max'),     csOptIn.eta_end_max     = 0.95;  end
    if ~isfield(csOptIn,'maxGen'),          csOptIn.maxGen          = 100;   end
    if ~isfield(csOptIn,'lambda'),          csOptIn.lambda          = 0;     end
    if ~isfield(csOptIn,'tolSigma'),        csOptIn.tolSigma        = 1e-4;  end
    if ~isfield(csOptIn,'tolFun'),          csOptIn.tolFun          = 1e-3;  end
    if ~isfield(csOptIn,'verbose'),         csOptIn.verbose         = 5;     end

    ctx = csOptIn;
    x0  = csOptIn.x0(:);
    lb  = csOptIn.lb(:);
    ub  = csOptIn.ub(:);
    n   = 4;

    % ---- CMA-ES hyperparameters (Hansen 2016 defaults) ----
    if csOptIn.lambda > 0
        lam = csOptIn.lambda;
    else
        lam = 4 + floor(3*log(n));
    end
    mu     = floor(lam/2);
    w_raw  = log(mu + 0.5) - log(1:mu)';
    w      = w_raw / sum(w_raw);
    mueff  = 1 / sum(w.^2);

    cs     = (mueff + 2) / (n + mueff + 5);
    ds     = 1 + 2*max(0, sqrt((mueff-1)/(n+1)) - 1) + cs;
    chiN   = sqrt(n) * (1 - 1/(4*n) + 1/(21*n^2));

    cc     = (4 + mueff/n) / (n + 4 + 2*mueff/n);
    c1     = 2 / ((n+1.3)^2 + mueff);
    cmu    = min(1-c1, 2*(mueff - 2 + 1/mueff) / ((n+2)^2 + mueff));

    % ---- state initialization ----
    m      = x0;
    sigma  = csOptIn.sigma0;
    C      = eye(n);
    ps     = zeros(n,1);
    pc     = zeros(n,1);
    eigenC = eye(n);
    diagD  = ones(n,1);
    eigAge = 0;

    JBest    = Inf;
    xBest    = m;
    infoBest = struct('R_min_m', NaN, 'de_trim_deg', NaN, 'p_ss_dps', NaN, ...
                      'n_max', NaN, 'phi_max_deg', NaN, 'Cn_ratio', NaN, 'failed', true);
    history  = struct('gen',{},'JBest',{},'sigma',{},'R_min_m',{},'de_trim_deg',{});

    fprintf('\n===== CONTROL SURFACE CMA-ES (n=%d, lambda=%d, mu=%d) =====\n', n, lam, mu);
    fprintf('%-6s %-10s %-10s %-10s %-10s %-8s\n','Gen','JBest','R_min[m]','de_trim','Cn_ratio','sigma');

    for gen = 1:csOptIn.maxGen

        if eigAge >= n/10
            [eigenC, D] = eig(C);
            diagD  = sqrt(max(diag(D), 0));
            eigAge = 0;
        end
        eigAge = eigAge + 1;

        Z = randn(n, lam);
        Y = eigenC * (diagD .* Z);
        X = m + sigma * Y;
        X = max(lb, min(ub, X));

        % evaluate population in parallel
        Jvals = nan(lam, 1);
        infos = cell(lam, 1);
        Xeval = X;

        parfor k = 1:lam
            [Jvals(k), infos{k}] = cs_objective(Xeval(:,k), ctx);
        end

        % sort
        [Jsorted, idx] = sort(Jvals, 'ascend');
        Xsorted = Xeval(:, idx);
        Ysorted = Y(:, idx);

        if Jsorted(1) < JBest
            JBest    = Jsorted(1);
            xBest    = Xsorted(:,1);
            infoBest = infos{idx(1)};
        end

        % update mean
        m_old = m;
        m     = Xsorted(:, 1:mu) * w;

        y_m          = (m - m_old) / sigma;
        invsqrtC_ym  = eigenC * ((1./diagD) .* (eigenC' * y_m));
        ps           = (1-cs)*ps + sqrt(cs*(2-cs)*mueff) * invsqrtC_ym;
        hsig         = norm(ps)/sqrt(1-(1-cs)^(2*(gen+1)))/chiN < 1.4 + 2/(n+1);
        pc           = (1-cc)*pc + hsig * sqrt(cc*(2-cc)*mueff) * y_m;

        rank1  = c1  * (pc*pc' + (1-hsig)*cc*(2-cc)*C);
        rankmu = cmu * (Ysorted(:,1:mu) * diag(w) * Ysorted(:,1:mu)');
        C      = (1 - c1 - cmu) * C + rank1 + rankmu;
        C      = (C + C') / 2;

        sigma  = sigma * exp((cs/ds) * (norm(ps)/chiN - 1));

        if csOptIn.verbose > 0 && mod(gen, csOptIn.verbose) == 0
            fprintf('%-6d %-10.3f %-10.2f %-10.2f %-10.3f %-8.2e\n', ...
                gen, JBest, infoBest.R_min_m, infoBest.de_trim_deg, infoBest.Cn_ratio, sigma);
        end
        history(end+1) = struct('gen', gen, 'JBest', JBest, 'sigma', sigma, ...
            'R_min_m', infoBest.R_min_m, 'de_trim_deg', infoBest.de_trim_deg);

        if sigma < csOptIn.tolSigma
            fprintf('Converged: sigma = %.2e\n', sigma); break;
        end
        if gen > 20 && range([history(end-9:end).JBest]) < csOptIn.tolFun
            fprintf('Converged: objective range < tolFun\n'); break;
        end
    end

    cf_best    = xBest(1);
    eta_s_best = xBest(2);
    eta_e_best = xBest(2) + xBest(3);
    rud_cf_best = xBest(4);

    fprintf('\n--- Best control surface configuration ---\n');
    fprintf('  cs_chord_frac  = %.3f\n',  cf_best);
    fprintf('  eta_cs_start   = %.3f\n',  eta_s_best);
    fprintf('  eta_cs_end     = %.3f\n',  eta_e_best);
    fprintf('  rudder_cf      = %.3f\n',  rud_cf_best);
    fprintf('  Turn radius    = %.2f m\n', infoBest.R_min_m);
    fprintf('  Trim deflection= %.2f deg\n', infoBest.de_trim_deg);
    fprintf('  Roll rate      = %.1f deg/s\n', infoBest.p_ss_dps);
    fprintf('  Max load factor= %.2f g\n', infoBest.n_max);
    fprintf('  Max bank angle = %.1f deg\n', infoBest.phi_max_deg);
    fprintf('  Cn_rud/Cn_adv  = %.3f  (>1 = rudder overcomes adverse yaw)\n', infoBest.Cn_ratio);
    fprintf('==============================================\n\n');

    % convergence plot
    gens   = [history.gen];
    R_hist = [history.R_min_m];
    figure('Name','Control Surface Optimization Convergence','NumberTitle','off');
    subplot(2,1,1);
    plot(gens, [history.JBest], 'b-', 'LineWidth', 2);
    ylabel('Objective J'); xlabel('Generation'); title('CMA-ES Convergence'); grid on;
    subplot(2,1,2);
    plot(gens, R_hist, 'r-', 'LineWidth', 2);
    ylabel('Min turn radius [m]'); xlabel('Generation'); title('Minimum Turn Radius'); grid on;

    csOptOut.xBest   = xBest;
    csOptOut.JBest   = JBest;
    csOptOut.best    = struct('chord_frac', cf_best, 'eta_start', eta_s_best, ...
                              'eta_end', eta_e_best, 'rudder_cf', rud_cf_best, ...
                              'R_min_m', infoBest.R_min_m, ...
                              'de_trim_deg', infoBest.de_trim_deg, ...
                              'p_ss_dps', infoBest.p_ss_dps, ...
                              'n_max', infoBest.n_max, ...
                              'phi_max_deg', infoBest.phi_max_deg, ...
                              'Cn_ratio', infoBest.Cn_ratio);
    csOptOut.history = history;
end

%% ========================================================================
function [J, info] = cs_objective(x, ctx)
    J    = 1e6;
    info = struct('R_min_m', NaN, 'de_trim_deg', NaN, 'p_ss_dps', NaN, ...
                  'n_max', NaN, 'phi_max_deg', NaN, 'Cn_ratio', NaN, 'failed', true);
    try
        cf     = x(1);
        eta_s  = x(2);
        eta_e  = x(2) + x(3);
        rud_cf = x(4);

        % hard geometry constraint — fin attachment clearance
        if eta_e > ctx.eta_end_max
            J = 1e6;
            return;
        end

        dynIn_k               = ctx.dynIn;
        dynIn_k.cs_chord_frac = cf;
        dynIn_k.eta_cs_start  = eta_s;
        dynIn_k.eta_cs_end    = eta_e;
        dynIn_k.rudder_cf     = rud_cf;
        dynIn_k.verbose       = false;
        dynIn_k.plotModes     = false;
        dynIn_k.viewGeometry  = false;

        dynOut_k = dynamicStabilityAVL(dynIn_k);

        csIn_k.CLde        = dynOut_k.controlDerivs.CLde;
        csIn_k.Cmde        = dynOut_k.controlDerivs.Cmde;
        csIn_k.Clda        = dynOut_k.controlDerivs.Clda;
        csIn_k.Cnda        = dynOut_k.controlDerivs.Cnda;
        csIn_k.Cndr        = dynOut_k.controlDerivs.Cndr;
        csIn_k.Cm0_trim    = dynOut_k.controlDerivs.Cm0_trim;
        csIn_k.CL_trim     = ctx.CL_trim;
        csIn_k.CLmax       = ctx.CLmax;
        csIn_k.V_mps       = ctx.V_mps;
        csIn_k.rho_kgm3    = ctx.rho_kgm3;
        csIn_k.S_ref_m2    = ctx.S_ref_m2;
        csIn_k.b_m         = ctx.b_m;
        csIn_k.mass_kg     = ctx.mass_kg;
        csIn_k.Clp         = dynOut_k.derivatives.Clp;
        csIn_k.delta_e_max = ctx.delta_e_max;
        csIn_k.delta_r_max = ctx.delta_r_max;
        csIn_k.showPlots   = false;

        csOut_k = controlSurfaceSizing(csIn_k);

        R_min   = csOut_k.R_min_m;
        de_trim = csOut_k.delta_e_trim_deg;
        p_ss    = csOut_k.p_ss_dps;

        % yaw authority: rudder must overcome adverse yaw from ailerons
        % Cn_ratio = Cndr*dr_max / (Cnda*da_max) — want >= 1
        Cn_rud = abs(dynOut_k.controlDerivs.Cndr) * ctx.delta_r_max;
        Cn_adv = abs(dynOut_k.controlDerivs.Cnda) * csIn_k.delta_e_max;
        Cn_ratio = Cn_rud / max(Cn_adv, 1e-9);

        % hard constraint: must meet mission turn requirement at cruise speed
        if R_min > ctx.R_min_max_m; return; end

        % primary objective: minimize turn radius
        J = R_min;

        % penalty: trim too large (every degree over the limit costs 50 m)
        if abs(de_trim) > ctx.de_trim_max_deg
            J = J + 50 * (abs(de_trim) - ctx.de_trim_max_deg);
        end

        % penalty: roll rate too low (every deg/s under limit costs 0.5 m)
        if p_ss < ctx.p_ss_min_dps
            J = J + 0.5 * (ctx.p_ss_min_dps - p_ss);
        end

        % soft penalty: rudder can't overcome adverse yaw
        % scales from 0 (Cn_ratio=1) to 30 m (Cn_ratio=0)
        if Cn_ratio < 1.0
            J = J + 30 * (1.0 - Cn_ratio);
        end

        info.R_min_m     = R_min;
        info.de_trim_deg = de_trim;
        info.p_ss_dps    = p_ss;
        info.n_max       = csOut_k.n_max;
        info.phi_max_deg = csOut_k.phi_max_deg;
        info.Cn_ratio    = Cn_ratio;
        info.failed      = false;

    catch
        % pipeline failure — return large J
    end
end
