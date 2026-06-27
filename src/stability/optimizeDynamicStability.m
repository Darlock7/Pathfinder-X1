function optOut = optimizeDynamicStability(optIn)
% optimizeDynamicStability
%
% Purpose:
%   CMA-ES (Covariance Matrix Adaptation Evolution Strategy) optimizer for
%   dynamic stability. Maximizes static margin subject to MIL-STD Level 1/2
%   handling quality constraints via penalty terms in stabilityObjective.m.
%   Uses parfor over each generation's population — run on HPC for speed.
%
% Inputs (optIn struct):
%   .ctx         context struct passed through to stabilityObjective
%                (same fields as sweepIn: wingIn, twistIn, vertIn, dynIn,
%                 cadMass, compFixed, eta_servo, m_wing_struct_kg, m_vert_struct_kg)
%   .x0          [7x1] initial mean  (baseline parameter vector)
%   .sigma0      initial step size (scalar, ~0.2 * range)
%   .lb          [7x1] lower bounds
%   .ub          [7x1] upper bounds
%   .maxGen      maximum generations  (default 200)
%   .tolSigma    step-size convergence tolerance (default 1e-5)
%   .tolFun      objective range convergence tolerance (default 1e-4)
%   .verbose     print progress every N generations (default 10, 0 = silent)
%
% Outputs (optOut struct):
%   .xBest       [7x1] best parameter vector found
%   .JBest       best objective value
%   .infoBest    info struct from stabilityObjective at best point
%   .history     struct array: .gen, .JBest, .sigma, .SM_pct per generation

    % ---- defaults ----
    if ~isfield(optIn,'maxGen'),   optIn.maxGen   = 200;   end
    if ~isfield(optIn,'tolSigma'), optIn.tolSigma = 1e-5;  end
    if ~isfield(optIn,'tolFun'),   optIn.tolFun   = 1e-4;  end
    if ~isfield(optIn,'verbose'),  optIn.verbose  = 10;    end
    if ~isfield(optIn,'lambda'),   optIn.lambda   = 0;     end  % 0 = use Hansen default

    ctx    = optIn.ctx;
    x0     = optIn.x0(:);
    sigma0 = optIn.sigma0;
    lb     = optIn.lb(:);
    ub     = optIn.ub(:);
    maxGen = optIn.maxGen;

    n = numel(x0);   % number of variables = 7

    % ---- CMA-ES hyperparameters (Hansen 2016 defaults) ----
    if optIn.lambda > 0
        lam = optIn.lambda;
    else
        lam = 4 + floor(3*log(n));             % population size (default ~10 for n=7)
    end
    mu     = floor(lam/2);                     % parents selected
    w_raw  = log(mu + 0.5) - log(1:mu)';       % raw recombination weights
    w      = w_raw / sum(w_raw);               % normalized weights
    mueff  = 1 / sum(w.^2);                    % effective selection mass

    % step-size control
    cs     = (mueff + 2) / (n + mueff + 5);
    ds     = 1 + 2*max(0, sqrt((mueff-1)/(n+1)) - 1) + cs;
    chiN   = sqrt(n) * (1 - 1/(4*n) + 1/(21*n^2));  % expected ||N(0,I)||

    % covariance matrix adaptation
    cc     = (4 + mueff/n) / (n + 4 + 2*mueff/n);
    c1     = 2 / ((n+1.3)^2 + mueff);
    cmu    = min(1-c1, 2*(mueff - 2 + 1/mueff) / ((n+2)^2 + mueff));

    % ---- state initialization ----
    m      = x0;
    sigma  = sigma0;
    C      = eye(n);
    ps     = zeros(n,1);   % step-size evolution path
    pc     = zeros(n,1);   % covariance evolution path
    eigenC = eye(n);       % eigenvectors of C
    diagD  = ones(n,1);    % sqrt of eigenvalues of C
    eigAge = 0;            % generation counter since last eigen decomposition

    JBest       = Inf;
    xBest       = m;
    infoBest    = struct('SM_pct',NaN,'Xcg_m',NaN,'sp_zeta',NaN,'ph_zeta',NaN,'dr_zeta',NaN,'failed',true);
    history     = struct('gen',{},'JBest',{},'sigma',{},'SM_pct',{});

    fprintf('\n===== CMA-ES OPTIMIZATION  (n=%d, lambda=%d, mu=%d) =====\n', n, lam, mu);
    fprintf('%-6s %-12s %-10s %-10s %-8s\n', 'Gen','JBest','SM%','sigma','Xcg_m');

    for gen = 1:maxGen

        % ---- sample population ----
        % Recompute eigen decomposition of C every ~n/10 generations
        if eigAge >= n/10
            [eigenC, D] = eig(C);
            diagD = sqrt(max(diag(D), 0));   % guard against tiny negatives
            eigAge = 0;
        end
        eigAge = eigAge + 1;

        % sample lam candidates: xk = m + sigma * B * D * z,  z ~ N(0,I)
        Z = randn(n, lam);
        Y = eigenC * (diagD .* Z);           % [n x lam]  in C-space
        X = m + sigma * Y;                   % [n x lam]  candidates

        % apply bounds (reflect if outside; clips to boundary as fallback)
        X = max(lb, min(ub, X));

        % ---- evaluate population (parfor for HPC) ----
        Jvals  = nan(lam, 1);
        infos  = cell(lam, 1);
        Xeval  = X;    % local copy for parfor slice

        parfor k = 1:lam
            [Jvals(k), infos{k}] = stabilityObjective(Xeval(:,k), ctx);
        end

        % ---- sort by objective ----
        [Jsorted, idx] = sort(Jvals, 'ascend');
        Xsorted = Xeval(:, idx);
        Ysorted = Y(:, idx);

        % track best
        if Jsorted(1) < JBest
            JBest    = Jsorted(1);
            xBest    = Xsorted(:,1);
            infoBest = infos{idx(1)};
        end

        % ---- update mean ----
        m_old = m;
        m     = Xsorted(:, 1:mu) * w;        % weighted recombination

        % ---- update evolution paths ----
        y_m = (m - m_old) / sigma;           % normalized mean shift

        % C^{-1/2} * y_m  via eigen decomposition
        invsqrtC_ym = eigenC * ((1./diagD) .* (eigenC' * y_m));

        ps = (1-cs)*ps + sqrt(cs*(2-cs)*mueff) * invsqrtC_ym;

        hsig = norm(ps) / sqrt(1-(1-cs)^(2*(gen+1))) / chiN < 1.4 + 2/(n+1);

        pc = (1-cc)*pc + hsig * sqrt(cc*(2-cc)*mueff) * y_m;

        % ---- update covariance matrix ----
        rank1  = c1 * (pc*pc' + (1-hsig)*cc*(2-cc)*C);
        rankmu = cmu * (Ysorted(:,1:mu) * diag(w) * Ysorted(:,1:mu)');
        C      = (1 - c1 - cmu) * C + rank1 + rankmu;

        % enforce symmetry and numerical stability
        C = (C + C') / 2;

        % ---- update step size ----
        sigma = sigma * exp((cs/ds) * (norm(ps)/chiN - 1));

        % ---- log ----
        SM_now = infoBest.SM_pct;
        if optIn.verbose > 0 && mod(gen, optIn.verbose) == 0
            fprintf('%-6d %-12.4f %-10.2f %-10.2e %-8.4f\n', ...
                gen, JBest, SM_now, sigma, infoBest.Xcg_m);
        end
        history(end+1) = struct('gen', gen, 'JBest', JBest, 'sigma', sigma, 'SM_pct', SM_now);

        % ---- convergence checks ----
        if sigma < optIn.tolSigma
            fprintf('Converged: sigma = %.2e < tolSigma\n', sigma);
            break;
        end
        if gen > 20 && range([history(end-9:end).JBest]) < optIn.tolFun
            fprintf('Converged: objective range < tolFun over last 10 generations\n');
            break;
        end
    end

    fprintf('\n--- Best result ---\n');
    fprintf('SM      = %.2f %%MAC\n',  infoBest.SM_pct);
    fprintf('Xcg     = %.4f m\n',      infoBest.Xcg_m);
    fprintf('SP zeta = %.4f\n',        infoBest.sp_zeta);
    fprintf('PH zeta = %.4f\n',        infoBest.ph_zeta);
    fprintf('x* = [sweep=%.1f  taper=%.3f  twistTip=%.2f  AR_v=%.2f  taperV=%.3f  sweepV=%.1f  xLE_root=%.4f]\n', ...
        xBest(1), xBest(2), xBest(3), xBest(4), xBest(5), xBest(6), xBest(7));
    fprintf('===================================================\n\n');

    optOut.xBest    = xBest;
    optOut.JBest    = JBest;
    optOut.infoBest = infoBest;
    optOut.history  = history;
end
