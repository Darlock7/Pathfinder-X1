function spanOut = spanwiseAeroEstimate(spanIn)
% spanwiseAeroEstimate
%
% First-pass wing-only spanwise aerodynamic estimate.
% Combines:
%   - trapezoidal wing chord variation
%   - root/tip section airfoil properties
%   - twist distribution
%
% It estimates:
%   - local chord
%   - local effective alpha
%   - local section cl
%   - local lift per unit span
%
% INPUTS
%   spanIn.rho
%   spanIn.V_ref_mps
%   spanIn.alpha_ref_deg
%   spanIn.b_half_m
%   spanIn.c_root_m
%   spanIn.c_tip_m
%   spanIn.taper
%   spanIn.rootCla_per_deg
%   spanIn.tipCla_per_deg
%   spanIn.rootAlphaL0_deg
%   spanIn.tipAlphaL0_deg
%   spanIn.rootClmax
%   spanIn.tipClmax
%   spanIn.rootCm0
%   spanIn.tipCm0
%   spanIn.eta_twist
%   spanIn.twist_deg
%   spanIn.Nspan
%
% OUTPUTS
%   spanOut.y_m
%   spanOut.eta
%   spanOut.c_m
%   spanOut.alpha_eff_deg
%   spanOut.cla_per_deg
%   spanOut.alphaL0_deg
%   spanOut.clmax
%   spanOut.cl_local
%   spanOut.cm0_local
%   spanOut.q_Pa
%   spanOut.Lprime_N_per_m
%   spanOut.L_half_N
%   spanOut.L_total_N

    arguments
        spanIn struct
    end

    %% ---------------- Required field checks ----------------
    req = { ...
        'rho','V_ref_mps','alpha_ref_deg', ...
        'b_half_m','c_root_m','c_tip_m','taper', ...
        'rootCla_per_deg','tipCla_per_deg', ...
        'rootAlphaL0_deg','tipAlphaL0_deg', ...
        'rootClmax','tipClmax', ...
        'rootCm0','tipCm0', ...
        'eta_twist','twist_deg','Nspan'};

    for k = 1:numel(req)
        if ~isfield(spanIn, req{k})
            error('spanwiseAeroEstimate:MissingField', ...
                'Missing required input field: %s', req{k});
        end
    end

    %% ---------------- Basic setup ----------------
    rho           = spanIn.rho;
    V_ref_mps     = spanIn.V_ref_mps;
    alpha_ref_deg = spanIn.alpha_ref_deg;

    b_half_m = spanIn.b_half_m;
    c_root_m = spanIn.c_root_m;
    c_tip_m  = spanIn.c_tip_m;

    Nspan = max(2, round(spanIn.Nspan));

    y_m = linspace(0, b_half_m, Nspan).';
    eta = y_m / b_half_m;

    %% ---------------- Local chord ----------------
    % Linear taper from root to tip
    c_m = c_root_m + (c_tip_m - c_root_m) .* eta;

    %% ---------------- Interpolate section properties ----------------
    cla_per_deg = spanIn.rootCla_per_deg + ...
        (spanIn.tipCla_per_deg - spanIn.rootCla_per_deg) .* eta;

    alphaL0_deg = spanIn.rootAlphaL0_deg + ...
        (spanIn.tipAlphaL0_deg - spanIn.rootAlphaL0_deg) .* eta;

    clmax = spanIn.rootClmax + ...
        (spanIn.tipClmax - spanIn.rootClmax) .* eta;

    cm0_local = spanIn.rootCm0 + ...
        (spanIn.tipCm0 - spanIn.rootCm0) .* eta;

    %% ---------------- Interpolate twist onto same span stations ----------------
    eta_twist = spanIn.eta_twist(:);
    twist_deg_in = spanIn.twist_deg(:);

    if numel(eta_twist) ~= numel(twist_deg_in)
        error('spanwiseAeroEstimate:TwistSizeMismatch', ...
            'eta_twist and twist_deg must have the same length.');
    end

    twist_deg = interp1(eta_twist, twist_deg_in, eta, 'linear', 'extrap');

    %% ---------------- Effective alpha ----------------
    alpha_eff_deg = alpha_ref_deg + twist_deg;

    %% ---------------- Local section cl estimate ----------------
    % Thin/linearized local estimate based on alpha_eff - alphaL0
    cl_local_linear = cla_per_deg .* (alpha_eff_deg - alphaL0_deg);

    % Clip to local clmax on positive side for first-pass realism
    cl_local = min(cl_local_linear, clmax);

    %% ---------------- Lift per unit span ----------------
    q_Pa = 0.5 * rho * V_ref_mps^2;

    % 2D section lift per unit span: L' = q * c * cl
    Lprime_N_per_m = q_Pa .* c_m .* cl_local;

    %% ---------------- Integrate lift ----------------
    L_half_N  = trapz(y_m, Lprime_N_per_m);
    L_total_N = 2 * L_half_N;

    %% ---------------- Output ----------------
    spanOut = struct();

    spanOut.y_m = y_m;
    spanOut.eta = eta;
    spanOut.c_m = c_m;

    spanOut.alpha_ref_deg = alpha_ref_deg;
    spanOut.twist_deg     = twist_deg;
    spanOut.alpha_eff_deg = alpha_eff_deg;

    spanOut.cla_per_deg = cla_per_deg;
    spanOut.alphaL0_deg = alphaL0_deg;
    spanOut.clmax       = clmax;
    spanOut.cl_local    = cl_local;
    spanOut.cm0_local   = cm0_local;

    spanOut.q_Pa = q_Pa;
    spanOut.Lprime_N_per_m = Lprime_N_per_m;
    spanOut.L_half_N  = L_half_N;
    spanOut.L_total_N = L_total_N;
end