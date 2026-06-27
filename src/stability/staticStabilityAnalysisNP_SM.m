function stabOut = staticStabilityAnalysisNP_SM(stabIn)
% staticStabilityAnalysis
%
% Purpose:
%   Compute loaded and unloaded longitudinal static-stability metrics:
%       - CG as %MAC
%       - Neutral point as %MAC
%       - Static margin
%       - Stability flags
%
% Notes:
%   - SI units only
%   - x positive aft
%   - Neutral point should ultimately come from AVL / aero model
%   - Current fallback uses a simple approximate wing aerodynamic center
%
% Required inputs:
%   stabIn.cMAC_m
%   stabIn.xLEMAC_m
%   stabIn.cg_loaded_m        [1x3]
%   stabIn.cg_unloaded_m      [1x3]
%
% One of:
%   stabIn.xNP_m              scalar [m]
%   OR
%   stabIn.useApproxNP        logical
%   stabIn.xACwingApprox_m    scalar [m]
%
% Optional:
%   stabIn.SM_target_min      scalar [-]
%   stabIn.SM_target_max      scalar [-]
%
% Outputs:
%   stabOut.loaded
%   stabOut.unloaded
%   stabOut.xNP_m
%   stabOut.usedApproxNP

    arguments
        stabIn struct
    end

    %% ---------------- Required inputs ----------------
    req = {'cMAC_m','xLEMAC_m','cg_loaded_m','cg_unloaded_m'};
    for k = 1:numel(req)
        if ~isfield(stabIn, req{k})
            error('staticStabilityAnalysis:MissingField', ...
                'Missing required input field: %s', req{k});
        end
    end

    cMAC   = stabIn.cMAC_m;
    xLEMAC = stabIn.xLEMAC_m;

    cg_loaded   = reshape(stabIn.cg_loaded_m,   1, 3);
    cg_unloaded = reshape(stabIn.cg_unloaded_m, 1, 3);

    %% ---------------- Target band defaults ----------------
    if ~isfield(stabIn,'SM_target_min')
        stabIn.SM_target_min = 0.10;   % [-]
    end
    if ~isfield(stabIn,'SM_target_max')
        stabIn.SM_target_max = 0.20;   % [-]
    end

    %% ---------------- Neutral point ----------------
    usedApproxNP = false;

    if isfield(stabIn,'xNP_m') && ~isempty(stabIn.xNP_m)
        xNP = stabIn.xNP_m;
    else
        if ~isfield(stabIn,'useApproxNP') || ~stabIn.useApproxNP
            error('staticStabilityAnalysis:MissingNP', ...
                ['No neutral point provided. Supply stabIn.xNP_m, or set ', ...
                 'stabIn.useApproxNP = true and provide stabIn.xACwingApprox_m.']);
        end
        if ~isfield(stabIn,'xACwingApprox_m')
            error('staticStabilityAnalysis:MissingApproxNP', ...
                'Approximate NP requested, but xACwingApprox_m was not provided.');
        end
        xNP = stabIn.xACwingApprox_m;
        usedApproxNP = true;
    end

    %% ---------------- Loaded case ----------------
    loaded = evaluateOneCase(cg_loaded, xLEMAC, cMAC, xNP, ...
        stabIn.SM_target_min, stabIn.SM_target_max);

    %% ---------------- Unloaded case ----------------
    unloaded = evaluateOneCase(cg_unloaded, xLEMAC, cMAC, xNP, ...
        stabIn.SM_target_min, stabIn.SM_target_max);

    %% ---------------- Output ----------------
    stabOut = struct();
    stabOut.xNP_m        = xNP;
    stabOut.usedApproxNP = usedApproxNP;
    stabOut.loaded       = loaded;
    stabOut.unloaded     = unloaded;
    stabOut.SM_target_min = stabIn.SM_target_min;
    stabOut.SM_target_max = stabIn.SM_target_max;
end

%% ========================================================================
function caseOut = evaluateOneCase(cg_m, xLEMAC, cMAC, xNP, SMmin, SMmax)

    xcg = cg_m(1);

    xcg_over_MAC = (xcg - xLEMAC) / cMAC;
    xnp_over_MAC = (xNP - xLEMAC) / cMAC;
    SM           = (xNP - xcg)   / cMAC;

    caseOut = struct();
    caseOut.cg_m          = cg_m;
    caseOut.xcg_m         = xcg;
    caseOut.xNP_m         = xNP;
    caseOut.xcg_over_MAC  = xcg_over_MAC;
    caseOut.xnp_over_MAC  = xnp_over_MAC;
    caseOut.SM            = SM;

    caseOut.isStaticallyStable = (SM > 0);
    caseOut.inTargetBand       = (SM >= SMmin) && (SM <= SMmax);
end