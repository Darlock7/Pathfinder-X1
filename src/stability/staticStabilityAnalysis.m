function stabOut = staticStabilityAnalysis(stabIn)
% staticStabilityAnalysis
%
% Purpose:
%   Compute total aircraft mass properties and longitudinal static stability
%   metrics from CAD-based empty-airframe data plus discrete components.
%
% Inputs:
%   stabIn.emptyMass.mass_kg
%   stabIn.emptyMass.cg_m                 [1x3]
%   stabIn.emptyMass.Icg_kgm2             [3x3]
%
%   stabIn.components                     struct array with fields:
%       .name
%       .mass_kg
%       .r_m                             [1x3]
%
%   stabIn.cMAC_m
%   stabIn.xLEMAC_m
%
%   EITHER:
%       stabIn.xNP_m
%   OR:
%       stabIn.useApproxNP = true
%       stabIn.xACwingApprox_m
%
% Optional:
%   stabIn.SM_target_min
%   stabIn.SM_target_max
%
% Outputs:
%   stabOut.mass_kg
%   stabOut.cg_m
%   stabOut.Icg_kgm2
%   stabOut.xcg_over_MAC
%   stabOut.xnp_m
%   stabOut.xnp_over_MAC
%   stabOut.SM
%   stabOut.isStaticallyStable
%   stabOut.inTargetBand
%   stabOut.usedApproxNP

    arguments
        stabIn struct
    end

    % -----------------------------
    % Defaults
    % -----------------------------
    if ~isfield(stabIn, 'components') || isempty(stabIn.components)
        stabIn.components = struct('name', {}, 'mass_kg', {}, 'r_m', {});
    end
    if ~isfield(stabIn, 'SM_target_min')
        stabIn.SM_target_min = 0.05;
    end
    if ~isfield(stabIn, 'SM_target_max')
        stabIn.SM_target_max = 0.20;
    end
    if ~isfield(stabIn, 'useApproxNP')
        stabIn.useApproxNP = false;
    end

    % -----------------------------
    % Build total mass and CG
    % -----------------------------
    m0 = stabIn.emptyMass.mass_kg;
    r0 = stabIn.emptyMass.cg_m(:);

    totalMass = m0;
    totalMoment = m0 * r0;

    for i = 1:numel(stabIn.components)
        mi = stabIn.components(i).mass_kg;
        ri = stabIn.components(i).r_m(:);

        totalMass   = totalMass + mi;
        totalMoment = totalMoment + mi * ri;
    end

    if totalMass <= 0
        error('Total mass must be positive.');
    end

    cg = totalMoment / totalMass;

    % -----------------------------
    % Build total inertia about total CG
    % -----------------------------
    % CAD empty-airframe inertia is assumed about its own CG.
    I_total = shiftInertiaToNewCG(stabIn.emptyMass.Icg_kgm2, m0, r0, cg);

    % Treat added discrete components as point masses
    for i = 1:numel(stabIn.components)
        mi = stabIn.components(i).mass_kg;
        ri = stabIn.components(i).r_m(:);
        d  = ri - cg;

        I_point = mi * [ ...
            d(2)^2 + d(3)^2,   d(1)*d(2),          d(1)*d(3);
            d(1)*d(2),         d(1)^2 + d(3)^2,    d(2)*d(3);
            d(1)*d(3),         d(2)*d(3),          d(1)^2 + d(2)^2];

        I_total = I_total + I_point;
    end

    % -----------------------------
    % Neutral point
    % -----------------------------
    usedApproxNP = false;

    if isfield(stabIn, 'xNP_m') && ~isempty(stabIn.xNP_m)
        xNP = stabIn.xNP_m;
    elseif stabIn.useApproxNP && isfield(stabIn, 'xACwingApprox_m')
        xNP = stabIn.xACwingApprox_m;
        usedApproxNP = true;
    else
        error(['Neutral point not provided. Supply stabIn.xNP_m or enable ', ...
               'stabIn.useApproxNP with stabIn.xACwingApprox_m.']);
    end

    % -----------------------------
    % Static margin
    % -----------------------------
    cMAC   = stabIn.cMAC_m;
    xLEMAC = stabIn.xLEMAC_m;

    if cMAC <= 0
        error('cMAC_m must be positive.');
    end

    xcg_over_MAC = (cg(1) - xLEMAC) / cMAC;
    xnp_over_MAC = (xNP   - xLEMAC) / cMAC;
    SM           = (xNP   - cg(1))  / cMAC;

    isStaticallyStable = SM > 0;
    inTargetBand       = (SM >= stabIn.SM_target_min) && (SM <= stabIn.SM_target_max);

    % -----------------------------
    % Output
    % -----------------------------
    stabOut = struct();
    stabOut.mass_kg            = totalMass;
    stabOut.cg_m               = cg(:).';
    stabOut.Icg_kgm2           = I_total;
    stabOut.xcg_over_MAC       = xcg_over_MAC;
    stabOut.xnp_m              = xNP;
    stabOut.xnp_over_MAC       = xnp_over_MAC;
    stabOut.SM                 = SM;
    stabOut.isStaticallyStable = isStaticallyStable;
    stabOut.inTargetBand       = inTargetBand;
    stabOut.usedApproxNP       = usedApproxNP;
end

function I_new = shiftInertiaToNewCG(I_oldcg, m, r_oldcg, r_newcg)
% shiftInertiaToNewCG
% Shift inertia tensor from one CG location to another point using the
% sign convention from class notes:
% J_ij = I_ij + m (r·r delta_ij - r_i r_j)

    d = r_oldcg(:) - r_newcg(:);

    shiftTerm = m * [ ...
        d(2)^2 + d(3)^2,   d(1)*d(2),          d(1)*d(3);
        d(1)*d(2),         d(1)^2 + d(3)^2,    d(2)*d(3);
        d(1)*d(3),         d(2)*d(3),          d(1)^2 + d(2)^2];

    I_new = I_oldcg + shiftTerm;
end