function foilOut = runSingleAirfoilXFOIL_pointwiseFallback(foilName, Re, alpha_deg, opts)
% runSingleAirfoilXFOIL_pointwiseFallback
%
% Purpose:
%   Robust fallback when a full alpha sweep hangs or fails.
%   Runs one alpha at a time and assembles a partial polar.
%
% Inputs:
%   foilName    : char/string
%   Re          : scalar Reynolds number [-]
%   alpha_deg   : vector [deg]
%   opts        : same struct used by runSingleAirfoilXFOIL
%
% Output:
%   foilOut     : struct with partial polar and derived metrics
%
% Notes:
%   Requires the existing airfoilAnalysisXFOIL(...) to return valid
%   single-alpha outputs.

    arguments
        foilName
        Re (1,1) double {mustBePositive}
        alpha_deg (:,1) double
        opts struct
    end

    alpha_deg = alpha_deg(:);

    alpha_valid = [];
    CL_valid = [];
    CD_valid = [];
    CM_valid = [];

    for i = 1:numel(alpha_deg)
        a = alpha_deg(i);

        try
            tmp = runSingleAirfoilXFOIL(foilName, Re, a, opts);

            if isfield(tmp, 'alpha_deg') && isfield(tmp, 'CL') && isfield(tmp, 'CD') && isfield(tmp, 'CM')
                if ~isempty(tmp.alpha_deg) && ~isempty(tmp.CL) && ~isempty(tmp.CD) && ~isempty(tmp.CM)
                    alpha_valid(end+1,1) = tmp.alpha_deg(1); %#ok<AGROW>
                    CL_valid(end+1,1)    = tmp.CL(1);        %#ok<AGROW>
                    CD_valid(end+1,1)    = tmp.CD(1);        %#ok<AGROW>
                    CM_valid(end+1,1)    = tmp.CM(1);        %#ok<AGROW>
                end
            end

        catch
            % Skip failed alpha point
        end
    end

    if numel(alpha_valid) < 5
        error('runSingleAirfoilXFOIL_pointwiseFallback:TooFewPoints', ...
            'Fallback produced too few valid points for %s at Re=%.3e.', foilName, Re);
    end

    % Sort by alpha
    [alpha_valid, idx] = sort(alpha_valid);
    CL_valid = CL_valid(idx);
    CD_valid = CD_valid(idx);
    CM_valid = CM_valid(idx);

    % Derived metrics
    % Use a simple linear fit around small alpha region if available
    maskLinear = (alpha_valid >= -2) & (alpha_valid <= 2);

    if nnz(maskLinear) >= 2
        pCL = polyfit(alpha_valid(maskLinear), CL_valid(maskLinear), 1);
        Cla_per_deg = pCL(1);
        alphaL0_deg = -pCL(2) / pCL(1);
    else
        pCL = polyfit(alpha_valid, CL_valid, 1);
        Cla_per_deg = pCL(1);
        alphaL0_deg = -pCL(2) / pCL(1);
    end

    % Cm0 from interpolation near alpha = 0
    Cm0 = interp1(alpha_valid, CM_valid, 0, 'linear', 'extrap');

    % Cl_max from available points
    Cl_max = max(CL_valid);

    % Best L/D from available points
    LD = CL_valid ./ CD_valid;
    LD(~isfinite(LD)) = nan;
    bestLD = max(LD);

    foilOut = struct();
    foilOut.name         = char(foilName);
    foilOut.alpha_deg    = alpha_valid;
    foilOut.CL           = CL_valid;
    foilOut.CD           = CD_valid;
    foilOut.CM           = CM_valid;
    foilOut.Cla_per_deg  = Cla_per_deg;
    foilOut.alphaL0_deg  = alphaL0_deg;
    foilOut.Cm0          = Cm0;
    foilOut.Cl_max       = Cl_max;
    foilOut.bestLD       = bestLD;
end