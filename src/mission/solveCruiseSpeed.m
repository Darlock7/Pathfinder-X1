function cruiseOut = solveCruiseSpeed(cruiseIn)
% solveCruiseSpeed
%
% Purpose:
%   Solve steady, level-flight cruise speed from thrust-drag balance:
%       T_avail(V) = D(V)
%
% IMPORTANT:
%   This version rejects non-flyable low-speed intersections by enforcing:
%       1) CL_required <= CLmax
%       2) V >= Vmin_cruise
%   and selects the HIGHEST-speed valid intersection.
%
% Inputs:
%   cruiseIn.rho_kgm3
%   cruiseIn.W_N
%   cruiseIn.Sref_m2
%   cruiseIn.CD0
%   cruiseIn.CLmax
%   cruiseIn.e
%   cruiseIn.AR
%   cruiseIn.V_vec_mps
%   cruiseIn.T_vec_N
%
% Optional:
%   cruiseIn.Vmin_cruise_mps
%   cruiseIn.plotFigure
%
% Outputs:
%   cruiseOut.validSolution
%   cruiseOut.V_cruise_mps
%   cruiseOut.T_cruise_N
%   cruiseOut.D_cruise_N
%   cruiseOut.CL_cruise
%   cruiseOut.CD_cruise
%   cruiseOut.LD_cruise
%   cruiseOut.Vmin_cruise_mps
%   cruiseOut.V_vec_mps
%   cruiseOut.T_vec_N
%   cruiseOut.D_vec_N
%   cruiseOut.CL_vec
%   cruiseOut.CD_vec
%   cruiseOut.resid_vec_N

rho   = cruiseIn.rho_kgm3;
W     = cruiseIn.W_N;
Sref  = cruiseIn.Sref_m2;
CD0   = cruiseIn.CD0;
CLmax = cruiseIn.CLmax;
e     = cruiseIn.e;
AR    = cruiseIn.AR;

V_vec = cruiseIn.V_vec_mps(:);
T_vec = cruiseIn.T_vec_N(:);

if isfield(cruiseIn,'Vmin_cruise_mps')
    Vmin_cruise = cruiseIn.Vmin_cruise_mps;
else
    Vs_est = sqrt(2 * W / (rho * Sref * CLmax));
    Vmin_cruise = 1.20 * Vs_est;
end

if isfield(cruiseIn,'plotFigure')
    plotFigure = cruiseIn.plotFigure;
else
    plotFigure = true;
end

k = 1 / (pi * e * AR);

% Protect against V = 0
V_eval = V_vec;
V_eval(V_eval < 0.1) = 0.1;

q = 0.5 * rho .* V_eval.^2;
CL_req = W ./ (q * Sref);
CD_req = CD0 + k .* CL_req.^2;
D_vec  = q .* Sref .* CD_req;

resid = T_vec - D_vec;

% Valid flight region
isFlyable = (CL_req <= CLmax) & (V_eval >= Vmin_cruise);

% Mask out invalid region
resid_valid = resid;
resid_valid(~isFlyable) = NaN;

% Find all valid sign changes
idxCandidates = [];
for i = 1:length(V_eval)-1
    if ~isnan(resid_valid(i)) && ~isnan(resid_valid(i+1))
        if resid_valid(i) * resid_valid(i+1) <= 0
            idxCandidates(end+1) = i; %#ok<AGROW>
        end
    end
end

if isempty(idxCandidates)
    cruiseOut = struct();
    cruiseOut.validSolution   = false;
    cruiseOut.V_cruise_mps    = NaN;
    cruiseOut.T_cruise_N      = NaN;
    cruiseOut.D_cruise_N      = NaN;
    cruiseOut.CL_cruise       = NaN;
    cruiseOut.CD_cruise       = NaN;
    cruiseOut.LD_cruise       = NaN;
    cruiseOut.Vmin_cruise_mps = Vmin_cruise;

    cruiseOut.V_vec_mps       = V_vec;
    cruiseOut.T_vec_N         = T_vec;
    cruiseOut.D_vec_N         = D_vec;
    cruiseOut.CL_vec          = CL_req;
    cruiseOut.CD_vec          = CD_req;
    cruiseOut.resid_vec_N     = resid;
    return;
end

% Choose HIGHEST-speed valid crossing
idxCross = idxCandidates(end);

V1 = V_eval(idxCross);
V2 = V_eval(idxCross+1);
R1 = resid(idxCross);
R2 = resid(idxCross+1);

V_cruise = V1 - R1 * (V2 - V1) / (R2 - R1);

q_cr = 0.5 * rho * V_cruise^2;
CL_cr = W / (q_cr * Sref);
CD_cr = CD0 + k * CL_cr^2;
D_cr  = q_cr * Sref * CD_cr;
T_cr  = interp1(V_vec, T_vec, V_cruise, 'linear', 'extrap');
LD_cr = CL_cr / CD_cr;

if plotFigure
    figure('Name','Cruise Speed Solve','NumberTitle','off');
    plot(V_vec, T_vec, 'LineWidth', 2); hold on;
    plot(V_vec, D_vec, 'LineWidth', 2);
    xline(Vmin_cruise, '--', 'LineWidth', 1.0);
    plot(V_cruise, T_cr, 'o', 'LineWidth', 1.5, 'MarkerSize', 8);
    grid on;
    xlabel('V [m/s]');
    ylabel('Force [N]');
    title('Cruise Speed from Valid Thrust-Drag Intersection');
    legend('Available thrust','Required drag','V_{min,cruise}','Cruise solution', ...
        'Location','best');

    txt1 = sprintf('V_{cruise} = %.2f m/s', V_cruise);
    txt2 = sprintf('T = D = %.2f N', D_cr);
    txt3 = sprintf('C_L = %.3f', CL_cr);
    txt4 = sprintf('L/D = %.2f', LD_cr);
    text(V_cruise + 0.4, D_cr, {txt1, txt2, txt3, txt4}, 'FontSize', 11);
end

cruiseOut = struct();
cruiseOut.validSolution   = true;
cruiseOut.V_cruise_mps    = V_cruise;
cruiseOut.T_cruise_N      = T_cr;
cruiseOut.D_cruise_N      = D_cr;
cruiseOut.CL_cruise       = CL_cr;
cruiseOut.CD_cruise       = CD_cr;
cruiseOut.LD_cruise       = LD_cr;
cruiseOut.Vmin_cruise_mps = Vmin_cruise;

cruiseOut.V_vec_mps       = V_vec;
cruiseOut.T_vec_N         = T_vec;
cruiseOut.D_vec_N         = D_vec;
cruiseOut.CL_vec          = CL_req;
cruiseOut.CD_vec          = CD_req;
cruiseOut.resid_vec_N     = resid;

end