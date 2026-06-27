function perfOut = updatePerformanceState(perfIn)
% updatePerformanceState
%
% Purpose:
%   Update key aircraft performance quantities using current
%   aero, propulsion, geometry, and loaded weight.
%
% Outputs:
%   - Stall speed
%   - Takeoff speed estimate
%   - Cruise speed from thrust-drag balance
%   - Max climb gradient at selected climb speed
%   - Max climb rate at selected climb speed

rho   = perfIn.rho_kgm3;      % [kg/m^3]
W     = perfIn.W_N;           % [N]
Sref  = perfIn.Sref_m2;       % [m^2]
CD0   = perfIn.CD0;           % [-]
CLmax = perfIn.CLmax;         % [-]
e     = perfIn.e;             % [-]
AR    = perfIn.AR;            % [-]

V_vec = perfIn.V_vec_mps(:);  % [m/s]
T_vec = perfIn.T_vec_N(:);    % [N]

if isfield(perfIn,'V_climb_eval_mps')
    Vclimb = perfIn.V_climb_eval_mps;
else
    Vclimb = 14.0;            % [m/s]
end

if isfield(perfIn,'CLTO_frac')
    CLTO_frac = perfIn.CLTO_frac;
else
    CLTO_frac = 0.8;          % [-]
end

k = 1/(pi*e*AR);

%% ---------------- Stall speed ----------------
Vs_mps = sqrt(2*W / (rho*Sref*CLmax));   % [m/s]

%% ---------------- Takeoff speed estimate ----------------
CLTO = CLTO_frac * CLmax;
VTO_mps = sqrt(2*W / (rho*Sref*CLTO));   % [m/s]

%% ---------------- Cruise speed solve ----------------
cruiseIn = struct();
cruiseIn.rho_kgm3 = rho;
cruiseIn.W_N      = W;
cruiseIn.Sref_m2  = Sref;
cruiseIn.CD0      = CD0;
cruiseIn.CLmax    = CLmax;
cruiseIn.e        = e;
cruiseIn.AR       = AR;
cruiseIn.V_vec_mps = V_vec;
cruiseIn.T_vec_N   = T_vec;
cruiseIn.Vmin_cruise_mps = 1.20 * Vs_mps;
cruiseIn.plotFigure = false;

cruiseOut = solveCruiseSpeed(cruiseIn);

%% ---------------- Climb performance at selected climb speed ----------------
q_cl = 0.5 * rho * Vclimb^2;
CL_cl = W / (q_cl*Sref);
CD_cl = CD0 + k*CL_cl^2;
Dclimb_N = q_cl*Sref*CD_cl;
Tclimb_N = interp1(V_vec, T_vec, Vclimb, 'linear', 'extrap');

G_climb_max = (Tclimb_N - Dclimb_N)/W;          % [-]
ROC_climb_max_mps = Vclimb * G_climb_max;       % [m/s]

%% ---------------- Output ----------------
perfOut = struct();

% Stall / takeoff
perfOut.Vs_mps            = Vs_mps;
perfOut.CLTO              = CLTO;
perfOut.VTO_mps           = VTO_mps;

% Cruise
perfOut.validCruise       = cruiseOut.validSolution;
perfOut.Vcruise_mps       = cruiseOut.V_cruise_mps;
perfOut.CLcruise          = cruiseOut.CL_cruise;
perfOut.CDcruise          = cruiseOut.CD_cruise;
perfOut.LDcruise          = cruiseOut.LD_cruise;

% Climb
perfOut.Vclimb_eval_mps   = Vclimb;
perfOut.Tclimb_N          = Tclimb_N;
perfOut.Dclimb_N          = Dclimb_N;
perfOut.G_climb_max       = G_climb_max;
perfOut.ROC_climb_max_mps = ROC_climb_max_mps;

end