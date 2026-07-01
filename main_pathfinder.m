% main_pathfinder.m
% Pathfinder X1 â€” top-level sizing / analysis driver.
% Sub-250 g thrust-vectored flying wing. SI units throughout.
%
% Run run_project.m first (adds all src/ paths), or run this after it.
%
% Pipeline (mirrors the Nimbus MDO architecture, adapted for Pathfinder):
%   USER INPUTS -> geometry -> aerodynamics -> propulsion -> mass budget
%                -> stability -> control allocation (NEW) -> mission/energy
%                -> objective (endurance) -> optimize
%
% Version arc:
%   v0.1  scaffold + reused Nimbus modules wired in
%   v0.2  sub-250 g mass budget (src/mass/weightBudget.m)
%   v0.3  thrust-vector control allocation (src/control/thrustVectorAllocation.m)
%   v0.4  three-regime mission (cruise / STOL / hover) + endurance objective
%   v0.5  hover/tailsitter controllability check
%   v1.0  CMA-ES optimization over geometry + propulsion for max endurance @ <249 g

clc; clear; close all;
run_project;   % path setup % This is better than doing it the manual way!

%% ------------------------------------------------------------------
% USER INPUTS  (edit these)
% ------------------------------------------------------------------
cfg.massBudget_g   = 249;          % hard all-up mass cap [g]
cfg.span_m         = 0.75;         % wing span (placeholder) [m]
cfg.cruiseSpeed_ms = 22;           % target cruise speed [m/s]
cfg.regimes        = ["cruise","STOL","hover"];

% Given:
g = 9.81;
roh = 0.93; % worst case scenario, high, hot desert flying.

% Design Variables: [NOT LOCKED IN AT ALL]
AR = 4.5;
wingTapper = 0.45;
QuarterChordSweep = 25;
Dihedral = 3;
Tip_Twist_Geo = -3;
rootAirfoil = "e222.dat";
tipAirfoil = "e230.dat";

% Engineering Assumptions (energy sizing â€” update as modules converge):
ac.W_N        = cfg.massBudget_g * 1e-3 * g;   % [N]   worst-case gross weight
ac.LD         = 8.0;                            % [-]   cruise L/D (flying wing, AR 4.5)
ac.eta_p      = 0.60;                           % [-]   cruise propulsive efficiency
ac.D_prop_m   = 0.127;                          % [m]   5-inch prop diameter (0.127 m)
ac.n_rotors   = 2;                              % [-]   twin pusher bicopter
ac.FM         = 0.55;                           % [-]   rotor figure of merit (small props ~0.55)
ac.eta_elec   = 0.85;                           % [-]   motor + ESC efficiency
ac.eta_hover  = ac.FM * ac.eta_elec;            % [-]   combined hover efficiency
ac.rho_kgm3   = roh;                            % [kg/m^3]  matches worst-case density above
ac.e_bat_Whkg = 150;                            % [Wh/kg]   2S LiPo specific energy
ac.m_bat_kg   = 0.055;                          % [kg]  battery mass (from weightBudget.m)

%% --------------------------------------------------------- 
%                     Mission Profiles
%       ---------------------------------------------

% [A] FPV mode: hand launch -> cruise (no hover phases)
mission_fpv.mode           = 'fpv';
mission_fpv.t_hover_to_s   = 0;          % hand-thrown -- no hover takeoff [s]
mission_fpv.t_hover_ld_s   = 0;          % belly-land or catch -- no hover [s]
mission_fpv.R_cruise_m     = 500;        % sizing range (radius from pilot) [m]
mission_fpv.reserve_factor = 1.2;        % 20% energy reserve [-]

% [B] Camera drone mode: hover takeoff -> 1 mi round trip -> hover land
%     1 mile = 1609.34 m one-way; total cruise distance = 3218.69 m
mission_cam.mode           = 'camera';
mission_cam.t_hover_to_s   = 30;         % hover takeoff [s]
mission_cam.t_hover_ld_s   = 30;         % hover precision landing [s]
mission_cam.R_cruise_m     = 1609.34;    % 1 mile one-way [m]
mission_cam.reserve_factor = 1.3;        % 30% reserve (GPS/transition risk) [-]

%% ------------------------------------------------------------------
% 1) MASS BUDGET  (binding constraint â€” check first)   [NEW]
%% ------------------------------------------------------------------
% mass = weightBudget();   % TODO: returns struct, errors/warns if > cfg.massBudget_g

%% ------------------------------------------------------------------
% 2) GEOMETRY        (ported: wingGeometryDesign, centerbodyGeometry, ...)
% 3) AERODYNAMICS    (ported: airfoil surrogates, spanwiseAeroEstimate, aeroPolarAircraft)
% 4) PROPULSION      (ported: propulsionAnalysis, buildPropSurrogate_*)
% 5) STABILITY       (ported: static/dynamic stability via AVL)
%% ------------------------------------------------------------------
% TODO: wire ported modules with Pathfinder geometry/props.

%% ------------------------------------------------------------------
% 6) CONTROL ALLOCATION â€” thrust vectoring, no control surfaces  [NEW]
%% ------------------------------------------------------------------
% For each regime, verify the twin (throttle, tilt) actuators can produce
% the moments needed for trim + control. Hover is the long-pole risk.
% u = thrustVectorAllocation(desiredWrench, state, cfg);

%% ------------------------------------------------------------------
% 7) MISSION + ENERGY -> ENDURANCE OBJECTIVE
%% ------------------------------------------------------------------
energy_fpv = hoverCruiseEnergy(ac, mission_fpv);   % FPV: size for 500 m radius cruise
energy_cam = hoverCruiseEnergy(ac, mission_cam);   % camera: 1 mi round trip + hover

disp('Pathfinder driver scaffold loaded. Fill in module calls as they come online.');
