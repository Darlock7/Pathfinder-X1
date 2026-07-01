% main_pathfinder.m
% Pathfinder X1 — top-level sizing / analysis driver.
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

% Design Variables:
AR = 4.5;
wingTapper = 0.45;
QuarterChordSweep = 25;
Dihedral = 3;
Tip_Twist_Geo = -3;
rootAirfoil = "e222.dat";
tipAirfoil = "e230.dat";


%% --------------------------------------------------------- 
%                     Mission Profiles
%       ---------------------------------------------

% [A] FPV mode:









% [B] Camera Drone Mode: 



%% ------------------------------------------------------------------
% 1) MASS BUDGET  (binding constraint — check first)   [NEW]
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
% 6) CONTROL ALLOCATION — thrust vectoring, no control surfaces  [NEW]
%% ------------------------------------------------------------------
% For each regime, verify the twin (throttle, tilt) actuators can produce
% the moments needed for trim + control. Hover is the long-pole risk.
% u = thrustVectorAllocation(desiredWrench, state, cfg);

%% ------------------------------------------------------------------
% 7) MISSION + ENERGY -> ENDURANCE OBJECTIVE
%% ------------------------------------------------------------------
% TODO: three-regime mission profile; endurance from energy model.

disp('Pathfinder driver scaffold loaded. Fill in module calls as they come online.');
