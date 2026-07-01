% main_pathfinder.m
% Pathfinder X1 — top-level sizing / analysis driver.
% Sub-250 g thrust-vectored flying wing. SI units throughout.
%
% Run run_project.m first (adds all src/ paths), or run this after it.
%
% Pipeline (mirrors Nimbus MDO architecture, adapted for Pathfinder):
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
run_project;   % path setup

%% =================== Run Flags =========================
showPlots        = true;   % true = show mission profile + energy plots
writeSpreadsheet = false;  % true = write energy results to Pathfinder Mission.xlsx
if ~showPlots
    set(0,'DefaultFigureVisible','off');
else
    set(0,'DefaultFigureVisible','on');
end
%% =======================================================

%% ------------------------------------------------------------------
% USER INPUTS  (edit these)
% ------------------------------------------------------------------
cfg.massBudget_g   = 249;          % hard all-up mass cap [g]
cfg.span_m         = 0.75;         % wing span (placeholder) [m]
cfg.cruiseSpeed_ms = 22;           % target cruise speed [m/s]
cfg.regimes        = ["cruise","STOL","hover"];

% Given:
g   = 9.81;                        % [m/s^2]
roh = 0.93;                        % [kg/m^3]  worst case: high/hot desert

% Design Variables: (NOT LOCKED IN)
AR                = 4.5;
wingTapper        = 0.45;
QuarterChordSweep = 25;
Dihedral          = 3;
Tip_Twist_Geo     = -3;
rootAirfoil       = "e222.dat";
tipAirfoil        = "e230.dat";

% Engineering Assumptions (energy sizing — update as modules converge):
ac.W_N        = cfg.massBudget_g * 1e-3 * g;   % [N]      worst-case gross weight
ac.LD         = 8.0;                            % [-]      cruise L/D  (AR 4.5 flying wing)
ac.eta_p      = 0.60;                           % [-]      cruise propulsive efficiency
ac.D_prop_m   = 0.127;                          % [m]      5-inch prop diameter
ac.n_rotors   = 2;                              % [-]      twin pusher bicopter
ac.FM         = 0.55;                           % [-]      rotor figure of merit (small props)
ac.eta_elec   = 0.85;                           % [-]      motor + ESC efficiency
ac.eta_hover  = ac.FM * ac.eta_elec;            % [-]      combined hover efficiency
ac.rho_kgm3   = roh;                            % [kg/m^3] matches worst-case density above
ac.e_bat_Whkg = 150;                            % [Wh/kg]  2S LiPo specific energy
ac.m_bat_kg   = 0.055;                          % [kg]     battery mass (from weightBudget.m)

%% =========================================================
%  Mission Profiles   (Ref: Pathfinder Mission.xlsx)
% =========================================================
%
%  Regime Table:
%   Regime           Speed      Lift mode              Power mult. (rel cruise)
%   Cruise           22  m/s    Wing-borne              1.00
%   Low-speed        13  m/s    Wing + thrust assist    1.3-1.8
%   STOL              5  m/s    Thrust-assisted          2.0-3.0
%   Harrier/hover    ~0  m/s    Thrust-borne             3.0-5.0+
%
%  Note: multipliers are spreadsheet estimates; hover is physics-derived
%  (actuator disk). STOL/low-speed actual values depend on tilt angle,
%  CL margin, and vectoring torque demand — refine when aero model is live.

% --- Regime speeds [m/s] ---
V_cruise_mps = 22;
V_low_mps    = 13;
V_stol_mps   =  5;

% --- Power multipliers vs cruise (spreadsheet midpoints) ---
mult_stol    = 2.50;   % [2.0 – 3.0]
mult_low     = 1.55;   % [1.3 – 1.8]
% Hover: computed from actuator disk theory in hoverCruiseEnergy

%% ==========================================
%  [A] Mission A — FPV Mode
% ==========================================
%  Phase          | dt [min]| dt [s] | Regime     | h_0 [m] | h_f [m]
%  Hand launch    |    3    |   180  | STOL       |    0    |   20
%  Wait/command   |    5    |   300  | Low-speed  |   20    |   20
%  FPV cruise     |   20    |  1200  | Cruise     |   20    |   20
%  Hand recovery  |    4    |   240  | STOL       |   20    |    0
%  TOTAL          |   32    |  1920  |            |         |

t0_A = 0;
t1_A = t0_A + 180;    % end STOL launch       [s]
t2_A = t1_A + 300;    % end low-speed wait    [s]
t3_A = t2_A + 1200;   % end FPV cruise        [s]
t4_A = t3_A + 240;    % end STOL recovery     [s]

t_bp_A = [t0_A, t1_A, t2_A, t3_A, t4_A];
h_bp_A = [0,    20,    20,   20,   0  ];      % altitude [m] at each event

% hoverCruiseEnergy struct (cruise-only; STOL handled separately below)
mission_fpv.mode           = 'fpv';
mission_fpv.t_hover_to_s   = 0;
mission_fpv.t_hover_ld_s   = 0;
mission_fpv.R_cruise_m     = V_cruise_mps * (t3_A - t2_A) / 2;   % one-way cruise [m]
mission_fpv.reserve_factor = 1.2;

%% ==========================================
%  [B] Mission B — Camera Drone Mode
% ==========================================
%  Phase           | dt [min]| dt [s] | Regime     | h_0 [m] | h_f [m]
%  Hand launch     |    3    |   180  | STOL       |    0    |   20
%  Wait/command    |    5    |   300  | Low-speed  |   20    |   20
%  Move to target  |    5    |   300  | Low-speed  |   20    |   10
%  Static record   |    5    |   300  | Hover      |   10    |   30
%  Return to pilot |    2    |   120  | Cruise     |   30    |   20
%  Hand recovery   |    4    |   240  | STOL       |   20    |    0
%  TOTAL           |   24    |  1440  |            |         |

t0_B = 0;
t1_B = t0_B + 180;    % end STOL launch           [s]
t2_B = t1_B + 300;    % end low-speed wait         [s]
t3_B = t2_B + 300;    % end low-speed move         [s]
t4_B = t3_B + 300;    % end hover / static record  [s]
t5_B = t4_B + 120;    % end cruise return          [s]
t6_B = t5_B + 240;    % end STOL recovery          [s]

t_bp_B = [t0_B, t1_B, t2_B, t3_B, t4_B, t5_B, t6_B];
h_bp_B = [0,    20,   20,   10,   30,   20,   0  ];   % altitude [m]

% hoverCruiseEnergy struct (hover = static record; STOL/low-speed separate)
mission_cam.mode           = 'camera';
mission_cam.t_hover_to_s   = t4_B - t3_B;                         % 600 s hover [s]
mission_cam.t_hover_ld_s   = 0;                                    % STOL recovery, not hover
mission_cam.R_cruise_m     = V_cruise_mps * (t5_B - t4_B) / 2;   % return leg one-way [m]
mission_cam.reserve_factor = 1.3;

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

% --- Actuator-disk hover power + simplified cruise model ---
energy_fpv = hoverCruiseEnergy(ac, mission_fpv);
energy_cam = hoverCruiseEnergy(ac, mission_cam);
P_hover_W  = energy_cam.P_hover_W;              % [W] electrical hover power

% --- Per-regime power levels [W] ---
P_cruise_W = (ac.W_N / ac.LD) * V_cruise_mps / ac.eta_p;
P_low_W    = mult_low  * P_cruise_W;
P_stol_W   = mult_stol * P_cruise_W;

fprintf('\n--- Regime Power Levels ---\n');
fprintf('  Cruise     (%2.0f m/s): %6.2f W  (1.00x)\n', V_cruise_mps, P_cruise_W);
fprintf('  Low-speed  (%2.0f m/s): %6.2f W  (%.2fx mult)\n', V_low_mps, P_low_W, mult_low);
fprintf('  STOL       (%2.0f m/s): %6.2f W  (%.2fx mult)\n', V_stol_mps, P_stol_W, mult_stol);
fprintf('  Hover      (~0 m/s): %6.2f W  (%.2fx cruise, actuator disk)\n', ...
        P_hover_W, P_hover_W/P_cruise_W);

% ============================================================
%  Mission A: Phase-by-phase energy
% ============================================================
dt  = 1;                          % time resolution [s]
t_A = (t0_A : dt : t4_A).';

P_A = zeros(size(t_A));
P_A(t_A <  t1_A)                  = P_stol_W;    % STOL launch
P_A(t_A >= t1_A & t_A < t2_A)     = P_low_W;     % low-speed wait
P_A(t_A >= t2_A & t_A < t3_A)     = P_cruise_W;  % FPV cruise
P_A(t_A >= t3_A)                  = P_stol_W;    % STOL recovery

E_A_cum_Wh = cumsum(P_A * dt) / 3600;            % cumulative energy [Wh]

dE_A_stol_launch = P_stol_W   * (t1_A - t0_A) / 3600;
dE_A_low_wait    = P_low_W    * (t2_A - t1_A) / 3600;
dE_A_cruise      = P_cruise_W * (t3_A - t2_A) / 3600;
dE_A_stol_rec    = P_stol_W   * (t4_A - t3_A) / 3600;
E_mission_A      = dE_A_stol_launch + dE_A_low_wait + dE_A_cruise + dE_A_stol_rec;
E_design_A       = mission_fpv.reserve_factor * E_mission_A;
E_avail_Wh       = energy_fpv.E_avail_Wh;

fprintf('\n--- Mission A (FPV) Phase Energy ---\n');
fprintf('  STOL launch    (%3.0f s): %6.3f Wh\n', t1_A-t0_A, dE_A_stol_launch);
fprintf('  Low-spd wait   (%3.0f s): %6.3f Wh\n', t2_A-t1_A, dE_A_low_wait);
fprintf('  FPV cruise     (%3.0f s): %6.3f Wh\n', t3_A-t2_A, dE_A_cruise);
fprintf('  STOL recovery  (%3.0f s): %6.3f Wh\n', t4_A-t3_A, dE_A_stol_rec);
fprintf('  Mission total:           %6.3f Wh\n',  E_mission_A);
fprintf('  Design (x%.1f rsv):      %6.3f Wh\n',  mission_fpv.reserve_factor, E_design_A);
fprintf('  Battery available:       %6.3f Wh\n',  E_avail_Wh);
if E_design_A <= E_avail_Wh
    fprintf('  STATUS: FEASIBLE  (+%.3f Wh margin)\n', E_avail_Wh - E_design_A);
else
    fprintf('  STATUS: *** INFEASIBLE (short %.3f Wh) ***\n', E_design_A - E_avail_Wh);
end

% ============================================================
%  Mission B: Phase-by-phase energy
% ============================================================
t_B = (t0_B : dt : t6_B).';

P_B = zeros(size(t_B));
P_B(t_B <  t1_B)                  = P_stol_W;    % STOL launch
P_B(t_B >= t1_B & t_B < t2_B)     = P_low_W;     % low-speed wait
P_B(t_B >= t2_B & t_B < t3_B)     = P_low_W;     % low-speed move to target
P_B(t_B >= t3_B & t_B < t4_B)     = P_hover_W;   % hover / static record
P_B(t_B >= t4_B & t_B < t5_B)     = P_cruise_W;  % cruise return
P_B(t_B >= t5_B)                  = P_stol_W;    % STOL recovery

E_B_cum_Wh = cumsum(P_B * dt) / 3600;

dE_B_stol_launch = P_stol_W   * (t1_B - t0_B) / 3600;
dE_B_low_wait    = P_low_W    * (t2_B - t1_B) / 3600;
dE_B_low_move    = P_low_W    * (t3_B - t2_B) / 3600;
dE_B_hover       = P_hover_W  * (t4_B - t3_B) / 3600;
dE_B_cruise_ret  = P_cruise_W * (t5_B - t4_B) / 3600;
dE_B_stol_rec    = P_stol_W   * (t6_B - t5_B) / 3600;
E_mission_B      = dE_B_stol_launch + dE_B_low_wait + dE_B_low_move + ...
                   dE_B_hover + dE_B_cruise_ret + dE_B_stol_rec;
E_design_B       = mission_cam.reserve_factor * E_mission_B;

fprintf('\n--- Mission B (Camera) Phase Energy ---\n');
fprintf('  STOL launch    (%3.0f s): %6.3f Wh\n', t1_B-t0_B, dE_B_stol_launch);
fprintf('  Low-spd wait   (%3.0f s): %6.3f Wh\n', t2_B-t1_B, dE_B_low_wait);
fprintf('  Low-spd move   (%3.0f s): %6.3f Wh\n', t3_B-t2_B, dE_B_low_move);
fprintf('  Hover record   (%3.0f s): %6.3f Wh\n', t4_B-t3_B, dE_B_hover);
fprintf('  Cruise return  (%3.0f s): %6.3f Wh\n', t5_B-t4_B, dE_B_cruise_ret);
fprintf('  STOL recovery  (%3.0f s): %6.3f Wh\n', t6_B-t5_B, dE_B_stol_rec);
fprintf('  Mission total:           %6.3f Wh\n',  E_mission_B);
fprintf('  Design (x%.1f rsv):      %6.3f Wh\n',  mission_cam.reserve_factor, E_design_B);
fprintf('  Battery available:       %6.3f Wh\n',  E_avail_Wh);
if E_design_B <= E_avail_Wh
    fprintf('  STATUS: FEASIBLE  (+%.3f Wh margin)\n', E_avail_Wh - E_design_B);
else
    fprintf('  STATUS: *** INFEASIBLE (short %.3f Wh) ***\n', E_design_B - E_avail_Wh);
end

% ============================================================
%  Battery Mass Fraction Analysis
%   BMF = E_design / (e_bat * m_total)
%   Ref: VTOL_SizingV9Mk2.m §"BATTERY MASS FRACTION"
% ============================================================
m_total_kg   = cfg.massBudget_g * 1e-3;          % [kg] AUW cap
m_non_bat_kg = 0.180;   % [kg] all non-battery components (from weightBudget.m: 235g-55g)

m_bat_req_A  = E_design_A / ac.e_bat_Whkg;        % [kg] battery required for Mission A
m_bat_req_B  = E_design_B / ac.e_bat_Whkg;        % [kg] battery required for Mission B
f_bat_cur    = ac.m_bat_kg  / m_total_kg;          % [-]  current battery fraction
f_bat_req_A  = m_bat_req_A  / m_total_kg;          % [-]  required for Mission A
f_bat_req_B  = m_bat_req_B  / m_total_kg;          % [-]  required for Mission B
AUW_req_A    = (m_non_bat_kg + m_bat_req_A) * 1e3; % [g]  if bat swapped for req'd size
AUW_req_B    = (m_non_bat_kg + m_bat_req_B) * 1e3; % [g]  if bat swapped for req'd size

fprintf('\n====================================================\n');
fprintf(' Battery Mass Fraction Analysis\n');
fprintf('====================================================\n');
fprintf(' AUW cap:              %6.1f g  (%5.3f kg)\n', cfg.massBudget_g, m_total_kg);
fprintf(' Non-bat components:   %6.1f g\n', m_non_bat_kg*1e3);
fprintf(' Current battery:      %6.1f g  f_bat = %.3f\n', ac.m_bat_kg*1e3, f_bat_cur);
fprintf(' Battery spec. energy: %6.0f Wh/kg\n', ac.e_bat_Whkg);
fprintf(' Available energy:     %6.3f Wh\n', E_avail_Wh);
fprintf('\n Mission A (FPV, 32 min):\n');
fprintf('   E_design:           %6.3f Wh\n',  E_design_A);
fprintf('   Bat. mass req:      %6.1f g   f_bat = %.3f\n', m_bat_req_A*1e3, f_bat_req_A);
fprintf('   AUW w/ req bat:     %6.1f g   (limit = 249 g)\n', AUW_req_A);
if AUW_req_A <= cfg.massBudget_g
    fprintf('   => Fits within 249 g budget.\n');
else
    fprintf('   => OVER LIMIT by %.1f g — reduce mission or battery spec energy too low.\n', ...
            AUW_req_A - cfg.massBudget_g);
end
fprintf('\n Mission B (Camera, 24 min):\n');
fprintf('   E_design:           %6.3f Wh\n',  E_design_B);
fprintf('   Bat. mass req:      %6.1f g   f_bat = %.3f\n', m_bat_req_B*1e3, f_bat_req_B);
fprintf('   AUW w/ req bat:     %6.1f g   (limit = 249 g)\n', AUW_req_B);
if AUW_req_B <= cfg.massBudget_g
    fprintf('   => Fits within 249 g budget.\n');
else
    fprintf('   => OVER LIMIT by %.1f g — reduce hover time or increase e_bat.\n', ...
            AUW_req_B - cfg.massBudget_g);
end
fprintf('====================================================\n\n');

% ============================================================
%  Optional: write results to Pathfinder Mission.xlsx
% ============================================================
if writeSpreadsheet
    xlPath = fullfile('reference material', 'docs', 'Pathfinder Mission.xlsx');
    header = {'Parameter', 'Mission A (FPV)', 'Mission B (Camera)', 'Units'};
    data = {
        'E_mission',       E_mission_A,       E_mission_B,       'Wh';
        'E_design',        E_design_A,        E_design_B,        'Wh';
        'E_avail',         E_avail_Wh,        E_avail_Wh,        'Wh';
        'P_hover',         0,                 P_hover_W,         'W';
        'P_cruise',        P_cruise_W,        P_cruise_W,        'W';
        'P_stol',          P_stol_W,          P_stol_W,          'W';
        'm_bat_required',  m_bat_req_A*1e3,   m_bat_req_B*1e3,   'g';
        'f_bat_required',  f_bat_req_A,       f_bat_req_B,       '-';
        'AUW_with_req_bat',AUW_req_A,         AUW_req_B,         'g';
        'Feasible',        E_design_A<=E_avail_Wh, E_design_B<=E_avail_Wh, 'bool';
    };
    writecell([header; data], xlPath, 'Sheet', 'Energy Analysis');
    fprintf('Energy results written to: %s\n', xlPath);
end

% ============================================================
%  Plots
% ============================================================
if showPlots

    % --- shared formatting ---
    phaseClr  = [0.3 0.3 0.3];   % gray for xline labels
    capClrSol = [0.85 0.10 0.10]; % red solid  = hard battery limit
    capClrDsh = [0.95 0.50 0.05]; % orange dash = reserve limit

    % --- Mission A (FPV) ---
    figure('Name','Mission A — FPV Mode','NumberTitle','off', ...
           'Position',[100 200 820 560]);
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    % Altitude profile
    ax1A = nexttile;
    h_vec_A = interp1(t_bp_A, h_bp_A, t_A, 'linear');
    plot(t_A/60, h_vec_A, 'b-', 'LineWidth', 2); grid on; hold on;
    ylabel('Altitude [m]'); title('Mission A — FPV Mode');
    ylim([-2, 30]);
    xline(t1_A/60,'--','Color',phaseClr,'Label','End STOL launch', ...
          'LabelHorizontalAlignment','right','FontSize',8);
    xline(t2_A/60,'--','Color',phaseClr,'Label','End low-spd wait', ...
          'LabelHorizontalAlignment','right','FontSize',8);
    xline(t3_A/60,'--','Color',phaseClr,'Label','End cruise', ...
          'LabelHorizontalAlignment','right','FontSize',8);

    % Energy + power
    ax2A = nexttile;
    yyaxis left
    plot(t_A/60, E_A_cum_Wh, 'b-', 'LineWidth', 2); hold on; grid on;
    yline(E_avail_Wh,                   '-', 'Color', capClrSol, 'LineWidth', 1.5, ...
          'Label', sprintf('Bat cap %.2f Wh', E_avail_Wh), ...
          'LabelHorizontalAlignment', 'left', 'FontSize', 8);
    yline(E_avail_Wh / mission_fpv.reserve_factor, '--', 'Color', capClrDsh, 'LineWidth', 1.2, ...
          'Label', sprintf('Reserve limit %.2f Wh', E_avail_Wh/mission_fpv.reserve_factor), ...
          'LabelHorizontalAlignment', 'left', 'FontSize', 8);
    ylabel('Cumul. Energy [Wh]');
    yyaxis right
    plot(t_A/60, P_A, 'r-', 'LineWidth', 1.5);
    ylabel('Power [W]');
    xlabel('Time [min]');
    xline(t1_A/60,'--','Color',phaseClr);
    xline(t2_A/60,'--','Color',phaseClr);
    xline(t3_A/60,'--','Color',phaseClr);
    legend({'Energy [Wh]','','','Power [W]'},'Location','northwest','FontSize',8);
    linkaxes([ax1A ax2A],'x');

    % --- Mission B (Camera) ---
    figure('Name','Mission B — Camera Drone Mode','NumberTitle','off', ...
           'Position',[950 200 820 560]);
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    % Altitude profile
    ax1B = nexttile;
    h_vec_B = interp1(t_bp_B, h_bp_B, t_B, 'linear');
    plot(t_B/60, h_vec_B, 'Color',[0.1 0.6 0.1],'LineWidth', 2); grid on; hold on;
    ylabel('Altitude [m]'); title('Mission B — Camera Drone Mode');
    ylim([-2, 40]);
    xline(t1_B/60,'--','Color',phaseClr,'Label','End STOL launch', ...
          'LabelHorizontalAlignment','right','FontSize',8);
    xline(t2_B/60,'--','Color',phaseClr,'Label','End wait', ...
          'LabelHorizontalAlignment','right','FontSize',8);
    xline(t3_B/60,'--','Color',phaseClr,'Label','End move', ...
          'LabelHorizontalAlignment','right','FontSize',8);
    xline(t4_B/60,'--','Color',phaseClr,'Label','End hover record', ...
          'LabelHorizontalAlignment','right','FontSize',8);
    xline(t5_B/60,'--','Color',phaseClr,'Label','End cruise return', ...
          'LabelHorizontalAlignment','right','FontSize',8);

    % Energy + power
    ax2B = nexttile;
    yyaxis left
    plot(t_B/60, E_B_cum_Wh, 'Color',[0.1 0.6 0.1],'LineWidth', 2); hold on; grid on;
    yline(E_avail_Wh,                   '-', 'Color', capClrSol, 'LineWidth', 1.5, ...
          'Label', sprintf('Bat cap %.2f Wh', E_avail_Wh), ...
          'LabelHorizontalAlignment', 'left', 'FontSize', 8);
    yline(E_avail_Wh / mission_cam.reserve_factor, '--', 'Color', capClrDsh, 'LineWidth', 1.2, ...
          'Label', sprintf('Reserve limit %.2f Wh', E_avail_Wh/mission_cam.reserve_factor), ...
          'LabelHorizontalAlignment', 'left', 'FontSize', 8);
    ylabel('Cumul. Energy [Wh]');
    yyaxis right
    plot(t_B/60, P_B, 'r-', 'LineWidth', 1.5);
    ylabel('Power [W]');
    xlabel('Time [min]');
    xline(t1_B/60,'--','Color',phaseClr);
    xline(t2_B/60,'--','Color',phaseClr);
    xline(t3_B/60,'--','Color',phaseClr);
    xline(t4_B/60,'--','Color',phaseClr);
    xline(t5_B/60,'--','Color',phaseClr);
    legend({'Energy [Wh]','','','Power [W]'},'Location','northwest','FontSize',8);
    linkaxes([ax1B ax2B],'x');

end % showPlots

disp('Pathfinder driver scaffold loaded. Fill in module calls as they come online.');
