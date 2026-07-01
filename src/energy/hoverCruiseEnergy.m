function energy = hoverCruiseEnergy(ac, mission)
% hoverCruiseEnergy   Hover + cruise energy budget for X1 mission modes.
%
% Supported modes
%   'fpv'    — hand launch, no hover (t_hover_* = 0)
%   'camera' — hover takeoff → round-trip cruise → hover landing
%
% Hover physics: actuator-disk theory, identical convention to
%   VTOL_SizingV9Mk2.m §"HOVER BATTERY FRACTION"
%   P_ideal = T^(3/2) / sqrt(2*rho*A_disk)
%   P_hover = P_ideal / eta_hover,  where eta_hover = FM * eta_motor_esc
%
% Climb approx: hover power is assumed constant during short vertical takeoff
%   (valid when t_hover_to_s is short; axial climb correction < 5% for
%    V_c < 0.5 * v_i, which holds for gentle climb-outs at this scale).
%
% Cruise: Breguet-electric
%   E_cruise = (W/LD) * (R_total / eta_p),   R_total = 2 * R_cruise_m
%
% Battery mass fraction (BMF) follows VTOL_SizingV9Mk2.m:
%   BMF = E_segment_J / (e_bat_J_per_kg * m_bat_kg)
%
% ---- ac struct (SI unless noted) ----
%   W_N          [N]       gross weight
%   LD           [-]       cruise lift-to-drag ratio
%   eta_p        [-]       cruise propulsive efficiency (battery → thrust power)
%   D_prop_m     [m]       single propeller diameter
%   n_rotors     [-]       number of rotors (X1 = 2)
%   FM           [-]       rotor figure of merit (~0.55 for small props)
%   eta_elec     [-]       motor + ESC efficiency (~0.85)
%   eta_hover    [-]       combined hover efficiency = FM * eta_elec
%   rho_kgm3     [kg/m^3]  air density
%   e_bat_Whkg   [Wh/kg]   battery specific energy
%   m_bat_kg     [kg]      battery mass
%
% ---- mission struct ----
%   mode           'fpv' | 'camera'
%   t_hover_to_s   [s]   hover takeoff duration  (set 0 for FPV)
%   t_hover_ld_s   [s]   hover landing duration  (set 0 for FPV)
%   R_cruise_m     [m]   one-way cruise range; round trip = 2×
%   reserve_factor [-]   energy margin multiplier (1.2–1.3 typical)
%
% ---- energy output struct ----
%   DL_Nm2         [N/m^2]  disk loading
%   A_disk_m2      [m^2]    total rotor disk area
%   P_hover_W      [W]      electrical hover power draw
%   E_hover_to_Wh  [Wh]     hover takeoff energy
%   E_hover_ld_Wh  [Wh]     hover landing energy
%   E_cruise_Wh    [Wh]     cruise energy (round trip)
%   E_mission_Wh   [Wh]     mission total (no reserve)
%   E_design_Wh    [Wh]     mission × reserve_factor
%   E_avail_Wh     [Wh]     battery available energy
%   BMF_hover      [-]      hover battery mass fraction
%   BMF_cruise     [-]      cruise battery mass fraction
%   BMF_total      [-]      design total battery mass fraction
%   feasible       [bool]   true if E_design <= E_avail
%   margin_Wh      [Wh]     energy margin (positive = good)

    % --- Rotor disk geometry ---
    r_prop    = ac.D_prop_m / 2;                        % [m]
    A_disk_m2 = ac.n_rotors * pi * r_prop^2;            % total disk area [m^2]
    DL_Nm2    = ac.W_N / A_disk_m2;                     % disk loading [N/m^2]

    % --- Hover power (actuator-disk + efficiency) ---
    P_hover_ideal_W = ac.W_N^1.5 / sqrt(2 * ac.rho_kgm3 * A_disk_m2);  % ideal [W]
    P_hover_W       = P_hover_ideal_W / ac.eta_hover;                    % electrical [W]

    % --- Hover segment energies ---
    E_hover_to_J = P_hover_W * mission.t_hover_to_s;    % [J]
    E_hover_ld_J = P_hover_W * mission.t_hover_ld_s;    % [J]

    % --- Cruise energy (round trip: outbound + return) ---
    R_total_m  = 2 * mission.R_cruise_m;                              % [m]
    E_cruise_J = (ac.W_N / ac.LD) * (R_total_m / ac.eta_p);          % [J]

    % --- Mission totals ---
    E_mission_J = E_hover_to_J + E_cruise_J + E_hover_ld_J;          % [J]
    E_design_J  = mission.reserve_factor * E_mission_J;               % [J]

    % --- Convert to Wh ---
    E_hover_to_Wh = E_hover_to_J  / 3600;
    E_hover_ld_Wh = E_hover_ld_J  / 3600;
    E_cruise_Wh   = E_cruise_J    / 3600;
    E_mission_Wh  = E_mission_J   / 3600;
    E_design_Wh   = E_design_J    / 3600;

    % --- Battery availability ---
    E_avail_Wh = ac.e_bat_Whkg * ac.m_bat_kg;           % [Wh]
    E_avail_J  = E_avail_Wh * 3600;                     % [J]

    % --- Battery mass fractions (Ref: VTOL_SizingV9Mk2.m) ---
    BMF_hover  = (E_hover_to_J + E_hover_ld_J) / E_avail_J;
    BMF_cruise = E_cruise_J  / E_avail_J;
    BMF_total  = E_design_J  / E_avail_J;               % includes reserve

    % --- Feasibility ---
    margin_Wh = E_avail_Wh - E_design_Wh;
    feasible  = (margin_Wh >= 0);

    % --- Pack output struct ---
    energy.mode          = mission.mode;
    energy.DL_Nm2        = DL_Nm2;
    energy.A_disk_m2     = A_disk_m2;
    energy.P_hover_W     = P_hover_W;
    energy.E_hover_to_Wh = E_hover_to_Wh;
    energy.E_hover_ld_Wh = E_hover_ld_Wh;
    energy.E_cruise_Wh   = E_cruise_Wh;
    energy.E_mission_Wh  = E_mission_Wh;
    energy.E_design_Wh   = E_design_Wh;
    energy.E_avail_Wh    = E_avail_Wh;
    energy.BMF_hover     = BMF_hover;
    energy.BMF_cruise    = BMF_cruise;
    energy.BMF_total     = BMF_total;
    energy.feasible      = feasible;
    energy.margin_Wh     = margin_Wh;

    % --- Print summary ---
    fprintf('\n====================================================\n');
    fprintf(' Energy Budget: %s Mode\n', upper(mission.mode));
    fprintf('====================================================\n');
    fprintf(' Prop dia: %.3f m  |  n_rotors: %d\n', ac.D_prop_m, ac.n_rotors);
    fprintf(' Disk area:         %8.5f m^2\n',      A_disk_m2);
    fprintf(' Disk loading:      %8.2f N/m^2\n',    DL_Nm2);
    fprintf(' eta_hover (FM*elec): %.3f\n',          ac.eta_hover);
    fprintf(' Hover power (elec):%8.2f W\n',         P_hover_W);
    fprintf(' ------------------------------------------------\n');
    fprintf(' Hover takeoff:     %8.4f Wh   (%.0f s)\n', E_hover_to_Wh, mission.t_hover_to_s);
    fprintf(' Cruise (RT %.0f m): %7.4f Wh\n',          R_total_m, E_cruise_Wh);
    fprintf(' Hover landing:     %8.4f Wh   (%.0f s)\n', E_hover_ld_Wh, mission.t_hover_ld_s);
    fprintf(' Mission total:     %8.4f Wh\n',            E_mission_Wh);
    fprintf(' Design (x%.1f rsv):%8.4f Wh\n',            mission.reserve_factor, E_design_Wh);
    fprintf(' Battery avail:     %8.4f Wh   (%.0f g @ %.0f Wh/kg)\n', ...
            E_avail_Wh, ac.m_bat_kg*1e3, ac.e_bat_Whkg);
    fprintf(' ------------------------------------------------\n');
    fprintf(' BMF hover:         %.4f\n', BMF_hover);
    fprintf(' BMF cruise:        %.4f\n', BMF_cruise);
    fprintf(' BMF total (w/rsv): %.4f\n', BMF_total);
    if feasible
        fprintf(' STATUS: FEASIBLE  (+%.4f Wh margin)\n', margin_Wh);
    else
        fprintf(' STATUS: *** INFEASIBLE  (short %.4f Wh) ***\n', -margin_Wh);
    end
    fprintf('====================================================\n\n');
end
