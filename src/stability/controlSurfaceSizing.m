function csOut = controlSurfaceSizing(csIn)
% controlSurfaceSizing
%
% Purpose:
%   Elevon and rudder maneuvering authority check from AVL control derivatives.
%   Computes trim deflection, max load factor, min turn radius, and roll rate.
%
% Inputs (csIn struct):
%   .CLde        [/deg]  elevator CL effectiveness  (CLd01 from AVL)
%   .Cmde        [/deg]  elevator Cm effectiveness  (Cmd01 from AVL)
%   .Clda        [/deg]  aileron  Cl effectiveness  (Cld02 from AVL)
%   .Cnda        [/deg]  aileron adverse yaw        (Cnd02 from AVL)
%   .Cndr        [/deg]  rudder yaw effectiveness   (Cnd03 from AVL)
%   .Cnb         [/rad]  directional stability      (Cnb   from AVL)
%   .Cm0_trim    [-]     pitching moment at δe=0    (Cmtot from AVL)
%   .CL_trim     [-]     cruise lift coefficient
%   .CLmax       [-]     airfoil stall CL (upper bound on CL_turn)
%   .V_mps       [m/s]   cruise speed
%   .rho_kgm3    [kg/m³] air density
%   .S_ref_m2    [m²]    wing reference area
%   .b_m         [m]     wingspan
%   .mass_kg     [kg]    aircraft mass
%   .Clp         [/rad]  roll damping (from AVL, negative)
%   .delta_e_max [deg]   max elevon deflection (default 20)
%   .delta_a_max [deg]   max aileron deflection (default 20)
%   .delta_r_max [deg]   max rudder deflection (default 25)
%   .showPlots   bool    generate turn-radius plot
%   .cs_chord_frac   [-]   elevon chord / total chord      (optional, for hinge moment)
%   .eta_cs_start    [-]   elevon inboard span fraction    (optional)
%   .eta_cs_end      [-]   elevon outboard span fraction   (optional)
%   .c_root_m        [m]   wing root chord                 (optional)
%   .c_tip_m         [m]   wing tip chord                  (optional)
%   .semiSpan_m      [m]   wing semispan                   (optional)
%   .rudder_c_avg_m  [m]   rudder average chord            (optional)
%   .rudder_height_m [m]   rudder span / height            (optional)
%
% Outputs (csOut struct):
%   .beta_max_deg      [deg]   max trimmable sideslip at full rudder
%   .delta_e_trim_deg  [deg]   trim elevon deflection
%   .n_max             [-]     max load factor (pitch authority)
%   .phi_max_deg       [deg]   max bank angle
%   .R_min_m           [m]     minimum turn radius
%   .phi_deg           [deg]   bank angle array (0 → phi_max)
%   .R_m               [m]     turn radius array
%   .delta_e_deg       [deg]   required elevon at each bank angle
%   .p_ss_dps          [°/s]   steady-state roll rate at delta_a_max
%   .HM_elevon_Nm      [N·m]   elevon hinge moment, one side, at delta_e_max
%   .HM_rudder_Nm      [N·m]   rudder hinge moment, one fin,  at delta_r_max
%   .T_sg90_Nm         [N·m]   SG90 stall torque reference (0.177 N·m)
%   .elevon_servo_ok   bool    true if SG90 can drive elevon
%   .rudder_servo_ok   bool    true if SG90 can drive rudder

    if ~isfield(csIn,'delta_e_max'), csIn.delta_e_max = 20;    end
    if ~isfield(csIn,'delta_a_max'), csIn.delta_a_max = 20;    end
    if ~isfield(csIn,'delta_r_max'), csIn.delta_r_max = 25;    end
    if ~isfield(csIn,'showPlots'),   csIn.showPlots   = false; end

    CLde    = csIn.CLde;
    Cmde    = csIn.Cmde;
    Clda    = abs(csIn.Clda);   % magnitude — sign depends on which wing AVL picked
    Cnda    = csIn.Cnda;
    Cndr    = csIn.Cndr;
    if isfield(csIn,'Cnb'), Cnb = csIn.Cnb; else, Cnb = NaN; end
    Cm0     = csIn.Cm0_trim;
    CL_c    = csIn.CL_trim;
    CLmax   = csIn.CLmax;
    V       = csIn.V_mps;
    rho     = csIn.rho_kgm3;
    S       = csIn.S_ref_m2;
    b       = csIn.b_m;
    Clp     = csIn.Clp;         % /rad, negative (roll damping)
    de_max  = csIn.delta_e_max;
    da_max  = csIn.delta_a_max;
    dr_max  = csIn.delta_r_max;
    g       = 9.81;

    hasHingeGeom = isfield(csIn,'cs_chord_frac') && isfield(csIn,'c_root_m') && ...
                   isfield(csIn,'c_tip_m')        && isfield(csIn,'semiSpan_m') && ...
                   isfield(csIn,'eta_cs_start')   && isfield(csIn,'eta_cs_end') && ...
                   isfield(csIn,'rudder_c_avg_m') && isfield(csIn,'rudder_height_m');

    % ---- trim elevon deflection ----
    % Cm0 + Cmde * de_trim = 0
    de_trim = -Cm0 / Cmde;

    % ---- pitch authority → max load factor ----
    % CL limited by either elevon saturation or airfoil stall
    dCL_elevon = CLde * (de_max - de_trim);   % CL available from full up-pull
    CL_max_elev  = CL_c + dCL_elevon;
    CL_max_stall = CLmax;
    CL_max       = min(CL_max_elev, CL_max_stall);
    n_max        = CL_max / CL_c;

    limited_by = 'elevon';
    if CL_max_stall < CL_max_elev
        limited_by = 'stall';
    end

    % ---- turn performance curve ----
    phi_max_deg = acosd(1 / n_max);
    phi_arr     = linspace(0, phi_max_deg * 0.999, 200);
    n_arr       = 1 ./ cosd(phi_arr);
    CL_arr      = CL_c * n_arr;
    de_arr      = de_trim + (CL_arr - CL_c) / CLde;
    R_arr       = V^2 ./ (g * tand(phi_arr));
    R_arr(phi_arr < 0.5) = Inf;

    R_min_m = V^2 / (g * tand(phi_max_deg));

    % ---- roll authority ----
    % Steady-state roll rate: Clda*da + Clp*(p*b/2V) = 0
    % => p_ss = -2V * Clda * da / (Clp * b)   [rad/s]
    p_ss_rads = 2 * V * Clda * da_max / (abs(Clp) * b);
    p_ss_dps  = p_ss_rads * 180 / pi;

    % ---- rudder yaw authority ----
    % Yaw moment coefficient at max rudder:  Cn_rud = Cndr * dr_max
    Cn_rud = abs(Cndr) * dr_max;
    qbar   = 0.5 * rho * V^2;
    N_rud  = Cn_rud * qbar * S * b;   % [N·m] yaw moment at max rudder

    % Steady sideslip balance:  Cnb*beta + Cndr*dr = 0  →  beta_max = Cndr*dr / Cnb
    % Cndr [/deg], dr_max [deg], Cnb [/rad] → convert Cndr to /rad for balance
    if ~isnan(Cnb) && Cnb > 0
        Cndr_rad  = abs(Cndr) * (180/pi);   % [/rad]
        beta_max_deg = Cndr_rad * dr_max / Cnb;
    else
        beta_max_deg = NaN;
    end

    % Adverse yaw from aileron that rudder must cancel
    dr_adverse_deg = abs(Cnda * da_max) / max(abs(Cndr), 1e-9);

    % ---- hinge moments vs SG90 servo (1.8 kg*cm = 0.177 N*m at 4.8V) ----
    T_sg90_Nm       = 0.177;
    HM_elevon_Nm    = NaN;
    HM_rudder_Nm    = NaN;
    elevon_servo_ok = NaN;
    rudder_servo_ok = NaN;

    if hasHingeGeom
        % Plain trailing-edge flap: |Ch_delta| ~ 0.50 /rad (first-pass estimate;
        % thin-airfoil theory gives ~0.45-0.75 depending on cf and airfoil shape)
        Ch_delta = 0.50;
        de_rad   = de_max * pi/180;
        dr_rad   = dr_max * pi/180;

        % elevon chord at inboard and outboard eta, then average
        c_cs_s   = csIn.cs_chord_frac * (csIn.c_root_m + ...
                   (csIn.c_tip_m - csIn.c_root_m) * csIn.eta_cs_start);
        c_cs_e   = csIn.cs_chord_frac * (csIn.c_root_m + ...
                   (csIn.c_tip_m - csIn.c_root_m) * csIn.eta_cs_end);
        c_cs_avg = 0.5 * (c_cs_s + c_cs_e);
        b_cs     = (csIn.eta_cs_end - csIn.eta_cs_start) * csIn.semiSpan_m;
        S_cs     = b_cs * c_cs_avg;   % planform area, one side
        HM_elevon_Nm    = Ch_delta * de_rad * qbar * S_cs * c_cs_avg;
        elevon_servo_ok = HM_elevon_Nm <= T_sg90_Nm;

        % rudder (one fin)
        c_rud_avg = csIn.rudder_c_avg_m;
        b_rud     = csIn.rudder_height_m;
        S_rud     = b_rud * c_rud_avg;
        HM_rudder_Nm    = Ch_delta * dr_rad * qbar * S_rud * c_rud_avg;
        rudder_servo_ok = HM_rudder_Nm <= T_sg90_Nm;
    end

    % ---- print ----
    fprintf('\n============= CONTROL SURFACE SIZING ================\n');
    fprintf('--- Elevon (pitch / roll) ---\n');
    fprintf('  Trim deflection          = %+.2f deg  (+ = TE down)\n', de_trim);
    fprintf('  Max deflection           = +/-%.0f deg\n', de_max);
    fprintf('  CL increment at de_max   = %.4f\n', dCL_elevon);
    fprintf('  Max load factor          = %.2f g  (limited by %s)\n', n_max, limited_by);
    fprintf('  Max bank angle           = %.1f deg\n', phi_max_deg);
    fprintf('  Minimum turn radius      = %.1f m  (at V=%.0f m/s)\n', R_min_m, V);
    fprintf('--- Aileron ---\n');
    fprintf('  Steady-state roll rate   = %.1f deg/s  (at da=%.0f deg)\n', p_ss_dps, da_max);
    fprintf('  Adverse yaw (Cnda*da)    = %.5f  (small = good)\n', Cnda * da_max);
    fprintf('--- Rudder ---\n');
    fprintf('  Cndr (per deg)           = %.6f\n', Cndr);
    fprintf('  Max yaw moment           = %.4f N*m  (at dr=%.0f deg)\n', N_rud, dr_max);
    if ~isnan(beta_max_deg)
        fprintf('  Max trimmable sideslip   = %.1f deg  (Cndr*dr_max / Cnb)\n', beta_max_deg);
        fprintf('  Adverse yaw correction   = %.1f deg rudder  (to cancel da=%.0f deg)\n', ...
                dr_adverse_deg, da_max);
    end
    if hasHingeGeom
        fprintf('--- Hinge Moments vs SG90 (%.3f N*m at 4.8V) ---\n', T_sg90_Nm);
        fprintf('  Elevon HM (one side)     = %.4f N*m  (de_max=%.0f deg)\n', HM_elevon_Nm, de_max);
        if elevon_servo_ok
            fprintf('  Elevon servo             = OK  (%.0f%% of SG90 capacity)\n', 100*HM_elevon_Nm/T_sg90_Nm);
        else
            fprintf('  *** ELEVON SERVO UNDERSIZED (%.0f%% over limit) ***\n', ...
                100*(HM_elevon_Nm/T_sg90_Nm - 1));
        end
        fprintf('  Rudder HM (one fin)      = %.4f N*m  (dr_max=%.0f deg)\n', HM_rudder_Nm, dr_max);
        if rudder_servo_ok
            fprintf('  Rudder servo             = OK  (%.0f%% of SG90 capacity)\n', 100*HM_rudder_Nm/T_sg90_Nm);
        else
            fprintf('  *** RUDDER SERVO UNDERSIZED (%.0f%% over limit) ***\n', ...
                100*(HM_rudder_Nm/T_sg90_Nm - 1));
        end
    end
    fprintf('======================================================\n\n');

    % ---- turn radius plot ----
    if csIn.showPlots
        figure('Name','Turn Performance','NumberTitle','off');
        idx = isfinite(R_arr);
        plot(phi_arr(idx), R_arr(idx), 'b-', 'LineWidth', 2); hold on;
        plot(phi_max_deg, R_min_m, 'ro', 'MarkerSize', 10, 'MarkerFaceColor','r');
        xline(phi_max_deg, 'r--', sprintf('  Max bank = %.1f°  (R_{min}=%.0f m)', ...
            phi_max_deg, R_min_m), 'LabelVerticalAlignment','bottom');
        xlabel('Bank angle [deg]');
        ylabel('Turn radius [m]');
        title('Turn Radius vs Bank Angle');
        ylim([0, min(400, max(R_arr(idx)))]);
        grid on; box on;
    end

    csOut.delta_e_trim_deg = de_trim;
    csOut.n_max            = n_max;
    csOut.phi_max_deg      = phi_max_deg;
    csOut.R_min_m          = R_min_m;
    csOut.phi_deg          = phi_arr;
    csOut.R_m              = R_arr;
    csOut.delta_e_deg      = de_arr;
    csOut.p_ss_dps         = p_ss_dps;
    csOut.N_rud_Nm         = N_rud;
    csOut.beta_max_deg     = beta_max_deg;
    csOut.dr_adverse_deg   = dr_adverse_deg;
    csOut.limited_by       = limited_by;
    csOut.HM_elevon_Nm     = HM_elevon_Nm;
    csOut.HM_rudder_Nm     = HM_rudder_Nm;
    csOut.T_sg90_Nm        = T_sg90_Nm;
    csOut.elevon_servo_ok  = elevon_servo_ok;
    csOut.rudder_servo_ok  = rudder_servo_ok;
end
