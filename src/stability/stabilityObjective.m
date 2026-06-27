function [J, info] = stabilityObjective(x, ctx)
% stabilityObjective
%
% Purpose:
%   Scalar objective for CMA-ES dynamic stability optimization.
%   Minimizing J maximizes static margin while penalizing violations of
%   MIL-STD handling quality requirements.
%
% Inputs:
%   x    [7x1] parameter vector:
%          x(1) wing quarter-chord sweep  [deg]
%          x(2) wing taper ratio          [-]
%          x(3) tip geometric twist       [deg]  (root fixed at 0; negative = washout)
%          x(4) fin aspect ratio          [-]
%          x(5) fin taper ratio           [-]
%          x(6) fin quarter-chord sweep   [deg]
%          x(7) wing LE root x position   [m]   (slides wing fore/aft on fuselage)
%   ctx  context struct (same fields as sweepIn in dynamicStabilitySweep)
%
% Outputs:
%   J    scalar objective (minimize)
%   info struct with SM_pct, Xcg_m, mode metrics, AVL success flag

    J    = 1e6;   % default: large penalty if pipeline fails
    info = struct('SM_pct', NaN, 'Xcg_m', NaN, 'failed', true, ...
                  'sp_zeta', NaN, 'ph_zeta', NaN);

    try
        wingSweep_x  = x(1);
        taper_x      = x(2);
        twistTip_x   = x(3);
        AR_v_x       = x(4);
        taperV_x     = x(5);
        sweepV_x     = x(6);
        xLE_root_x   = x(7);

        wingIn  = ctx.wingIn;
        vertIn  = ctx.vertIn;
        dynIn   = ctx.dynIn;

        % ---- wing ----
        wingIn_x              = wingIn;
        wingIn_x.sweep_c4_deg = wingSweep_x;
        wingIn_x.taper        = taper_x;
        wingIn_x.xLE_root_m   = xLE_root_x;
        wingOut_x             = wingGeometryDesign(wingIn_x);

        % ---- twist (direct tip input; root fixed at 0) ----

        % ---- fins ----
        vertIn_x                = vertIn;
        vertIn_x.AR_v           = AR_v_x;
        vertIn_x.taper_v        = taperV_x;
        vertIn_x.sweep_c4_v_deg = sweepV_x;
        vertIn_x.xLE_root_v_m   = wingOut_x.xLE_tip_m;
        vertIn_x.y_root_v_m     = wingIn_x.y_root_m + wingOut_x.semiSpan_m;
        vertOut_x               = verticalSurfaceDesign(vertIn_x);

        % ---- physical fin height constraint ----
        % Fins taller than 50% of semispan are not physically buildable.
        % DR Level 1 requires AR_v > 4 which exceeds this limit — the
        % optimizer will find the best DR achievable within the constraint.
        b_v_max_m = 0.50 * wingOut_x.semiSpan_m;
        if vertOut_x.b_v_m > b_v_max_m
            J = 1e6;
            return;
        end

        % ---- rebuild mass ----
        x_c4_MAC_x = wingOut_x.x_c4_MAC_m;
        eta_s      = ctx.eta_servo;
        comp_x     = ctx.compFixed;

        comp_x(end+1) = makePointMass('S5 Servo cargo bay', 0.009, [0.61980000, 0.000, 0.000]);

        y_servo_x = wingIn_x.y_root_m + eta_s * wingOut_x.semiSpan_m;
        x_hinge_x = wingOut_x.xLE_root_m + ...
            (wingOut_x.xLE_tip_m - wingOut_x.xLE_root_m)*eta_s + ...
            0.75*(wingOut_x.c_root_m + (wingOut_x.c_tip_m - wingOut_x.c_root_m)*eta_s);
        comp_x(end+1) = makePointMass('S2 Servo LHS wing', 0.009, [x_hinge_x, -y_servo_x, wingIn_x.z_root_m]);
        comp_x(end+1) = makePointMass('S3 Servo RHS wing', 0.009, [x_hinge_x,  y_servo_x, wingIn_x.z_root_m]);
        comp_x(end+1) = makePointMass('S1 Servo back wing', 0.009, [x_c4_MAC_x + 0.020, 0.000, wingIn_x.z_root_m]);
        comp_x(end+1) = makePointMass('S4 Servo vert stab', 0.009, ...
            [vertOut_x.xLE_root_v_m + 0.70*vertOut_x.c_root_v_m, ...
             vertOut_x.y_root_v_m, ...
             vertOut_x.z_root_v_m + 0.20*vertOut_x.b_v_m]);

        y_ws_x = wingIn_x.y_root_m + 0.42*wingOut_x.semiSpan_m;
        comp_x(end+1) = makePointMass('Wing structure L', 0.5*ctx.m_wing_struct_kg, [x_c4_MAC_x, -y_ws_x, wingIn_x.z_root_m]);
        comp_x(end+1) = makePointMass('Wing structure R', 0.5*ctx.m_wing_struct_kg, [x_c4_MAC_x,  y_ws_x, wingIn_x.z_root_m]);

        if vertOut_x.isTwin
            m_fin_x = 0.5 * ctx.m_vert_struct_kg;
        else
            m_fin_x = ctx.m_vert_struct_kg;
        end
        x_fin_x = vertOut_x.xLE_root_v_m + 0.40*vertOut_x.c_root_v_m;
        y_fin_x = vertOut_x.y_root_v_m;
        z_fin_x = vertOut_x.z_root_v_m + 0.30*vertOut_x.b_v_m;
        comp_x(end+1) = makePointMass('Vertical structure R', m_fin_x, [ x_fin_x,  y_fin_x, z_fin_x]);
        if vertOut_x.isTwin
            comp_x(end+1) = makePointMass('Vertical structure L', m_fin_x, [x_fin_x, -y_fin_x, z_fin_x]);
        end

        massIn_x.cadBodies   = ctx.cadMass;
        massIn_x.pointMasses = comp_x;
        massOut_x            = aircraftMassProperties(massIn_x);

        % ---- AVL ----
        dynIn_x             = dynIn;
        dynIn_x.verbose     = false;
        dynIn_x.mass_kg     = massOut_x.mass_kg;
        dynIn_x.Icg_kgm2    = massOut_x.Icg_kgm2;
        dynIn_x.cg_m        = massOut_x.cg_m;
        dynIn_x.S_ref_m2    = wingOut_x.S_ref_m2;
        dynIn_x.MAC_m       = wingOut_x.MAC_m;
        dynIn_x.b_m         = wingOut_x.b_m;
        dynIn_x.xLE_root_m  = wingOut_x.xLE_root_m;
        dynIn_x.xLE_tip_m   = wingOut_x.xLE_tip_m;
        dynIn_x.semiSpan_m  = wingOut_x.semiSpan_m;
        dynIn_x.c_root_m    = wingOut_x.c_root_m;
        dynIn_x.c_tip_m     = wingOut_x.c_tip_m;
        dynIn_x.twist_root_deg = 0;
        dynIn_x.twist_tip_deg  = twistTip_x;
        dynIn_x.xLE_root_v_m   = vertOut_x.xLE_root_v_m;
        dynIn_x.y_root_v_m     = vertOut_x.y_root_v_m;
        dynIn_x.z_root_v_m     = vertOut_x.z_root_v_m;
        dynIn_x.xLE_top_v_m    = vertOut_x.xLE_top_v_m;
        dynIn_x.y_top_v_m      = vertOut_x.y_top_v_m;
        dynIn_x.z_top_v_m      = vertOut_x.z_top_v_m;
        dynIn_x.xLE_bottom_v_m = vertOut_x.xLE_bottom_v_m;
        dynIn_x.y_bottom_v_m   = vertOut_x.y_bottom_v_m;
        dynIn_x.z_bottom_v_m   = vertOut_x.z_bottom_v_m;
        dynIn_x.c_root_v_m     = vertOut_x.c_root_v_m;
        dynIn_x.c_tip_v_m      = vertOut_x.c_tip_v_m;

        dynOut_x = dynamicStabilityAVL(dynIn_x);

        SM   = dynOut_x.SM_pct;
        sp   = dynOut_x.longModes.shortPeriod.metrics;
        ph   = dynOut_x.longModes.phugoid.metrics;
        dr   = dynOut_x.latModes.dutchRoll.metrics;

        % ---- objective: minimize J ----
        %
        % PRIMARY GOAL: flying wing that is longitudinally stable and
        % controllable. Priority order:
        %   1. SM > 0          — must be statically stable (hard constraint)
        %   2. SP oscillatory  — pilot must be able to control pitch axis
        %   3. PH stable       — phugoid just needs to be non-divergent
        %   4. DR Level 1/2    — lateral quality, soft constraint
        %
        % Base: reward SM in the 5–20% band. Cap at 20% so the optimizer
        % does not chase unrealistically high margins at the expense of mode
        % quality. Below 5% is still rewarded (better than nothing) but the
        % cap prevents the 79%-SM outliers that plagued earlier runs.
        J = -min(SM, 20);

        % Priority 1 — longitudinal stability (disqualifying if violated)
        if SM <= 0
            J = J + 1000 * (1 + abs(SM)/5);
        end

        % Priority 2 — short period must be oscillatory and controllable
        if sp.isComplex
            if sp.zeta >= 0.35 && sp.zeta <= 1.30
                % Level 1 — no penalty
            elseif sp.zeta >= 0.25
                J = J + 100;   % Level 2, still flyable
            elseif sp.zeta > 0
                J = J + 300;   % stable but sluggish
            else
                J = J + 600;   % unstable SP — aircraft diverges in pitch
            end
        else
            % SP split to real roots: overdamped, no oscillation
            % Aircraft is stable but pitch response is sluggish and
            % unclassifiable under MIL-STD — penalise heavily
            J = J + 400;
        end

        % Priority 3 — phugoid just needs to be stable (Level 2 acceptable)
        % Flying wings inherently have low phugoid damping — do not penalise
        % Level 2, only penalise divergent phugoid.
        if ph.isComplex
            if ph.zeta >= 0.04
                % Level 1 — no penalty
            elseif ph.zeta > 0
                J = J + 30;    % Level 2: slow oscillation, easily managed
            else
                J = J + 200;   % divergent phugoid — penalise
            end
        else
            J = J + 80;
        end

        % Priority 4 — Dutch roll (lateral-directional, soft constraint)
        if dr.isComplex
            if dr.zeta >= 0.19 && (dr.zeta*dr.wn) >= 0.35 && dr.wn >= 1.0
                % Level 1 — no penalty
            elseif dr.zeta >= 0.08 && (dr.zeta*dr.wn) >= 0.15 && dr.wn >= 0.4
                J = J + 80;    % Level 2 — acceptable for RC aircraft
            elseif dr.zeta > 0
                J = J + 200;
            else
                J = J + 400;   % unstable Dutch roll
            end
        else
            J = J + 300;
        end

        info.SM_pct   = SM;
        info.Xcg_m    = massOut_x.cg_m(1);
        info.sp_zeta  = sp.zeta;
        info.ph_zeta  = ph.zeta;
        info.dr_zeta  = dr.zeta;
        info.failed   = false;

    catch
        % pipeline failure — return large J, keep info.failed = true
    end
end
