function rfxIn = nimbusRFXParams(wingOut, vertOut, massOut, propIn, massOut_unloaded)
% nimbusRFXParams
%
% Purpose:
%   Collect Nimbus aircraft parameters from the main.m pipeline outputs
%   and package them into the rfxIn struct expected by exportNimbusToRFX.
%
% USAGE (call after running main.m):
%   rfxIn = nimbusRFXParams(wingOut, vertOut, massOut, propIn, massOut_unloaded);
%   exportNimbusToRFX(rfxIn, 'exports/');
%
% If massOut_unloaded is omitted, loaded configuration is used for all weights.
%
% INPUTS:
%   wingOut         — output from wingGeometryDesign(wingIn)
%   vertOut         — output from verticalSurfaceDesign(vertIn)
%   massOut         — output from aircraftMassProperties (loaded, with payload)
%   propIn          — propulsion input struct from main.m
%   massOut_unloaded — (optional) mass output without payload

    if nargin < 5
        massOut_unloaded = massOut;
    end

    g = 9.81;

    %% ---- Airframe / Weight ----
    rfxIn.W_total_N       = massOut.weight_N;           % [N] loaded gross weight
    rfxIn.W_wing_N        = 0.40 * massOut.weight_N;    % [N] rough wing structural fraction
    rfxIn.W_centerbody_N  = 0.35 * massOut.weight_N;    % [N] centerbody + motor mount fraction
    % Remaining weight (~25%) allocated to battery via BatteryStoredEnergy component

    rfxIn.cg_x_m  = massOut.cg_m(1);    % [m] CG x-position from wing root LE
    rfxIn.cg_z_m  = massOut.cg_m(3);    % [m] CG z from wing mean surface
    rfxIn.Ixx_kgm2 = massOut.Icg_kgm2(1,1);
    rfxIn.Iyy_kgm2 = massOut.Icg_kgm2(2,2);
    rfxIn.Izz_kgm2 = massOut.Icg_kgm2(3,3);

    %% ---- Wing Geometry ----
    rfxIn.b_m           = wingOut.b_m;
    rfxIn.c_root_m      = wingOut.c_root_m;
    rfxIn.c_tip_m       = wingOut.c_tip_m;
    rfxIn.sweep_LE_deg  = wingOut.sweep_LE_deg;
    rfxIn.dihedral_deg  = 0;               % flying wing, no geometric dihedral
    rfxIn.airfoil_name  = 'NACA 2412';     % closest symmetric/cambered to MH95 in RF9 library

    %% ---- Elevon Control Surfaces ----
    % Pulled from main.m wingIn settings — adjust if you changed them
    rfxIn.cs_eta_start      = 0.600;   % 60% semispan
    rfxIn.cs_eta_end        = 0.950;   % 95% semispan
    rfxIn.cs_chord_frac     = 0.450;   % 45% chord
    rfxIn.cs_deflect_up_deg = 25.0;    % [deg] up limit
    rfxIn.cs_deflect_dn_deg = 25.0;    % [deg] down limit

    %% ---- Winglets ----
    rfxIn.has_winglets    = true;
    if isfield(vertOut, 'b_v_m')
        rfxIn.wl_span_m   = vertOut.b_v_m;
    else
        rfxIn.wl_span_m   = 0.10;   % [m] default if vertOut field missing
    end
    if isfield(vertOut, 'c_root_v_m')
        rfxIn.wl_c_root_m = vertOut.c_root_v_m;
    else
        rfxIn.wl_c_root_m = wingOut.c_tip_m;
    end
    if isfield(vertOut, 'c_tip_v_m')
        rfxIn.wl_c_tip_m  = vertOut.c_tip_v_m;
    else
        rfxIn.wl_c_tip_m  = 0.01;
    end
    if isfield(vertOut, 'sweep_c4_v_deg')
        % approximate LE sweep from c/4 sweep + taper geometry
        AR_wl   = 2 * rfxIn.wl_span_m^2 / ...
                  (rfxIn.wl_span_m * (rfxIn.wl_c_root_m + rfxIn.wl_c_tip_m));
        taper_wl = rfxIn.wl_c_tip_m / rfxIn.wl_c_root_m;
        sweep_c4 = vertOut.sweep_c4_v_deg;
        rfxIn.wl_sweep_LE_deg = atand(tand(sweep_c4) + (1 - taper_wl) / (AR_wl * (1 + taper_wl)));
    else
        rfxIn.wl_sweep_LE_deg = 60.0;  % delta winglet LE sweep from main.m
    end

    %% ---- Propulsion ----
    rfxIn.D_prop_in     = propIn.D_in;        % [in]
    rfxIn.pitch_prop_in = propIn.pitch_in;    % [in]
    rfxIn.is_pusher     = false;              % tractor configuration
    rfxIn.motor_name    = 'ElectriFly RimFire 28-30-1100 Outrunner';  % KV=1100 match

    %% ---- Battery (3S 2200 mAh LiPo) ----
    rfxIn.n_cells_series    = 3;       % 3S = 11.1V nominal
    rfxIn.n_cells_parallel  = 1;       % 1P
    rfxIn.cell_capacity_mah = 2200;    % [mAh]
    rfxIn.batt_mass_kg      = 0.170;   % [kg] typical 3S 2200 mAh = ~170g
    rfxIn.batt_length_m     = 0.110;   % [m] physical pack length
    rfxIn.batt_x_m          = massOut.cg_m(1) - 0.015;  % [m] 15mm fwd of CG

    fprintf('\n--- Nimbus RFX Parameters ---\n');
    fprintf('  Total weight:  %.2f N (%.3f kg)\n', rfxIn.W_total_N, rfxIn.W_total_N/g);
    fprintf('  Wingspan:      %.3f m\n', rfxIn.b_m);
    fprintf('  Root chord:    %.3f m\n', rfxIn.c_root_m);
    fprintf('  Tip chord:     %.3f m\n', rfxIn.c_tip_m);
    fprintf('  LE sweep:      %.1f deg\n', rfxIn.sweep_LE_deg);
    fprintf('  CG (x,z):      (%.3f, %.3f) m from LE\n', rfxIn.cg_x_m, rfxIn.cg_z_m);
    fprintf('  Prop:          %.0f x %.0f in\n', rfxIn.D_prop_in, rfxIn.pitch_prop_in);
    fprintf('  Battery:       %dS %d mAh\n', rfxIn.n_cells_series, rfxIn.cell_capacity_mah);
    fprintf('  Winglet span:  %.3f m\n', rfxIn.wl_span_m);
end
