function wingOut = wingGeometryDesign(wingIn)
% wingGeometryDesign
% Builds straight-taper wing planform geometry from S_ref and user-selected
% design variables. Quarter-chord sweep is the primary sweep input.
%
% REQUIRED INPUTS (wingIn)
%   wingIn.S_ref_m2         [m^2] reference wing area
%   wingIn.taper            [-]   taper ratio = c_tip / c_root
%   wingIn.sweep_c4_deg     [deg] quarter-chord sweep
%
% EITHER
%   wingIn.AR               [-]   aspect ratio
% OR
%   wingIn.useSpecifiedSpan = true
%   wingIn.b_m              [m]   full span
%
% OPTIONAL INPUTS
%   wingIn.symmetric        [-]   default = true
%   wingIn.xLE_root_m       [m]   default = 0
%   wingIn.y_root_m         [m]   default = 0
%   wingIn.z_root_m         [m]   default = 0
%
% OPTIONAL CONTROL-SURFACE INPUTS
%   wingIn.eta_cs_start     [-]   start span fraction on semispan, default = 0.60
%   wingIn.eta_cs_end       [-]   end   span fraction on semispan, default = 0.90
%   wingIn.cs_chord_frac    [-]   control surface chord / local chord, default = 0.25
%
% OUTPUTS
%   Planform geometry, MAC data, sweep conversions, quarter-chord
%   coordinates, and first-pass control-surface geometry.

    %% ---------------- Inputs ----------------
    S_ref        = wingIn.S_ref_m2;
    taper        = wingIn.taper;
    sweep_c4_deg = wingIn.sweep_c4_deg;

    if isfield(wingIn,'symmetric')
        symmetric = wingIn.symmetric;
    else
        symmetric = true;
    end

    if isfield(wingIn,'useSpecifiedSpan')
        useSpecifiedSpan = wingIn.useSpecifiedSpan;
    else
        useSpecifiedSpan = false;
    end

    if isfield(wingIn,'xLE_root_m')
        xLE_root = wingIn.xLE_root_m;
    else
        xLE_root = 0;
    end

    if isfield(wingIn,'y_root_m')
        y_root = wingIn.y_root_m;
    else
        y_root = 0;
    end

    if isfield(wingIn,'z_root_m')
        z_root = wingIn.z_root_m;
    else
        z_root = 0;
    end

    if isfield(wingIn,'eta_cs_start')
        eta_cs_start = wingIn.eta_cs_start;
    else
        eta_cs_start = 0.60;
    end

    if isfield(wingIn,'eta_cs_end')
        eta_cs_end = wingIn.eta_cs_end;
    else
        eta_cs_end = 0.90;
    end

    if isfield(wingIn,'cs_chord_frac')
        cs_chord_frac = wingIn.cs_chord_frac;
    else
        cs_chord_frac = 0.25;
    end

    %% ---------------- Validation ----------------
    if S_ref <= 0
        error('wingGeometryDesign:InvalidInput','S_ref_m2 must be > 0.');
    end

    if taper <= 0
        error('wingGeometryDesign:InvalidInput','taper must be > 0.');
    end

    if eta_cs_start < 0 || eta_cs_start > 1 || eta_cs_end < 0 || eta_cs_end > 1
        error('wingGeometryDesign:InvalidInput', ...
            'eta_cs_start and eta_cs_end must be between 0 and 1.');
    end

    if eta_cs_end <= eta_cs_start
        error('wingGeometryDesign:InvalidInput', ...
            'eta_cs_end must be greater than eta_cs_start.');
    end

    if cs_chord_frac <= 0 || cs_chord_frac >= 1
        error('wingGeometryDesign:InvalidInput', ...
            'cs_chord_frac must be between 0 and 1.');
    end

    %% ---------------- Span and AR ----------------
    if useSpecifiedSpan
        if ~isfield(wingIn,'b_m')
            error('wingGeometryDesign:MissingInput', ...
                'b_m must be provided when useSpecifiedSpan = true.');
        end
        b = wingIn.b_m;
        if b <= 0
            error('wingGeometryDesign:InvalidInput','b_m must be > 0.');
        end
        AR = b^2 / S_ref;
    else
        if ~isfield(wingIn,'AR')
            error('wingGeometryDesign:MissingInput', ...
                'AR must be provided when useSpecifiedSpan = false.');
        end
        AR = wingIn.AR;
        if AR <= 0
            error('wingGeometryDesign:InvalidInput','AR must be > 0.');
        end
        b = sqrt(AR * S_ref);
    end

    if symmetric
        semiSpan = b / 2;
    else
        semiSpan = b;
    end

    %% ---------------- Chords ----------------
    c_root = (2 * S_ref) / (b * (1 + taper));
    c_tip  = taper * c_root;

    % Local chord function c(y), with y measured from centerline
    chordAtY = @(y) c_root - (c_root - c_tip) * (y / semiSpan);

    %% ---------------- Sweep conversions ----------------
    % For a straight-taper wing:
    % tan(Lambda_c/4) = tan(Lambda_LE) - (1- taper)/AR
    %
    % so:
    tan_sweepLE = tand(sweep_c4_deg) + (1 - taper) / AR;
    sweep_LE_rad = atan(tan_sweepLE);
    sweep_LE_deg = rad2deg(sweep_LE_rad);

    % Trailing-edge sweep
    xLE_tip = xLE_root + semiSpan * tan(sweep_LE_rad);
    xTE_root = xLE_root + c_root;
    xTE_tip  = xLE_tip  + c_tip;

    tan_sweepTE = (xTE_tip - xTE_root) / semiSpan;
    sweep_TE_rad = atan(tan_sweepTE);
    sweep_TE_deg = rad2deg(sweep_TE_rad);

    %% ---------------- MAC relations ----------------
    MAC = (2/3) * c_root * ((1 + taper + taper^2) / (1 + taper));
    y_MAC = semiSpan * (1 + 2*taper) / (3 * (1 + taper));
    xLE_MAC = xLE_root + y_MAC * tan(sweep_LE_rad);
    x_c4_MAC = xLE_MAC + 0.25 * MAC;

    %% ---------------- Key coordinates ----------------
    rootLE = [xLE_root, y_root, z_root];
    tipLE  = [xLE_tip,  y_root + semiSpan, z_root];

    rootTE = [xTE_root, y_root, z_root];
    tipTE  = [xTE_tip,  y_root + semiSpan, z_root];

    rootC4 = [xLE_root + 0.25*c_root, y_root, z_root];
    tipC4  = [xLE_tip  + 0.25*c_tip,  y_root + semiSpan, z_root];
    MAC_LE = [xLE_MAC, y_root + y_MAC, z_root];
    MAC_C4 = [x_c4_MAC, y_root + y_MAC, z_root];

    %% ---------------- Control surface geometry ----------------
    y_cs_start = eta_cs_start * semiSpan;
    y_cs_end   = eta_cs_end   * semiSpan;

    c_cs_start = chordAtY(y_cs_start);
    c_cs_end   = chordAtY(y_cs_end);

    xLE_cs_start = xLE_root + y_cs_start * tan(sweep_LE_rad);
    xLE_cs_end   = xLE_root + y_cs_end   * tan(sweep_LE_rad);

    xTE_cs_start = xLE_cs_start + c_cs_start;
    xTE_cs_end   = xLE_cs_end   + c_cs_end;

    cs_depth_start = cs_chord_frac * c_cs_start;
    cs_depth_end   = cs_chord_frac * c_cs_end;

    xHinge_start = xTE_cs_start - cs_depth_start;
    xHinge_end   = xTE_cs_end   - cs_depth_end;

    cs.span_m = y_cs_end - y_cs_start;
    cs.area_one_side_m2 = 0.5 * (cs_depth_start + cs_depth_end) * cs.span_m;

    cs.LE_inboard = [xLE_cs_start, y_root + y_cs_start, z_root];
    cs.LE_outboard = [xLE_cs_end,  y_root + y_cs_end,   z_root];
    cs.TE_inboard = [xTE_cs_start, y_root + y_cs_start, z_root];
    cs.TE_outboard = [xTE_cs_end,  y_root + y_cs_end,   z_root];
    cs.hinge_inboard = [xHinge_start, y_root + y_cs_start, z_root];
    cs.hinge_outboard = [xHinge_end,  y_root + y_cs_end,   z_root];

    %% ---------------- Output struct ----------------
    wingOut = struct();

    wingOut.S_ref_m2       = S_ref;
    wingOut.AR             = AR;
    wingOut.b_m            = b;
    wingOut.semiSpan_m     = semiSpan;

    wingOut.taper          = taper;

    wingOut.c_root_m       = c_root;
    wingOut.c_tip_m        = c_tip;
    wingOut.MAC_m          = MAC;

    wingOut.sweep_c4_deg   = sweep_c4_deg;
    wingOut.sweep_LE_deg   = sweep_LE_deg;
    wingOut.sweep_TE_deg   = sweep_TE_deg;

    wingOut.sweep_c4_rad   = deg2rad(sweep_c4_deg);
    wingOut.sweep_LE_rad   = sweep_LE_rad;
    wingOut.sweep_TE_rad   = sweep_TE_rad;

    wingOut.y_MAC_m        = y_MAC;
    wingOut.xLE_root_m     = xLE_root;
    wingOut.xLE_tip_m      = xLE_tip;
    wingOut.xLE_MAC_m      = xLE_MAC;
    wingOut.x_c4_MAC_m     = x_c4_MAC;

    wingOut.xTE_root_m     = xTE_root;
    wingOut.xTE_tip_m      = xTE_tip;

    wingOut.rootLE         = rootLE;
    wingOut.tipLE          = tipLE;
    wingOut.rootTE         = rootTE;
    wingOut.tipTE          = tipTE;
    wingOut.rootC4         = rootC4;
    wingOut.tipC4          = tipC4;
    wingOut.MAC_LE         = MAC_LE;
    wingOut.MAC_C4         = MAC_C4;

    wingOut.controlSurface = cs;

    %% ---------------- Print ----------------
    fprintf('\n================ Wing Geometry Design =================\n');
    fprintf('S_ref                 = %.4f m^2\n', S_ref);
    fprintf('AR                    = %.4f\n', AR);
    fprintf('Span b                = %.4f m\n', b);
    fprintf('Semi-span             = %.4f m\n', semiSpan);
    fprintf('Taper ratio           = %.4f\n', taper);
    fprintf('Sweep c/4             = %.3f deg\n', sweep_c4_deg);
    fprintf('Sweep LE              = %.3f deg\n', sweep_LE_deg);
    fprintf('Sweep TE              = %.3f deg\n', sweep_TE_deg);
    fprintf('Root chord            = %.4f m\n', c_root);
    fprintf('Tip chord             = %.4f m\n', c_tip);
    fprintf('MAC                   = %.4f m\n', MAC);
    fprintf('y_MAC                 = %.4f m\n', y_MAC);
    fprintf('xLE_MAC               = %.4f m\n', xLE_MAC);
    fprintf('x_c/4_MAC             = %.4f m\n', x_c4_MAC);
    fprintf('Control surface span  = %.4f m\n', cs.span_m);
    fprintf('Control surface area  = %.4f m^2 (one side)\n', cs.area_one_side_m2);
    fprintf('=======================================================\n\n');
end