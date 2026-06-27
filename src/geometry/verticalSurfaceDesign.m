function vertOut = verticalSurfaceDesign(vertIn)
% verticalSurfaceDesign
%
% First-pass trapezoidal vertical stabilizer / winglet sizing function.
%
% Area convention used in THIS VERSION:
%   S_v_total_m2 = total vertical area for the full aircraft
%                  (sum of both fins if twin vertical surfaces are used)
%
%   S_v_m2       = single-fin area used to generate the geometry returned
%                  by this function
%
% Sizing modes:
%   (1) manualArea:
%           vertIn.S_v_m2 is interpreted as TOTAL vertical area
%
%   (2) tailVolumeCoeff:
%           c_v = (L_v * S_v_total) / (b_w * S_w)
%
% Twin-fin behavior:
%   if vertIn.isTwin = true, this function splits total area equally
%   between the two fins and returns geometry for ONE fin.
%
% Also computes a first-pass rudder geometry on the single vertical surface.

    arguments
        vertIn struct
    end

    % ==============================================================
    % Required common fields
    % ==============================================================
    reqCommon = {'AR_v','taper_v','sweep_c4_v_deg', ...
                 'cant_deg','toe_deg','topFrac', ...
                 'xLE_root_v_m','y_root_v_m','z_root_v_m','airfoilName'};

    for k = 1:numel(reqCommon)
        if ~isfield(vertIn, reqCommon{k})
            error('verticalSurfaceDesign:MissingField', ...
                'Missing required input field: %s', reqCommon{k});
        end
    end

    % ==============================================================
    % Vertical area sizing mode
    % ==============================================================
    if ~isfield(vertIn,'sizeMode')
        vertIn.sizeMode = 'manualArea';
    end

    validModes = {'manualArea','tailVolumeCoeff'};
    if ~any(strcmpi(vertIn.sizeMode, validModes))
        error('verticalSurfaceDesign:BadSizeMode', ...
            'sizeMode must be ''manualArea'' or ''tailVolumeCoeff''.');
    end

    % ==============================================================
    % Twin / single fin option
    % ==============================================================
    if ~isfield(vertIn,'isTwin')
        vertIn.isTwin = true;   % default for your wingtip twin-fin setup
    end

    % ==============================================================
    % Inputs
    % ==============================================================
    AR_v            = vertIn.AR_v;
    taper_v         = vertIn.taper_v;
    sweep_c4_v_deg  = vertIn.sweep_c4_v_deg;
    cant_deg        = vertIn.cant_deg;
    toe_deg         = vertIn.toe_deg;
    topFrac         = vertIn.topFrac;
    isTwin          = vertIn.isTwin;

    x_mount_m       = vertIn.xLE_root_v_m;
    y_mount_m       = vertIn.y_root_v_m;
    z_mount_m       = vertIn.z_root_v_m;
    airfoilName     = vertIn.airfoilName;

    % ==============================================================
    % Sanity checks
    % ==============================================================
    if AR_v <= 0
        error('verticalSurfaceDesign:BadAR', 'AR_v must be positive.');
    end
    if taper_v <= 0
        error('verticalSurfaceDesign:BadTaper', 'taper_v must be positive.');
    end
    if topFrac <= 0 || topFrac >= 1
        error('verticalSurfaceDesign:BadTopFrac', ...
            'topFrac must be between 0 and 1.');
    end

    % ==============================================================
    % Compute TOTAL vertical area first
    % ==============================================================
    switch lower(vertIn.sizeMode)

        case 'manualarea'
            if ~isfield(vertIn,'S_v_m2')
                error('verticalSurfaceDesign:MissingField', ...
                    'manualArea mode requires S_v_m2.');
            end

            S_v_total_m2 = vertIn.S_v_m2;

            if S_v_total_m2 <= 0
                error('verticalSurfaceDesign:BadArea', ...
                    'S_v_m2 must be positive.');
            end

            c_v = NaN;
            L_v_m = NaN;
            x_c4_wing_ref_m = NaN;
            x_c4_vert_ref_m = NaN;

        case 'tailvolumecoeff'
            reqVol = {'S_ref_m2','b_w_m','c_v','x_c4_wing_ref_m'};
            for k = 1:numel(reqVol)
                if ~isfield(vertIn, reqVol{k})
                    error('verticalSurfaceDesign:MissingField', ...
                        'tailVolumeCoeff mode requires %s.', reqVol{k});
                end
            end

            S_ref_m2        = vertIn.S_ref_m2;
            b_w_m           = vertIn.b_w_m;
            c_v             = vertIn.c_v;
            x_c4_wing_ref_m = vertIn.x_c4_wing_ref_m;

            % First-pass solve for TOTAL vertical area
            S_v_total_m2 = 0.08 * S_ref_m2;  % initial guess

            for iter = 1:20
                if isTwin
                    S_v_single_guess = S_v_total_m2 / 2;
                else
                    S_v_single_guess = S_v_total_m2;
                end

                b_v_guess = sqrt(AR_v * S_v_single_guess);
                c_root_guess = (2 * S_v_single_guess) / (b_v_guess * (1 + taper_v));

                x_c4_vert_ref_m = x_mount_m + 0.25 * c_root_guess;
                L_v_m = x_c4_vert_ref_m - x_c4_wing_ref_m;

                if L_v_m <= 0
                    error('verticalSurfaceDesign:BadMomentArm', ...
                        ['Computed L_v <= 0. Check vertical mounting x-location ', ...
                         'and wing reference quarter-chord location.']);
                end

                S_new_total = (c_v * b_w_m * S_ref_m2) / L_v_m;

                if abs(S_new_total - S_v_total_m2) < 1e-8
                    S_v_total_m2 = S_new_total;
                    break;
                end
                S_v_total_m2 = S_new_total;
            end

            % Recompute final reference x-location using SINGLE-fin geometry
            if isTwin
                S_v_single_guess = S_v_total_m2 / 2;
            else
                S_v_single_guess = S_v_total_m2;
            end

            b_v_guess = sqrt(AR_v * S_v_single_guess);
            c_root_guess = (2 * S_v_single_guess) / (b_v_guess * (1 + taper_v));
            x_c4_vert_ref_m = x_mount_m + 0.25 * c_root_guess;
            L_v_m = x_c4_vert_ref_m - x_c4_wing_ref_m;
    end

    % ==============================================================
    % Convert total area to SINGLE-fin area for geometry
    % ==============================================================
    if isTwin
        S_v_m2 = S_v_total_m2 / 2;
    else
        S_v_m2 = S_v_total_m2;
    end

    % ==============================================================
    % Basic trapezoidal planform for ONE fin
    % ==============================================================
    b_v_m = sqrt(AR_v * S_v_m2);

    c_root_v_m = (2 * S_v_m2) / (b_v_m * (1 + taper_v));
    c_tip_v_m  = taper_v * c_root_v_m;

    MAC_v_m = (2/3) * c_root_v_m * ((1 + taper_v + taper_v^2) / (1 + taper_v));

    eta_MAC = (1 + 2*taper_v) / (3*(1 + taper_v));
    s_MAC_m = eta_MAC * b_v_m;

    % ==============================================================
    % Split above / below mount
    % ==============================================================
    z_top_m    = topFrac * b_v_m;
    z_bottom_m = (1 - topFrac) * b_v_m;

    % ==============================================================
    % Orientation
    % ==============================================================
    sweep_c4_v_rad = deg2rad(sweep_c4_v_deg);
    cant_rad       = deg2rad(cant_deg);
    toe_rad        = deg2rad(toe_deg); %#ok<NASGU>

    dy_top =  z_top_m * sin(cant_rad);
    dz_top =  z_top_m * cos(cant_rad);

    dy_bot = -z_bottom_m * sin(cant_rad);
    dz_bot = -z_bottom_m * cos(cant_rad);

    dx_c4_top = z_top_m    * tan(sweep_c4_v_rad);
    dx_c4_bot = z_bottom_m * tan(sweep_c4_v_rad);

    xLE_root_v_m = x_mount_m;
    y_root_v_m   = y_mount_m;
    z_root_v_m   = z_mount_m;

    x_c4_root_v_m = xLE_root_v_m + 0.25 * c_root_v_m;

    x_c4_top_v_m  = x_c4_root_v_m + dx_c4_top;
    y_top_v_m     = y_mount_m + dy_top;
    z_top_v_m     = z_mount_m + dz_top;
    xLE_top_v_m   = x_c4_top_v_m - 0.25 * c_tip_v_m;

    x_c4_bot_v_m    = x_c4_root_v_m + dx_c4_bot;
    y_bottom_v_m    = y_mount_m + dy_bot;
    z_bottom_v_m    = z_mount_m + dz_bot;
    xLE_bottom_v_m  = x_c4_bot_v_m - 0.25 * c_tip_v_m;

    xLE_MAC_v_m = xLE_root_v_m + s_MAC_m * tan(sweep_c4_v_rad) ...
                  - 0.25 * (MAC_v_m - c_root_v_m);
    y_MAC_v_m   = y_mount_m + (topFrac - 0.5) * b_v_m * sin(cant_rad);
    z_MAC_v_m   = z_mount_m + (topFrac - 0.5) * b_v_m * cos(cant_rad);

    % ==============================================================
    % Rudder sizing / geometry (on ONE fin)
    % ==============================================================
    if ~isfield(vertIn,'rudder')
        vertIn.rudder.enable = true;
    end
    if ~isfield(vertIn.rudder,'enable');               vertIn.rudder.enable = true; end
    if ~isfield(vertIn.rudder,'eta_start');            vertIn.rudder.eta_start = 0.15; end
    if ~isfield(vertIn.rudder,'eta_end');              vertIn.rudder.eta_end   = 0.95; end
    if ~isfield(vertIn.rudder,'cf_root');              vertIn.rudder.cf_root   = 0.30; end
    if ~isfield(vertIn.rudder,'cf_tip');               vertIn.rudder.cf_tip    = 0.30; end
    if ~isfield(vertIn.rudder,'useTopOnly');           vertIn.rudder.useTopOnly = true; end

    rudderOut = struct();
    rudderOut.enable = vertIn.rudder.enable;

    if vertIn.rudder.enable
        eta1 = vertIn.rudder.eta_start;
        eta2 = vertIn.rudder.eta_end;
        cf1  = vertIn.rudder.cf_root;
        cf2  = vertIn.rudder.cf_tip;

        if eta1 < 0 || eta1 >= eta2 || eta2 > 1
            error('verticalSurfaceDesign:BadRudderEta', ...
                'Rudder eta_start and eta_end must satisfy 0 <= start < end <= 1.');
        end
        if cf1 <= 0 || cf1 >= 1 || cf2 <= 0 || cf2 >= 1
            error('verticalSurfaceDesign:BadRudderChordFrac', ...
                'Rudder chord fractions must be between 0 and 1.');
        end

        if vertIn.rudder.useTopOnly
            h_rudder_m = (eta2 - eta1) * z_top_m;

            z1_local = eta1 * z_top_m;
            z2_local = eta2 * z_top_m;

            c1 = c_root_v_m + (c_tip_v_m - c_root_v_m) * (z1_local / b_v_m);
            c2 = c_root_v_m + (c_tip_v_m - c_root_v_m) * (z2_local / b_v_m);

            c_rud1 = cf1 * c1;
            c_rud2 = cf2 * c2;

            S_rudder_m2 = 0.5 * (c_rud1 + c_rud2) * h_rudder_m;

            rudderOut.useTopOnly   = true;
            rudderOut.eta_start    = eta1;
            rudderOut.eta_end      = eta2;
            rudderOut.cf_root      = cf1;
            rudderOut.cf_tip       = cf2;
            rudderOut.height_m     = h_rudder_m;
            rudderOut.c_root_m     = c_rud1;
            rudderOut.c_tip_m      = c_rud2;
            rudderOut.S_rudder_m2  = S_rudder_m2;
            rudderOut.S_over_Sv    = S_rudder_m2 / S_v_m2;
        else
            h_rudder_m = (eta2 - eta1) * b_v_m;

            c1 = c_root_v_m + (c_tip_v_m - c_root_v_m) * eta1;
            c2 = c_root_v_m + (c_tip_v_m - c_root_v_m) * eta2;

            c_rud1 = cf1 * c1;
            c_rud2 = cf2 * c2;

            S_rudder_m2 = 0.5 * (c_rud1 + c_rud2) * h_rudder_m;

            rudderOut.useTopOnly   = false;
            rudderOut.eta_start    = eta1;
            rudderOut.eta_end      = eta2;
            rudderOut.cf_root      = cf1;
            rudderOut.cf_tip       = cf2;
            rudderOut.height_m     = h_rudder_m;
            rudderOut.c_root_m     = c_rud1;
            rudderOut.c_tip_m      = c_rud2;
            rudderOut.S_rudder_m2  = S_rudder_m2;
            rudderOut.S_over_Sv    = S_rudder_m2 / S_v_m2;
        end
    end

    % ==============================================================
    % Output
    % ==============================================================
    vertOut = struct();

    vertOut.airfoilName = airfoilName;
    vertOut.sizeMode    = vertIn.sizeMode;
    vertOut.isTwin      = isTwin;

    % Area bookkeeping
    vertOut.S_v_total_m2 = S_v_total_m2;  % total both-fin area
    vertOut.S_v_m2       = S_v_m2;        % single-fin area used for geometry

    vertOut.AR_v    = AR_v;
    vertOut.taper_v = taper_v;

    vertOut.sweep_c4_v_deg = sweep_c4_v_deg;
    vertOut.cant_deg       = cant_deg;
    vertOut.toe_deg        = toe_deg;
    vertOut.topFrac        = topFrac;

    vertOut.b_v_m      = b_v_m;
    vertOut.c_root_v_m = c_root_v_m;
    vertOut.c_tip_v_m  = c_tip_v_m;
    vertOut.MAC_v_m    = MAC_v_m;

    vertOut.x_mount_m = x_mount_m;
    vertOut.y_mount_m = y_mount_m;
    vertOut.z_mount_m = z_mount_m;

    vertOut.xLE_root_v_m = xLE_root_v_m;
    vertOut.y_root_v_m   = y_root_v_m;
    vertOut.z_root_v_m   = z_root_v_m;

    vertOut.xLE_top_v_m = xLE_top_v_m;
    vertOut.y_top_v_m   = y_top_v_m;
    vertOut.z_top_v_m   = z_top_v_m;

    vertOut.xLE_bottom_v_m = xLE_bottom_v_m;
    vertOut.y_bottom_v_m   = y_bottom_v_m;
    vertOut.z_bottom_v_m   = z_bottom_v_m;

    vertOut.xLE_MAC_v_m = xLE_MAC_v_m;
    vertOut.y_MAC_v_m   = y_MAC_v_m;
    vertOut.z_MAC_v_m   = z_MAC_v_m;

    vertOut.z_top_m    = z_top_m;
    vertOut.z_bottom_m = z_bottom_m;

    if exist('c_v','var');             vertOut.c_v = c_v; else; vertOut.c_v = NaN; end
    if exist('L_v_m','var');           vertOut.L_v_m = L_v_m; else; vertOut.L_v_m = NaN; end
    if exist('x_c4_wing_ref_m','var'); vertOut.x_c4_wing_ref_m = x_c4_wing_ref_m; else; vertOut.x_c4_wing_ref_m = NaN; end
    if exist('x_c4_vert_ref_m','var'); vertOut.x_c4_vert_ref_m = x_c4_vert_ref_m; else; vertOut.x_c4_vert_ref_m = NaN; end

    vertOut.rudder = rudderOut;
end