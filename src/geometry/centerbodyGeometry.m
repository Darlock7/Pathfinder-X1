function bodyOut = centerbodyGeometry(bodyIn)
% centerbodyGeometry
%
% Purpose:
%   First-pass centerbody / fuselage geometry generator for a flying-wing
%   style aircraft. This version uses geometry-driving inputs rather than
%   prescribing frontal area directly.
%
% Inputs:
%   bodyIn.Vp_m3                required internal volume [m^3]
%   bodyIn.b_ref_m              reference span used for width scaling [m]
%   bodyIn.widthFracOfSpan      body width as fraction of full span [-]
%   bodyIn.heightFracOfWidth    body height as fraction of body width [-]
%   bodyIn.xRef_m               body reference x-location [m]
%   bodyIn.xRefType             'nose', 'center', or 'tail'
%   bodyIn.zCenter_m            body center z-location [m]
%   bodyIn.rho_struct_eff       effective structural density [kg/m^3]
%   bodyIn.m_fixed_kg           fixed internal systems mass [kg]
%
% Optional:
%   bodyIn.minLength_m          minimum allowable body length [m]
%   bodyIn.maxWidth_m           max allowable body width [m]
%   bodyIn.maxHeight_m          max allowable body height [m]
%
% Outputs:
%   bodyOut.W_body_m            body width [m]
%   bodyOut.H_body_m            body height [m]
%   bodyOut.L_body_m            body length [m]
%   bodyOut.A_front_m2          frontal area [m^2]
%   bodyOut.S_wet_est_m2        crude wetted-area estimate [m^2]
%   bodyOut.x_body_nose_m       body nose x [m]
%   bodyOut.x_body_tail_m       body tail x [m]
%   bodyOut.x_body_center_m     body center x [m]
%   bodyOut.y_body_min_m        min y [m]
%   bodyOut.y_body_max_m        max y [m]
%   bodyOut.z_body_min_m        min z [m]
%   bodyOut.z_body_max_m        max z [m]
%   bodyOut.vertices            8 box vertices [8 x 3]
%   bodyOut.m_struct_kg         structural mass [kg]
%   bodyOut.m_fixed_kg          fixed body systems mass [kg]
%   bodyOut.m_total_kg          total body-related mass [kg]

    arguments
        bodyIn struct
    end

    req = {'Vp_m3','b_ref_m','widthFracOfSpan','heightFracOfWidth', ...
           'xRef_m','xRefType','zCenter_m','rho_struct_eff','m_fixed_kg'};
    for k = 1:numel(req)
        if ~isfield(bodyIn, req{k})
            error('centerbodyGeometry:MissingField', ...
                'Missing required input field: %s', req{k});
        end
    end

    Vp_m3             = bodyIn.Vp_m3;
    b_ref_m           = bodyIn.b_ref_m;
    widthFracOfSpan   = bodyIn.widthFracOfSpan;
    heightFracOfWidth = bodyIn.heightFracOfWidth;
    xRef_m            = bodyIn.xRef_m;
    xRefType          = lower(string(bodyIn.xRefType));
    zCenter_m         = bodyIn.zCenter_m;
    rho_struct_eff    = bodyIn.rho_struct_eff;
    m_fixed_kg        = bodyIn.m_fixed_kg;

    if isfield(bodyIn,'minLength_m'); minLength_m = bodyIn.minLength_m; else; minLength_m = 0; end
    if isfield(bodyIn,'maxWidth_m');  maxWidth_m  = bodyIn.maxWidth_m;  else; maxWidth_m  = inf; end
    if isfield(bodyIn,'maxHeight_m'); maxHeight_m = bodyIn.maxHeight_m; else; maxHeight_m = inf; end

    if Vp_m3 <= 0
        error('centerbodyGeometry:BadVolume', 'Vp_m3 must be positive.');
    end
    if b_ref_m <= 0
        error('centerbodyGeometry:BadSpan', 'b_ref_m must be positive.');
    end
    if widthFracOfSpan <= 0 || widthFracOfSpan >= 1
        error('centerbodyGeometry:BadWidthFrac', ...
            'widthFracOfSpan must satisfy 0 < widthFracOfSpan < 1.');
    end
    if heightFracOfWidth <= 0 || heightFracOfWidth >= 1
        error('centerbodyGeometry:BadHeightFrac', ...
            'heightFracOfWidth must satisfy 0 < heightFracOfWidth < 1.');
    end
    if rho_struct_eff < 0
        error('centerbodyGeometry:BadDensity', 'rho_struct_eff must be nonnegative.');
    end
    if m_fixed_kg < 0
        error('centerbodyGeometry:BadFixedMass', 'm_fixed_kg must be nonnegative.');
    end
    if ~any(strcmp(xRefType, ["nose","center","tail"]))
        error('centerbodyGeometry:BadXRefType', ...
            'xRefType must be ''nose'', ''center'', or ''tail''.');
    end

    % --------------------------------------------------------------
    % Geometry from span fraction and height fraction
    % --------------------------------------------------------------
    W_body_m = widthFracOfSpan * b_ref_m;
    W_body_m = min(W_body_m, maxWidth_m);

    H_body_m = heightFracOfWidth * W_body_m;
    H_body_m = min(H_body_m, maxHeight_m);

    A_front_m2 = W_body_m * H_body_m;

    if A_front_m2 <= 0
        error('centerbodyGeometry:BadFrontArea', ...
            'Computed frontal area must be positive.');
    end

    L_body_m = Vp_m3 / A_front_m2;
    L_body_m = max(L_body_m, minLength_m);

    % If min length forced a longer body, update implied internal volume
    V_geom_m3 = L_body_m * A_front_m2;

    % --------------------------------------------------------------
    % x-location bookkeeping
    % --------------------------------------------------------------
    switch xRefType
        case "nose"
            x_body_nose_m   = xRef_m;
            x_body_tail_m   = x_body_nose_m + L_body_m;
            x_body_center_m = x_body_nose_m + 0.5 * L_body_m;

        case "center"
            x_body_center_m = xRef_m;
            x_body_nose_m   = x_body_center_m - 0.5 * L_body_m;
            x_body_tail_m   = x_body_center_m + 0.5 * L_body_m;

        case "tail"
            x_body_tail_m   = xRef_m;
            x_body_nose_m   = x_body_tail_m - L_body_m;
            x_body_center_m = x_body_nose_m + 0.5 * L_body_m;
    end

    % --------------------------------------------------------------
    % y/z extents
    % --------------------------------------------------------------
    y_body_min_m = -0.5 * W_body_m;
    y_body_max_m =  0.5 * W_body_m;

    z_body_min_m = zCenter_m - 0.5 * H_body_m;
    z_body_max_m = zCenter_m + 0.5 * H_body_m;

    % --------------------------------------------------------------
    % Crude wetted area estimate for a box-like centerbody
    % --------------------------------------------------------------
    S_wet_est_m2 = 2 * (L_body_m*W_body_m + L_body_m*H_body_m + W_body_m*H_body_m);

    % --------------------------------------------------------------
    % First-pass effective structural mass
    % --------------------------------------------------------------
    m_struct_kg = rho_struct_eff * V_geom_m3;
    m_total_kg  = m_struct_kg + m_fixed_kg;

    % --------------------------------------------------------------
    % Vertices for plotting
    % --------------------------------------------------------------
    x0 = x_body_nose_m;
    y0 = y_body_min_m;
    z0 = z_body_min_m;

    X = [0 1 1 0 0 1 1 0]*L_body_m + x0;
    Y = [0 0 1 1 0 0 1 1]*W_body_m + y0;
    Z = [0 0 0 0 1 1 1 1]*H_body_m + z0;

    vertices = [X' Y' Z'];

    % --------------------------------------------------------------
    % Output
    % --------------------------------------------------------------
    bodyOut = struct();

    bodyOut.Vp_m3             = Vp_m3;
    bodyOut.V_geom_m3         = V_geom_m3;
    bodyOut.b_ref_m           = b_ref_m;
    bodyOut.widthFracOfSpan   = widthFracOfSpan;
    bodyOut.heightFracOfWidth = heightFracOfWidth;

    bodyOut.W_body_m          = W_body_m;
    bodyOut.H_body_m          = H_body_m;
    bodyOut.L_body_m          = L_body_m;
    bodyOut.A_front_m2        = A_front_m2;
    bodyOut.S_wet_est_m2      = S_wet_est_m2;

    bodyOut.xRef_m            = xRef_m;
    bodyOut.xRefType          = char(xRefType);
    bodyOut.x_body_nose_m     = x_body_nose_m;
    bodyOut.x_body_tail_m     = x_body_tail_m;
    bodyOut.x_body_center_m   = x_body_center_m;

    bodyOut.y_body_min_m      = y_body_min_m;
    bodyOut.y_body_max_m      = y_body_max_m;
    bodyOut.z_body_min_m      = z_body_min_m;
    bodyOut.z_body_max_m      = z_body_max_m;

    bodyOut.vertices          = vertices;

    bodyOut.rho_struct_eff    = rho_struct_eff;
    bodyOut.m_struct_kg       = m_struct_kg;
    bodyOut.m_fixed_kg        = m_fixed_kg;
    bodyOut.m_total_kg        = m_total_kg;
end