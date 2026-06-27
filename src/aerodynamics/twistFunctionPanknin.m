function twistOut = twistFunctionPanknin(twistIn)
% twistFunctionPanknin
%
% Purpose:
%   Compute required total twist and geometric twist using the
%   Panknin-style formula, then generate a spanwise geometric
%   twist distribution.
%
% Formula used:
%   alphaTotal = [ (K1*CMroot + K2*CMtip) - (CL*ST) ] ...
%                / [ 1.4e-5 * lambda^1.43 * C4Sweep_deg ]
%
%   alphaGeo = alphaTotal - (alphaL0root - alphaL0tip)
%
% where
%   lambda = aspect ratio AR
%   TR     = c_tip / c_root
%   K1     = 0.25 * (3 + 2*TR + TR^2) / (1 + TR + TR^2)
%   K2     = 1 - K1
%
% INPUTS
%   twistIn.b_m                : full span [m]
%   twistIn.AR                 : wing aspect ratio [-]
%   twistIn.c_root_m           : root chord [m]
%   twistIn.c_tip_m            : tip chord [m]
%   twistIn.sweep_c4_deg       : quarter-chord sweep angle [deg]
%   twistIn.alphaL0_root_deg   : root zero-lift angle [deg]
%   twistIn.alphaL0_tip_deg    : tip zero-lift angle [deg]
%   twistIn.Cm_root            : root section pitching moment coefficient [-]
%   twistIn.Cm_tip             : tip section pitching moment coefficient [-]
%   twistIn.CL_design          : design lift coefficient [-]
%   twistIn.static_margin      : stability factor / static margin [-]
%
% Optional inputs
%   twistIn.model              : 'linear' only for now
%   twistIn.twist_root_deg     : root geometric twist [deg], default 0
%   twistIn.Nspan              : number of semispan stations, default 200
%
% OUTPUTS
%   twistOut.y_m
%   twistOut.eta
%   twistOut.twist_deg
%   twistOut.model
%   twistOut.twist_root_deg
%   twistOut.twist_tip_deg
%   twistOut.alphaTotal_deg
%   twistOut.alphaGeo_deg
%   twistOut.aeroTwist_deg
%   twistOut.lambda_panknin
%   twistOut.taperRatio
%   twistOut.K1
%   twistOut.K2
%   twistOut.b_half_m

    arguments
        twistIn struct
    end

    %% ---------------- Defaults ----------------
    if ~isfield(twistIn, 'model') || isempty(twistIn.model)
        twistIn.model = 'linear';
    end
    if ~isfield(twistIn, 'twist_root_deg') || isempty(twistIn.twist_root_deg)
        twistIn.twist_root_deg = 0.0;
    end
    if ~isfield(twistIn, 'Nspan') || isempty(twistIn.Nspan)
        twistIn.Nspan = 200;
    end

    %% ---------------- Required fields ----------------
    req = { ...
        'b_m', ...
        'AR', ...
        'c_root_m', ...
        'c_tip_m', ...
        'sweep_c4_deg', ...
        'alphaL0_root_deg', ...
        'alphaL0_tip_deg', ...
        'Cm_root', ...
        'Cm_tip', ...
        'CL_design', ...
        'static_margin'};

    for k = 1:numel(req)
        if ~isfield(twistIn, req{k})
            error('twistFunctionPanknin:MissingField', ...
                'Missing required input field: %s', req{k});
        end
    end

    %% ---------------- Unpack ----------------
    b_m              = twistIn.b_m;              % [m]
    AR               = twistIn.AR;               % [-]
    c_root_m         = twistIn.c_root_m;         % [m]
    c_tip_m          = twistIn.c_tip_m;          % [m]
    sweep_c4_deg     = twistIn.sweep_c4_deg;     % [deg]
    alphaL0_root_deg = twistIn.alphaL0_root_deg; % [deg]
    alphaL0_tip_deg  = twistIn.alphaL0_tip_deg;  % [deg]
    Cm_root          = twistIn.Cm_root;          % [-]
    Cm_tip           = twistIn.Cm_tip;           % [-]
    CL_design        = twistIn.CL_design;        % [-]
    static_margin    = twistIn.static_margin;    % [-]

    if b_m <= 0 || AR <= 0 || c_root_m <= 0 || c_tip_m <= 0
        error('twistFunctionPanknin:BadGeometry', ...
            'Span, aspect ratio, and chord inputs must be positive.');
    end
    if sweep_c4_deg <= 0
        error('twistFunctionPanknin:BadSweep', ...
            'sweep_c4_deg must be positive for this formula.');
    end

    %% ---------------- Panknin geometry terms ----------------
    lambda_panknin = AR;
    taperRatio = c_tip_m / c_root_m;

    K1 = 0.25 * (3 + 2*taperRatio + taperRatio^2) / ...
                (1 + taperRatio + taperRatio^2);
    K2 = 1 - K1;

    %% ---------------- Twist calculations ----------------
    numerator = (K1 * Cm_root + K2 * Cm_tip) - (CL_design * static_margin);
    denominator = 1.4e-5 * lambda_panknin^1.43 * sweep_c4_deg;

    alphaTotal_deg = numerator / denominator;

    % This is the aerodynamic-twist term as it appears in the Panknin formula
    aeroTwist_deg = alphaL0_root_deg - alphaL0_tip_deg;

    % Required geometric twist
    alphaGeo_deg = alphaTotal_deg - aeroTwist_deg;

    %% ---------------- Spanwise geometric twist distribution ----------------
    b_half_m = 0.5 * b_m;
    Nspan = max(2, round(twistIn.Nspan));

    y_m = linspace(0, b_half_m, Nspan).';
    eta = y_m / b_half_m;

    switch lower(strtrim(twistIn.model))
        case 'linear'
            twist_tip_deg = alphaGeo_deg;
            twist_deg = twistIn.twist_root_deg + ...
                (twist_tip_deg - twistIn.twist_root_deg) .* eta;

        otherwise
            error('twistFunctionPanknin:UnsupportedModel', ...
                'Unsupported twist model: %s', twistIn.model);
    end

    %% ---------------- Output ----------------
    twistOut = struct();
    twistOut.y_m = y_m;
    twistOut.eta = eta;
    twistOut.twist_deg = twist_deg;

    twistOut.model = lower(strtrim(twistIn.model));
    twistOut.twist_root_deg = twistIn.twist_root_deg;
    twistOut.twist_tip_deg  = twist_tip_deg;

    twistOut.alphaTotal_deg = alphaTotal_deg;
    twistOut.alphaGeo_deg   = alphaGeo_deg;
    twistOut.aeroTwist_deg  = aeroTwist_deg;

    twistOut.lambda_panknin = lambda_panknin;
    twistOut.taperRatio     = taperRatio;
    twistOut.K1             = K1;
    twistOut.K2             = K2;

    twistOut.numerator      = numerator;
    twistOut.denominator    = denominator;

    twistOut.b_half_m       = b_half_m;
    twistOut.sweep_c4_deg   = sweep_c4_deg;
end