function exportNimbusToRFX(rfxIn, outputDir, kexTemplatePath)
% exportNimbusToRFX
%
% Purpose:
%   Generate a RealFlight 9.5S compatible .RFX file from Nimbus aircraft
%   design parameters computed by the MAE155B MATLAB pipeline.
%
%   The .RFX is a ZIP archive containing:
%       Nimbus_EA.rfvehicle  — all flight physics (text, fully generated)
%       Nimbus_EA.bse        — metadata (text, fully generated)
%
%   A .kex 3D model is NOT generated (binary format). RealFlight will load
%   the aircraft with no visual mesh but correct physics. To add the visual
%   model: rename your FBX export to Nimbus_EA.kex, add it to the ZIP, and
%   update the XK_FileName in Nimbus_EA.bse.
%
% INPUTS (rfxIn struct):
%   === Airframe / Weight ===
%   .W_total_N          [N]   total gross weight (loaded)
%   .W_wing_N           [N]   wing structural weight
%   .W_centerbody_N     [N]   centerbody / fuselage weight (incl. motor mount)
%   .cg_x_m             [m]   CG x-position from leading edge of root chord
%   .cg_z_m             [m]   CG z-position from wing mean surface (+ = above)
%   .Ixx_kgm2           [kg*m^2]  roll inertia about CG
%   .Iyy_kgm2           [kg*m^2]  pitch inertia about CG
%   .Izz_kgm2           [kg*m^2]  yaw inertia about CG
%
%   === Wing Geometry ===
%   .b_m                [m]   full wingspan
%   .c_root_m           [m]   root chord
%   .c_tip_m            [m]   tip chord
%   .sweep_LE_deg       [deg] leading-edge sweep angle
%   .dihedral_deg       [deg] geometric dihedral (typically 0 for flying wing)
%   .airfoil_name       [str] RealFlight airfoil name (e.g. 'NACA 2412')
%
%   === Elevon Control Surface ===
%   .cs_eta_start       [-]   inboard  span fraction on semispan
%   .cs_eta_end         [-]   outboard span fraction on semispan
%   .cs_chord_frac      [-]   elevon chord / local chord
%   .cs_deflect_up_deg  [deg] max up deflection (positive = trailing edge up)
%   .cs_deflect_dn_deg  [deg] max down deflection
%
%   === Winglets ===
%   .has_winglets       [bool] include delta winglets (default: true)
%   .wl_span_m          [m]   winglet span (height when vertical)
%   .wl_c_root_m        [m]   winglet root chord (= wing tip chord typically)
%   .wl_c_tip_m         [m]   winglet tip chord
%   .wl_sweep_LE_deg    [deg] winglet LE sweep
%
%   === Propulsion ===
%   .D_prop_in          [in]  propeller diameter
%   .pitch_prop_in      [in]  propeller pitch
%   .is_pusher          [bool] pusher config (default: false = tractor)
%   .motor_name         [str] RealFlight motor string (see note below)
%
%   === Battery ===
%   .n_cells_series     [-]   cells in series (e.g. 3 for 3S)
%   .n_cells_parallel   [-]   cells in parallel (e.g. 1 for 1P)
%   .cell_capacity_mah  [-]   cell capacity in mAh (e.g. 2200)
%   .batt_mass_kg       [kg]  battery mass
%   .batt_length_m      [m]   battery pack length (Y dimension)
%   .batt_x_m           [m]   battery CG x from wing LE (for CG tuning)
%
%   === Output ===
%   outputDir           [str] folder to write Nimbus_EA.RFX  (default: pwd)
%
% MOTOR NAMES (TorqueGeneratorElectric):
%   RealFlight only accepts exact strings from its motor database.
%   Some known electric motors in RF9:
%       'ElectriFly RimFire 28-30-1100 Outrunner'   (KV~1100, ~250W)
%       'ElectriFly RimFire 35-30-910 Outrunner'    (KV~910,  ~400W)
%       'ElectriFly RimFire 42-40-800 Outrunner'    (KV~800,  ~800W)
%       'Spektrum Avian 2826-1100Kv'                 (KV~1100)
%   If unsure, leave the default — physics still work, only sound/visual differ.
%
% VALIDATION:
%   After importing into RealFlight:
%     1. Check CG marker position in the Aircraft Editor
%     2. Verify wing area matches S_ref from MATLAB
%     3. Check static stability by flying in Trainer mode
%
% EXAMPLE:
%   rfxIn = nimbusRFXParams();   % call the parameter collector below
%   exportNimbusToRFX(rfxIn, 'exports/');

    %% ---- Defaults ----
    % Save project root immediately — previous failed runs may have left cd in tempdir
    projectRoot = pwd;

    if nargin < 2 || isempty(outputDir)
        outputDir = projectRoot;
    end
    if nargin < 3 || isempty(kexTemplatePath)
        % default: use the Extra330 placeholder extracted during setup
        kexTemplatePath = '/tmp/rfx_inspect/Pilot RC Extra330SX103.kex';
    end
    hasKex       = exist(kexTemplatePath, 'file') == 2;
    baseName     = 'Nimbus';
    templateBase = 'Pilot RC Extra330SX103';
    kexFile      = [baseName, '.kex'];
    if ~isfield(rfxIn,'has_winglets'),      rfxIn.has_winglets      = true;  end
    if ~isfield(rfxIn,'is_pusher'),         rfxIn.is_pusher         = false; end
    if ~isfield(rfxIn,'dihedral_deg'),      rfxIn.dihedral_deg      = 0;     end
    if ~isfield(rfxIn,'cg_z_m'),            rfxIn.cg_z_m            = 0;     end
    if ~isfield(rfxIn,'n_cells_parallel'),  rfxIn.n_cells_parallel  = 1;     end
    if ~isfield(rfxIn,'motor_name')
        rfxIn.motor_name = 'ElectriFly RimFire 28-30-1100 Outrunner';
    end
    if ~isfield(rfxIn,'airfoil_name')
        rfxIn.airfoil_name = 'NACA 2412';
    end

    %% ---- Derived geometry ----
    g           = 9.81;
    W_total_kg  = rfxIn.W_total_N / g;
    W_wing_kg   = rfxIn.W_wing_N  / g;
    W_cb_kg     = rfxIn.W_centerbody_N / g;

    b_half      = rfxIn.b_m / 2;           % [m] semispan
    c_root      = rfxIn.c_root_m;
    c_tip       = rfxIn.c_tip_m;
    MAC         = (2/3) * c_root * (1 + rfxIn.c_tip_m/c_root + (rfxIn.c_tip_m/c_root)^2) ...
                  / (1 + rfxIn.c_tip_m/c_root);

    D_prop_m    = rfxIn.D_prop_in  * 0.0254;   % [m]
    pitch_m     = rfxIn.pitch_prop_in * 0.0254; % [m]

    % Overall bounding box for PhysicsDimensionsOnReset
    tot_len_m   = c_root;                  % flying wing length = root chord
    tot_ht_m    = 0.06 * c_root;           % estimated thickness

    % CG adjustment: offset from bounding box center to actual CG
    % Box center is at (b/2, c_root/2, tot_ht/2) in local frame
    % CG is at (0, cg_x_m, cg_z_m) from LE of root chord
    % CGAdj_Y = box_center_y - cg_x (positive Y = forward in RFX)
    cg_adj_x = 0;                           % symmetric about X
    cg_adj_y = (c_root/2 - rfxIn.cg_x_m);  % forward shift from box center
    cg_adj_z = rfxIn.cg_z_m;               % vertical offset

    % Wing LE position relative to centerbody origin (at CG)
    % In RFX: Y positive = forward, Z positive = up
    wing_loc_y  =  rfxIn.cg_x_m;   % how far forward LE is from CG
    wing_loc_z  = -rfxIn.cg_z_m;   % vertical offset (sign flip: down = negative Z)

    % Prop location relative to centerbody (forward = positive Y in RFX)
    if rfxIn.is_pusher
        prop_loc_y = -(c_root - rfxIn.cg_x_m);  % behind CG
    else
        prop_loc_y =  rfxIn.cg_x_m;             % forward of CG (at LE)
    end

    % Elevon geometry
    cs_span_frac = rfxIn.cs_eta_end - rfxIn.cs_eta_start;
    cs_len_m     = cs_span_frac * b_half;
    cs_dist_tip  = (1 - rfxIn.cs_eta_end) * b_half;

    % Winglet location: at wing tip
    wl_loc_x    = b_half;   % at tip, X = half span
    wl_loc_y    = -(c_root - c_tip + c_tip/2 - rfxIn.cg_x_m); % near TE at tip, approx
    wl_loc_z    = 0;

    % Battery location relative to centerbody
    batt_loc_y  = rfxIn.cg_x_m - rfxIn.batt_x_m;  % fwd/aft of CG

    %% ---- Build rfvehicle text ----
    fv = '';

    % --- Radio channels (minimal: ch100=right elevon, ch101=left elevon, ch102=throttle) ---
    fv = [fv, sprintf('[AirplaneSoftwareRadio]\n')];
    fv = [fv, sprintf('OutputChannelsArray=INTARRAY:1~2~3~\n\n')];
    fv = [fv, sprintf('   SUBGROUP[OutputChannelsArray]\n\n')];

    % Channel 1 — Right elevon (aileron + elevator mix)
    fv = [fv, sprintf('      SUBGROUP[#1]\n')];
    fv = [fv, sprintf('         Expo=FLOAT:0.25\n')];
    fv = [fv, sprintf('         ExpoLowRates=FLOAT:0.25\n')];
    fv = [fv, sprintf('         InputFeedsArray=INTARRAY:1~2~\n')];
    fv = [fv, sprintf('         LowRates=FLOAT:0.75\n')];
    fv = [fv, sprintf('         Trim=FLOAT:0.\n\n')];
    fv = [fv, sprintf('         SUBGROUP[ExpoWhen]\n')];
    fv = [fv, sprintf('            Value1=FLOAT:0.\n')];
    fv = [fv, sprintf('            Value2=FLOAT:0.\n')];
    fv = [fv, sprintf('            WhenInput=INT:100\n')];
    fv = [fv, sprintf('            WhenLogic=INT:1\n')];
    fv = [fv, sprintf('         ENDGROUP[ExpoWhen]\n\n')];
    fv = [fv, sprintf('         SUBGROUP[LowRatesActivatedWhen]\n')];
    fv = [fv, sprintf('            Value1=FLOAT:0.\n')];
    fv = [fv, sprintf('            Value2=FLOAT:0.\n')];
    fv = [fv, sprintf('            WhenInput=INT:100\n')];
    fv = [fv, sprintf('            WhenLogic=INT:1\n')];
    fv = [fv, sprintf('         ENDGROUP[LowRatesActivatedWhen]\n\n')];
    fv = [fv, sprintf('         SUBGROUP[InputFeedsArray]\n\n')];
    % aileron mix
    fv = [fv, sprintf('            SUBGROUP[#1]\n')];
    fv = [fv, sprintf('               CurveInputValues=FLOATARRAY:\n')];
    fv = [fv, sprintf('               CurveOutputValues=FLOATARRAY:\n')];
    fv = [fv, sprintf('               InputChannel=INT:100\n')];
    fv = [fv, sprintf('               InputFeedType=INT:1\n')];
    fv = [fv, sprintf('               InputName=STRING:Aileron\n')];
    fv = [fv, sprintf('               Logic=INT:0\n')];
    fv = [fv, sprintf('               SimpleMaxPercent=FLOAT:0.5\n')];
    fv = [fv, sprintf('               SimpleReversed=BOOL:No\n\n')];
    fv = [fv, sprintf('               SUBGROUP[InputActivatedWhen]\n')];
    fv = [fv, sprintf('                  Value1=FLOAT:0.\n')];
    fv = [fv, sprintf('                  Value2=FLOAT:0.\n')];
    fv = [fv, sprintf('                  WhenInput=INT:100\n')];
    fv = [fv, sprintf('                  WhenLogic=INT:1\n')];
    fv = [fv, sprintf('               ENDGROUP[InputActivatedWhen]\n')];
    fv = [fv, sprintf('            ENDGROUP[#1]\n\n')];
    % elevator mix
    fv = [fv, sprintf('            SUBGROUP[#2]\n')];
    fv = [fv, sprintf('               CurveInputValues=FLOATARRAY:\n')];
    fv = [fv, sprintf('               CurveOutputValues=FLOATARRAY:\n')];
    fv = [fv, sprintf('               InputChannel=INT:101\n')];
    fv = [fv, sprintf('               InputFeedType=INT:1\n')];
    fv = [fv, sprintf('               InputName=STRING:Elevator\n')];
    fv = [fv, sprintf('               Logic=INT:0\n')];
    fv = [fv, sprintf('               SimpleMaxPercent=FLOAT:0.5\n')];
    fv = [fv, sprintf('               SimpleReversed=BOOL:No\n\n')];
    fv = [fv, sprintf('               SUBGROUP[InputActivatedWhen]\n')];
    fv = [fv, sprintf('                  Value1=FLOAT:0.\n')];
    fv = [fv, sprintf('                  Value2=FLOAT:0.\n')];
    fv = [fv, sprintf('                  WhenInput=INT:100\n')];
    fv = [fv, sprintf('                  WhenLogic=INT:1\n')];
    fv = [fv, sprintf('               ENDGROUP[InputActivatedWhen]\n')];
    fv = [fv, sprintf('            ENDGROUP[#2]\n\n')];
    fv = [fv, sprintf('         ENDGROUP[InputFeedsArray]\n')];
    fv = [fv, sprintf('      ENDGROUP[#1]\n\n\n')];

    % Channel 2 — Left elevon (aileron reversed + elevator)
    fv = [fv, sprintf('      SUBGROUP[#2]\n')];
    fv = [fv, sprintf('         Expo=FLOAT:0.25\n')];
    fv = [fv, sprintf('         ExpoLowRates=FLOAT:0.25\n')];
    fv = [fv, sprintf('         InputFeedsArray=INTARRAY:1~2~\n')];
    fv = [fv, sprintf('         LowRates=FLOAT:0.75\n')];
    fv = [fv, sprintf('         Trim=FLOAT:0.\n\n')];
    fv = [fv, sprintf('         SUBGROUP[ExpoWhen]\n')];
    fv = [fv, sprintf('            Value1=FLOAT:0.\n')];
    fv = [fv, sprintf('            Value2=FLOAT:0.\n')];
    fv = [fv, sprintf('            WhenInput=INT:100\n')];
    fv = [fv, sprintf('            WhenLogic=INT:1\n')];
    fv = [fv, sprintf('         ENDGROUP[ExpoWhen]\n\n')];
    fv = [fv, sprintf('         SUBGROUP[LowRatesActivatedWhen]\n')];
    fv = [fv, sprintf('            Value1=FLOAT:0.\n')];
    fv = [fv, sprintf('            Value2=FLOAT:0.\n')];
    fv = [fv, sprintf('            WhenInput=INT:100\n')];
    fv = [fv, sprintf('            WhenLogic=INT:1\n')];
    fv = [fv, sprintf('         ENDGROUP[LowRatesActivatedWhen]\n\n')];
    fv = [fv, sprintf('         SUBGROUP[InputFeedsArray]\n\n')];
    fv = [fv, sprintf('            SUBGROUP[#1]\n')];
    fv = [fv, sprintf('               CurveInputValues=FLOATARRAY:\n')];
    fv = [fv, sprintf('               CurveOutputValues=FLOATARRAY:\n')];
    fv = [fv, sprintf('               InputChannel=INT:100\n')];
    fv = [fv, sprintf('               InputFeedType=INT:1\n')];
    fv = [fv, sprintf('               InputName=STRING:Aileron\n')];
    fv = [fv, sprintf('               Logic=INT:0\n')];
    fv = [fv, sprintf('               SimpleMaxPercent=FLOAT:0.5\n')];
    fv = [fv, sprintf('               SimpleReversed=BOOL:Yes\n\n')];  % reversed for left elevon
    fv = [fv, sprintf('               SUBGROUP[InputActivatedWhen]\n')];
    fv = [fv, sprintf('                  Value1=FLOAT:0.\n')];
    fv = [fv, sprintf('                  Value2=FLOAT:0.\n')];
    fv = [fv, sprintf('                  WhenInput=INT:100\n')];
    fv = [fv, sprintf('                  WhenLogic=INT:1\n')];
    fv = [fv, sprintf('               ENDGROUP[InputActivatedWhen]\n')];
    fv = [fv, sprintf('            ENDGROUP[#1]\n\n')];
    fv = [fv, sprintf('            SUBGROUP[#2]\n')];
    fv = [fv, sprintf('               CurveInputValues=FLOATARRAY:\n')];
    fv = [fv, sprintf('               CurveOutputValues=FLOATARRAY:\n')];
    fv = [fv, sprintf('               InputChannel=INT:101\n')];
    fv = [fv, sprintf('               InputFeedType=INT:1\n')];
    fv = [fv, sprintf('               InputName=STRING:Elevator\n')];
    fv = [fv, sprintf('               Logic=INT:0\n')];
    fv = [fv, sprintf('               SimpleMaxPercent=FLOAT:0.5\n')];
    fv = [fv, sprintf('               SimpleReversed=BOOL:No\n\n')];
    fv = [fv, sprintf('               SUBGROUP[InputActivatedWhen]\n')];
    fv = [fv, sprintf('                  Value1=FLOAT:0.\n')];
    fv = [fv, sprintf('                  Value2=FLOAT:0.\n')];
    fv = [fv, sprintf('                  WhenInput=INT:100\n')];
    fv = [fv, sprintf('                  WhenLogic=INT:1\n')];
    fv = [fv, sprintf('               ENDGROUP[InputActivatedWhen]\n')];
    fv = [fv, sprintf('            ENDGROUP[#2]\n\n')];
    fv = [fv, sprintf('         ENDGROUP[InputFeedsArray]\n')];
    fv = [fv, sprintf('      ENDGROUP[#2]\n\n\n')];

    % Channel 3 — Throttle
    fv = [fv, sprintf('      SUBGROUP[#3]\n')];
    fv = [fv, sprintf('         Expo=FLOAT:0.\n')];
    fv = [fv, sprintf('         ExpoLowRates=FLOAT:0.\n')];
    fv = [fv, sprintf('         InputFeedsArray=INTARRAY:1~\n')];
    fv = [fv, sprintf('         LowRates=FLOAT:1.\n')];
    fv = [fv, sprintf('         Trim=FLOAT:0.\n\n')];
    fv = [fv, sprintf('         SUBGROUP[ExpoWhen]\n')];
    fv = [fv, sprintf('            Value1=FLOAT:0.\n')];
    fv = [fv, sprintf('            Value2=FLOAT:0.\n')];
    fv = [fv, sprintf('            WhenInput=INT:100\n')];
    fv = [fv, sprintf('            WhenLogic=INT:1\n')];
    fv = [fv, sprintf('         ENDGROUP[ExpoWhen]\n\n')];
    fv = [fv, sprintf('         SUBGROUP[LowRatesActivatedWhen]\n')];
    fv = [fv, sprintf('            Value1=FLOAT:0.\n')];
    fv = [fv, sprintf('            Value2=FLOAT:0.\n')];
    fv = [fv, sprintf('            WhenInput=INT:100\n')];
    fv = [fv, sprintf('            WhenLogic=INT:1\n')];
    fv = [fv, sprintf('         ENDGROUP[LowRatesActivatedWhen]\n\n')];
    fv = [fv, sprintf('         SUBGROUP[InputFeedsArray]\n\n')];
    fv = [fv, sprintf('            SUBGROUP[#1]\n')];
    fv = [fv, sprintf('               CurveInputValues=FLOATARRAY:\n')];
    fv = [fv, sprintf('               CurveOutputValues=FLOATARRAY:\n')];
    fv = [fv, sprintf('               InputChannel=INT:102\n')];
    fv = [fv, sprintf('               InputFeedType=INT:1\n')];
    fv = [fv, sprintf('               InputName=STRING:Throttle\n')];
    fv = [fv, sprintf('               Logic=INT:0\n')];
    fv = [fv, sprintf('               SimpleMaxPercent=FLOAT:1.\n')];
    fv = [fv, sprintf('               SimpleReversed=BOOL:No\n\n')];
    fv = [fv, sprintf('               SUBGROUP[InputActivatedWhen]\n')];
    fv = [fv, sprintf('                  Value1=FLOAT:0.\n')];
    fv = [fv, sprintf('                  Value2=FLOAT:0.\n')];
    fv = [fv, sprintf('                  WhenInput=INT:100\n')];
    fv = [fv, sprintf('                  WhenLogic=INT:1\n')];
    fv = [fv, sprintf('               ENDGROUP[InputActivatedWhen]\n')];
    fv = [fv, sprintf('            ENDGROUP[#1]\n\n')];
    fv = [fv, sprintf('         ENDGROUP[InputFeedsArray]\n')];
    fv = [fv, sprintf('      ENDGROUP[#3]\n\n')];
    fv = [fv, sprintf('   ENDGROUP[OutputChannelsArray]\n\n')];

    % --- Main section ---
    fv = [fv, sprintf('[Main]\n')];
    fv = [fv, sprintf('AircraftType=INT:1\n')];
    fv = [fv, sprintf('AppVersionBuild=UINT64:950038\n')];
    % BasedOn must reference an installed aircraft — Extra330 is our visual base
    fv = [fv, sprintf('BasedOn=STRING:%s\n', templateBase)];
    fv = [fv, sprintf('CDName=STRING:RealFlight G3\n')];
    fv = [fv, sprintf('CommentTSTRING=STRING:MAE155B Nimbus — flying wing delivery UAV\n')];
    fv = [fv, sprintf('EnableDaytimeLights=BOOL:No\n')];
    fv = [fv, sprintf('LaunchMethod=INT:3\n')];  % hand-launch
    fv = [fv, sprintf('PLNVersion=INT:3\n')];
    fv = [fv, sprintf('Version=INT:1\n\n')];

    % --- Electronics ---
    fv = [fv, sprintf('[VehicleElectronics]\n')];
    fv = [fv, sprintf('NextHR_SIGNAL_GENERATOR_ID=INT:105\n')];
    fv = [fv, sprintf('NumReceiverChannels=INT:4\n')];
    fv = [fv, sprintf('NumReceiverChannels2=INT:4\n')];
    fv = [fv, sprintf('SignalGenerators=INTARRAY:100~101~102~\n')];
    fv = [fv, sprintf('TransmitterType=INT:2\n\n')];
    % Right elevon servo
    fv = [fv, sprintf('   SUBGROUP[#100]\n')];
    fv = [fv, sprintf('      ConnectTo_InternalName=INT:1\n')];
    fv = [fv, sprintf('      ServoType=INT:1\n')];
    fv = [fv, sprintf('      Speed=FLOAT:0.012\n')];
    fv = [fv, sprintf('      Type=STRING:PhysicalServo\n')];
    fv = [fv, sprintf('      UserNameTSTRING=STRING:Right Elevon Servo\n')];
    fv = [fv, sprintf('      Volume=FLOAT:5.\n')];
    fv = [fv, sprintf('   ENDGROUP[#100]\n\n')];
    % Left elevon servo
    fv = [fv, sprintf('   SUBGROUP[#101]\n')];
    fv = [fv, sprintf('      ConnectTo_InternalName=INT:2\n')];
    fv = [fv, sprintf('      ServoType=INT:1\n')];
    fv = [fv, sprintf('      Speed=FLOAT:0.012\n')];
    fv = [fv, sprintf('      Type=STRING:PhysicalServo\n')];
    fv = [fv, sprintf('      UserNameTSTRING=STRING:Left Elevon Servo\n')];
    fv = [fv, sprintf('      Volume=FLOAT:5.\n')];
    fv = [fv, sprintf('   ENDGROUP[#101]\n\n')];
    % Throttle ESC
    fv = [fv, sprintf('   SUBGROUP[#102]\n')];
    fv = [fv, sprintf('      ConnectTo_InternalName=INT:3\n')];
    fv = [fv, sprintf('      ServoType=INT:1\n')];
    fv = [fv, sprintf('      Speed=FLOAT:0.003\n')];
    fv = [fv, sprintf('      Type=STRING:PhysicalServo\n')];
    fv = [fv, sprintf('      UserNameTSTRING=STRING:ESC Throttle\n')];
    fv = [fv, sprintf('      Volume=FLOAT:3.\n')];
    fv = [fv, sprintf('   ENDGROUP[#102]\n\n')];

    % --- BaseObject ---
    fv = [fv, sprintf('[BaseObject]\n')];
    fv = [fv, sprintf('AvailableItem=STRING:\n')];
    fv = [fv, sprintf('CGAdjustmentMTR=VECTOR3:%.4f,%.4f,%.4f\n', cg_adj_x, cg_adj_y, cg_adj_z)];
    fv = [fv, sprintf('CastShadows=BOOL:Yes\n')];
    fv = [fv, sprintf('ChildrenObjectIDs=INTARRAY:\n')];
    fv = [fv, sprintf('ForceProjectileCollisions=BOOL:No\n')];
    fv = [fv, sprintf('IsCollidable=BOOL:Yes\n')];
    fv = [fv, sprintf('Material=STRING:Foam\n')];
    fv = [fv, sprintf('NextComponentID=INT:20\n')];
    fv = [fv, sprintf('ObjectID=INT:-1\n')];
    fv = [fv, sprintf('PhysicsDimensionsOnResetMTR=VECTOR3:%.4f,%.4f,%.4f\n', ...
        rfxIn.b_m, tot_len_m, tot_ht_m)];
    fv = [fv, sprintf('PhysicsScale=FLOAT:1.\n')];
    fv = [fv, sprintf('PitchIntertia=FLOAT:%.2f\n', rfxIn.Iyy_kgm2 * 100)];
    fv = [fv, sprintf('PowerPlantType=INT:4\n')];  % 4 = electric
    fv = [fv, sprintf('RecieveShadows=BOOL:Yes\n')];
    fv = [fv, sprintf('RollIntertia=FLOAT:%.2f\n',  rfxIn.Ixx_kgm2 * 100)];
    fv = [fv, sprintf('Scale=FLOAT:1.\n')];
    fv = [fv, sprintf('StabilityModifier=FLOAT:0.80\n')];
    fv = [fv, sprintf('UserName=STRING:Nimbus\n')];
    fv = [fv, sprintf('Visible=BOOL:Yes\n')];
    fv = [fv, sprintf('YawIntertia=FLOAT:%.2f\n\n', rfxIn.Izz_kgm2 * 100)];

    % --- RootComponent ---
    fv = [fv, sprintf('[RootComponent]\n')];
    if rfxIn.has_winglets
        fv = [fv, sprintf('Children=INTARRAY:2~5~6~8~9~10~\n')];
    else
        fv = [fv, sprintf('Children=INTARRAY:2~5~6~\n')];
    end
    fv = [fv, sprintf('ComponentID=INT:1\n')];
    fv = [fv, sprintf('ComponentNameTSTRING=STRING:Airframe\n')];
    fv = [fv, sprintf('ComponentType=STRING:RootComponent\n\n')];

    %% ---- Centerbody / Fuselage (#2) ----
    cb_w = 0.12;    % [m] centerbody width
    cb_l = c_root;  % [m] centerbody length = root chord
    cb_h = tot_ht_m + 0.05;
    fv = [fv, sprintf('   SUBGROUP[#2]\n')];
    fv = [fv, sprintf('      AerodynamicsPercent=FLOAT:0.\n')];
    fv = [fv, sprintf('      AirfoilSide=STRING:NACA 0012\n')];
    fv = [fv, sprintf('      AirfoilTop=STRING:NACA 0012\n')];
    fv = [fv, sprintf('      AspectRatioFactor=FLOAT:1.\n')];
    fv = [fv, sprintf('      BreakBothAtOnce=BOOL:No\n')];
    fv = [fv, sprintf('      Children=INTARRAY:3~4~\n')];  % prop + balance weight
    fv = [fv, sprintf('      ComponentID=INT:2\n')];
    fv = [fv, sprintf('      ComponentNameTSTRING=STRING:Centerbody\n')];
    fv = [fv, sprintf('      ComponentType=STRING:Fuselage\n')];
    fv = [fv, sprintf('      DisplacementModifier_0to1=FLOAT:0.8\n')];
    fv = [fv, sprintf('      EffectVolume=FLOAT:1.\n')];
    fv = [fv, sprintf('      FrontalAreaSQMTR=FLOAT:%.6f\n', cb_w * cb_h)];
    fv = [fv, sprintf('      FrontalDragModification=FLOAT:1.\n')];
    fv = [fv, sprintf('      FuseCenterOfLift=FLOAT:0.5\n')];
    fv = [fv, sprintf('      FuseCenterOfLiftMod_Side=FLOAT:0.\n')];
    fv = [fv, sprintf('      FuseCenterOfLiftMod_Top=FLOAT:0.\n')];
    fv = [fv, sprintf('      FuseDimensionMTR=VECTOR3:%.4f,%.4f,%.4f\n', cb_w, cb_l, cb_h)];
    fv = [fv, sprintf('      LocationInParentMTR=VECTOR3:0.,0.,0.\n')];
    fv = [fv, sprintf('      OffsetMTR=VECTOR3:0.,0.,0.\n')];
    fv = [fv, sprintf('      ParasiticDragFactor=FLOAT:1.\n')];
    fv = [fv, sprintf('      SnapRollBoostFactor=FLOAT:1.\n')];
    fv = [fv, sprintf('      StallSeverity=FLOAT:0.\n')];
    fv = [fv, sprintf('      StallSeverity_000=FLOAT:1.\n')];
    fv = [fv, sprintf('      StrengthMultiplier=FLOAT:3.\n')];
    fv = [fv, sprintf('      SymetricalAboutAxis=INT:0\n')];
    fv = [fv, sprintf('      WeightKG=FLOAT:%.5f\n', W_cb_kg)];
    fv = [fv, sprintf('      WettedFuselageScale=VECTOR3:1.,1.,1.\n\n')];

    %% ---- PropellerComponent (#3, child of centerbody) ----
    fv = [fv, sprintf('      SUBGROUP[#3]\n')];
    fv = [fv, sprintf('         BackTorqueFactor=FLOAT:0.8\n')];
    fv = [fv, sprintf('         Children=INTARRAY:\n')];
    fv = [fv, sprintf('         ClockSpinsClockwiseFromRear=BOOL:Yes\n')];
    fv = [fv, sprintf('         ComponentID=INT:3\n')];
    fv = [fv, sprintf('         ComponentNameTSTRING=STRING:Motor\n')];
    fv = [fv, sprintf('         ComponentType=STRING:PropellerComponent\n')];
    fv = [fv, sprintf('         DownThrustDEG=FLOAT:0.\n')];
    fv = [fv, sprintf('         EngineToShow=STRING:None\n')];
    fv = [fv, sprintf('         GearRatio=FLOAT:1.\n')];
    fv = [fv, sprintf('         HasReversingThrottle=BOOL:No\n')];
    fv = [fv, sprintf('         HasSpeedControlBrake=BOOL:Yes\n')];
    fv = [fv, sprintf('         IsDuctedFan=BOOL:No\n')];
    fv = [fv, sprintf('         IsPusher=BOOL:%s\n', iif(rfxIn.is_pusher,'Yes','No'))];
    fv = [fv, sprintf('         LocationInParentMTR=VECTOR3:0.,%.4f,0.\n', prop_loc_y)];
    fv = [fv, sprintf('         NumBlades=INT:2\n')];
    fv = [fv, sprintf('         ParasiticDragFactor=FLOAT:1.\n')];
    fv = [fv, sprintf('         PropDiameterMTR=FLOAT:%.5f\n', D_prop_m)];
    fv = [fv, sprintf('         PropManufacturer=STRING:APC 10x47SF\n')];
    fv = [fv, sprintf('         PropPitchMTR=FLOAT:%.5f\n', pitch_m)];
    fv = [fv, sprintf('         PropVisualScale=FLOAT:1.\n')];
    fv = [fv, sprintf('         PropWashFactor=FLOAT:1.0\n')];
    fv = [fv, sprintf('         RightThrustDEG=FLOAT:0.\n')];
    fv = [fv, sprintf('         ServoThrottle=INT:102\n')];
    fv = [fv, sprintf('         ServoThrottleRev=BOOL:No\n')];
    fv = [fv, sprintf('         SnapRollBoostFactor=FLOAT:1.\n')];
    fv = [fv, sprintf('         StallSeverity=FLOAT:0.\n')];
    fv = [fv, sprintf('         StallSeverity_000=FLOAT:1.\n')];
    fv = [fv, sprintf('         SymetricalAboutAxis=INT:0\n')];
    fv = [fv, sprintf('         TorqueGeneratorElectric=STRING:%s\n', rfxIn.motor_name)];
    fv = [fv, sprintf('         TorquePercent=FLOAT:1.\n')];
    fv = [fv, sprintf('      ENDGROUP[#3]\n\n')];

    %% ---- WeightComponent for CG balance (#4, child of centerbody) ----
    % small mass at CG to fine-tune balance if needed
    fv = [fv, sprintf('      SUBGROUP[#4]\n')];
    fv = [fv, sprintf('         Children=INTARRAY:\n')];
    fv = [fv, sprintf('         ComponentID=INT:4\n')];
    fv = [fv, sprintf('         ComponentNameTSTRING=STRING:CG Ballast\n')];
    fv = [fv, sprintf('         ComponentType=STRING:WeightComponent\n')];
    fv = [fv, sprintf('         DimensionsMTR=VECTOR3:0.05,0.05,0.02\n')];
    fv = [fv, sprintf('         LocationInParentMTR=VECTOR3:0.,0.,0.\n')];
    fv = [fv, sprintf('         MassKG=FLOAT:0.\n')];
    fv = [fv, sprintf('         SymetricalAboutAxis=INT:0\n')];
    fv = [fv, sprintf('      ENDGROUP[#4]\n\n')];

    fv = [fv, sprintf('   ENDGROUP[#2]\n\n\n')];

    %% ---- BatteryStoredEnergy (#5) ----
    batt_dim_x = 0.035;
    batt_dim_z = 0.025;
    cell_str   = sprintf('LiPoly %d mah', rfxIn.cell_capacity_mah);
    fv = [fv, sprintf('   SUBGROUP[#5]\n')];
    fv = [fv, sprintf('      CellType=STRING:%s\n', cell_str)];
    fv = [fv, sprintf('      Children=INTARRAY:\n')];
    fv = [fv, sprintf('      ComponentID=INT:5\n')];
    fv = [fv, sprintf('      ComponentNameTSTRING=STRING:Main Battery\n')];
    fv = [fv, sprintf('      ComponentType=STRING:BatteryStoredEnergy\n')];
    fv = [fv, sprintf('      DimensionsMTR=VECTOR3:%.5f,%.5f,%.5f\n', ...
        batt_dim_x, rfxIn.batt_length_m, batt_dim_z)];
    fv = [fv, sprintf('      LocationInParentMTR=VECTOR3:0.,%.5f,0.\n', batt_loc_y)];
    fv = [fv, sprintf('      NumCellsInParallel=INT:%d\n', rfxIn.n_cells_parallel)];
    fv = [fv, sprintf('      NumCellsInSeries=INT:%d\n', rfxIn.n_cells_series)];
    fv = [fv, sprintf('      SpeedControllerResistanceOHMS=FLOAT:0.004\n')];
    fv = [fv, sprintf('      SymetricalAboutAxis=INT:0\n')];
    fv = [fv, sprintf('   ENDGROUP[#5]\n\n\n')];

    %% ---- Main Wing (#6, symmetric) ----
    fv = [fv, sprintf('   SUBGROUP[#6]\n')];
    fv = [fv, sprintf('      AirfoilAtRoot=STRING:%s\n', rfxIn.airfoil_name)];
    fv = [fv, sprintf('      AirfoilAtTip=STRING:%s\n',  rfxIn.airfoil_name)];
    fv = [fv, sprintf('      BreakBothAtOnce=BOOL:No\n')];
    fv = [fv, sprintf('      Children=INTARRAY:\n')];
    fv = [fv, sprintf('      ChordAtRootMTR=FLOAT:%.5f\n', c_root)];
    fv = [fv, sprintf('      ChordAtTipMTR=FLOAT:%.5f\n',  c_tip)];
    fv = [fv, sprintf('      ComponentID=INT:6\n')];
    fv = [fv, sprintf('      ComponentNameTSTRING=STRING:Main Wing\n')];
    fv = [fv, sprintf('      ComponentType=STRING:Wing\n')];
    fv = [fv, sprintf('      ControlSurfaces=INTARRAY:7~\n')];
    fv = [fv, sprintf('      DisplacementModifier_0to1=FLOAT:0.8\n')];
    fv = [fv, sprintf('      EffectVolume=FLOAT:1.\n')];
    fv = [fv, sprintf('      LeadingEdgeSweepAngleDEG=FLOAT:%.2f\n', rfxIn.sweep_LE_deg)];
    fv = [fv, sprintf('      LengthMTR=FLOAT:%.5f\n', b_half)];
    fv = [fv, sprintf('      LocationInParentMTR=VECTOR3:0.,%.5f,%.5f\n', wing_loc_y, wing_loc_z)];
    fv = [fv, sprintf('      OverallWingLift=FLOAT:1.\n')];
    fv = [fv, sprintf('      OverallWingLift_v0600=FLOAT:0.9\n')];
    fv = [fv, sprintf('      ParasiticDragFactor=FLOAT:1.\n')];
    fv = [fv, sprintf('      PostStallDragFactor=FLOAT:1.\n')];
    fv = [fv, sprintf('      PostStallLiftFactor=FLOAT:1.\n')];
    fv = [fv, sprintf('      PostStallMomentFactor=FLOAT:1.\n')];
    fv = [fv, sprintf('      RotationAboutYDEG=FLOAT:%.2f\n', rfxIn.dihedral_deg)];
    fv = [fv, sprintf('      SnapRollBoostFactor=FLOAT:1.\n')];
    fv = [fv, sprintf('      SpanLengthMTR=FLOAT:%.5f\n', b_half)];
    fv = [fv, sprintf('      StallSeverity=FLOAT:0.\n')];
    fv = [fv, sprintf('      StallSeverity_000=FLOAT:1.\n')];
    fv = [fv, sprintf('      StrengthMultiplier=FLOAT:2.\n')];
    fv = [fv, sprintf('      SubComponent=STRING:\n')];
    fv = [fv, sprintf('      SymetricalAboutAxis=INT:2\n')];  % mirror about longitudinal axis
    fv = [fv, sprintf('      WashoutAtTipDEG=FLOAT:0.\n')];
    fv = [fv, sprintf('      WeightKG=FLOAT:%.5f\n', W_wing_kg)];
    fv = [fv, sprintf('      WingEffec=FLOAT:0.85\n')];
    fv = [fv, sprintf('      WingIncedenceDEG=FLOAT:0.\n\n')];

    %% ---- Elevon control surface (#7, child of wing) ----
    fv = [fv, sprintf('      SUBGROUP[#7]\n')];
    fv = [fv, sprintf('         BlowbackFactor=FLOAT:0.\n')];
    fv = [fv, sprintf('         ControlSurfaceID=INT:7\n')];
    fv = [fv, sprintf('         ControlSurfaceNameTSTRING=STRING:Elevon\n')];
    fv = [fv, sprintf('         DeflectionCenterDEG=FLOAT:0.\n')];
    fv = [fv, sprintf('         DeflectionDownDEG=FLOAT:-%.1f\n', rfxIn.cs_deflect_dn_deg)];
    fv = [fv, sprintf('         DeflectionUpDEG=FLOAT:%.1f\n',   rfxIn.cs_deflect_up_deg)];
    fv = [fv, sprintf('         DistanceToTipMTR=FLOAT:%.5f\n',  cs_dist_tip)];
    fv = [fv, sprintf('         FrameNameMaster=STRING:<None>\n')];
    fv = [fv, sprintf('         FrameNameSlave=STRING:<None>\n')];
    fv = [fv, sprintf('         IsSpoiler=BOOL:No\n')];
    fv = [fv, sprintf('         LengthMTR=FLOAT:%.5f\n', cs_len_m)];
    fv = [fv, sprintf('         PercentOfChordTowardsRoot=FLOAT:%.3f\n', 1 - rfxIn.cs_chord_frac)];
    fv = [fv, sprintf('         PercentOfChordTowardsTip=FLOAT:%.3f\n',  1 - rfxIn.cs_chord_frac)];
    fv = [fv, sprintf('         ServeSlaveRev=BOOL:Yes\n')];
    fv = [fv, sprintf('         ServoMaster=INT:100\n')];
    fv = [fv, sprintf('         ServoMasterRev=BOOL:No\n')];
    fv = [fv, sprintf('         ServoSlave=INT:101\n')];
    fv = [fv, sprintf('         ServoSlaveRev=BOOL:Yes\n')];  % slave reverses for left elevon
    fv = [fv, sprintf('      ENDGROUP[#7]\n\n')];
    fv = [fv, sprintf('   ENDGROUP[#6]\n\n\n')];

    %% ---- Winglets (#8 right, #9 left) ----
    if rfxIn.has_winglets
        % Right winglet (#8)
        fv = [fv, sprintf('   SUBGROUP[#8]\n')];
        fv = [fv, sprintf('      AirfoilAtRoot=STRING:NACA 0009\n')];
        fv = [fv, sprintf('      AirfoilAtTip=STRING:NACA 0009\n')];
        fv = [fv, sprintf('      BreakBothAtOnce=BOOL:No\n')];
        fv = [fv, sprintf('      Children=INTARRAY:\n')];
        fv = [fv, sprintf('      ChordAtRootMTR=FLOAT:%.5f\n', rfxIn.wl_c_root_m)];
        fv = [fv, sprintf('      ChordAtTipMTR=FLOAT:%.5f\n',  rfxIn.wl_c_tip_m)];
        fv = [fv, sprintf('      ComponentID=INT:8\n')];
        fv = [fv, sprintf('      ComponentNameTSTRING=STRING:Right Winglet\n')];
        fv = [fv, sprintf('      ComponentType=STRING:Wing\n')];
        fv = [fv, sprintf('      ControlSurfaces=INTARRAY:\n')];
        fv = [fv, sprintf('      DisplacementModifier_0to1=FLOAT:0.8\n')];
        fv = [fv, sprintf('      EffectVolume=FLOAT:1.\n')];
        fv = [fv, sprintf('      LeadingEdgeSweepAngleDEG=FLOAT:%.2f\n', rfxIn.wl_sweep_LE_deg)];
        fv = [fv, sprintf('      LengthMTR=FLOAT:%.5f\n', rfxIn.wl_span_m)];
        fv = [fv, sprintf('      LocationInParentMTR=VECTOR3:%.5f,%.5f,%.5f\n', ...
            wl_loc_x, wl_loc_y, wl_loc_z)];
        fv = [fv, sprintf('      OverallWingLift=FLOAT:1.\n')];
        fv = [fv, sprintf('      ParasiticDragFactor=FLOAT:1.\n')];
        fv = [fv, sprintf('      PostStallDragFactor=FLOAT:1.\n')];
        fv = [fv, sprintf('      PostStallLiftFactor=FLOAT:1.\n')];
        fv = [fv, sprintf('      PostStallMomentFactor=FLOAT:1.\n')];
        fv = [fv, sprintf('      RotationAboutYDEG=FLOAT:90.\n')];  % vertical
        fv = [fv, sprintf('      SnapRollBoostFactor=FLOAT:1.\n')];
        fv = [fv, sprintf('      SpanLengthMTR=FLOAT:%.5f\n', rfxIn.wl_span_m)];
        fv = [fv, sprintf('      StallSeverity=FLOAT:0.\n')];
        fv = [fv, sprintf('      StallSeverity_000=FLOAT:1.\n')];
        fv = [fv, sprintf('      StrengthMultiplier=FLOAT:2.\n')];
        fv = [fv, sprintf('      SubComponent=STRING:\n')];
        fv = [fv, sprintf('      SymetricalAboutAxis=INT:0\n')];  % no mirror — explicitly placed
        fv = [fv, sprintf('      WashoutAtTipDEG=FLOAT:0.\n')];
        fv = [fv, sprintf('      WeightKG=FLOAT:0.02\n')];
        fv = [fv, sprintf('      WingEffec=FLOAT:0.8\n')];
        fv = [fv, sprintf('      WingIncedenceDEG=FLOAT:0.\n')];
        fv = [fv, sprintf('   ENDGROUP[#8]\n\n\n')];

        % Left winglet (#9) — mirror of right, negative X
        fv = [fv, sprintf('   SUBGROUP[#9]\n')];
        fv = [fv, sprintf('      AirfoilAtRoot=STRING:NACA 0009\n')];
        fv = [fv, sprintf('      AirfoilAtTip=STRING:NACA 0009\n')];
        fv = [fv, sprintf('      BreakBothAtOnce=BOOL:No\n')];
        fv = [fv, sprintf('      Children=INTARRAY:\n')];
        fv = [fv, sprintf('      ChordAtRootMTR=FLOAT:%.5f\n', rfxIn.wl_c_root_m)];
        fv = [fv, sprintf('      ChordAtTipMTR=FLOAT:%.5f\n',  rfxIn.wl_c_tip_m)];
        fv = [fv, sprintf('      ComponentID=INT:9\n')];
        fv = [fv, sprintf('      ComponentNameTSTRING=STRING:Left Winglet\n')];
        fv = [fv, sprintf('      ComponentType=STRING:Wing\n')];
        fv = [fv, sprintf('      ControlSurfaces=INTARRAY:\n')];
        fv = [fv, sprintf('      DisplacementModifier_0to1=FLOAT:0.8\n')];
        fv = [fv, sprintf('      EffectVolume=FLOAT:1.\n')];
        fv = [fv, sprintf('      LeadingEdgeSweepAngleDEG=FLOAT:%.2f\n', rfxIn.wl_sweep_LE_deg)];
        fv = [fv, sprintf('      LengthMTR=FLOAT:%.5f\n', rfxIn.wl_span_m)];
        fv = [fv, sprintf('      LocationInParentMTR=VECTOR3:%.5f,%.5f,%.5f\n', ...
            -wl_loc_x, wl_loc_y, wl_loc_z)];  % negative X = left side
        fv = [fv, sprintf('      OverallWingLift=FLOAT:1.\n')];
        fv = [fv, sprintf('      ParasiticDragFactor=FLOAT:1.\n')];
        fv = [fv, sprintf('      PostStallDragFactor=FLOAT:1.\n')];
        fv = [fv, sprintf('      PostStallLiftFactor=FLOAT:1.\n')];
        fv = [fv, sprintf('      PostStallMomentFactor=FLOAT:1.\n')];
        fv = [fv, sprintf('      RotationAboutYDEG=FLOAT:90.\n')];  % vertical
        fv = [fv, sprintf('      SnapRollBoostFactor=FLOAT:1.\n')];
        fv = [fv, sprintf('      SpanLengthMTR=FLOAT:%.5f\n', rfxIn.wl_span_m)];
        fv = [fv, sprintf('      StallSeverity=FLOAT:0.\n')];
        fv = [fv, sprintf('      StallSeverity_000=FLOAT:1.\n')];
        fv = [fv, sprintf('      StrengthMultiplier=FLOAT:2.\n')];
        fv = [fv, sprintf('      SubComponent=STRING:\n')];
        fv = [fv, sprintf('      SymetricalAboutAxis=INT:0\n')];
        fv = [fv, sprintf('      WashoutAtTipDEG=FLOAT:0.\n')];
        fv = [fv, sprintf('      WeightKG=FLOAT:0.02\n')];
        fv = [fv, sprintf('      WingEffec=FLOAT:0.8\n')];
        fv = [fv, sprintf('      WingIncedenceDEG=FLOAT:0.\n')];
        fv = [fv, sprintf('   ENDGROUP[#9]\n\n\n')];
    end

    fv = [fv, sprintf('[PlaneData]\n')];
    fv = [fv, sprintf('PLNVersion=INT:3\n\n')];
    fv = [fv, sprintf('[Transmitter]\n')];
    fv = [fv, sprintf('UseOwnRadio=BOOL:No\n')];

    %% ---- Build .bse text ----
    bse = '';
    bse = [bse, sprintf('[RenderInfo]\n')];
    bse = [bse, sprintf('DefaultColorScheme=STRING:pilot rc extra330sx_blackred\n')];
    bse = [bse, sprintf('RealFlightG3Override=STRING:\n')];
    bse = [bse, sprintf('XK_FileName=STRING:%s\n\n', [templateBase, '.kex'])];
    bse = [bse, sprintf('[Main]\n')];
    bse = [bse, sprintf('AppVersionBuild=UINT64:950038\n')];
    bse = [bse, sprintf('Version=INT:1\n\n')];
    bse = [bse, sprintf('[BaseInfo]\n')];
    bse = [bse, sprintf('KnifeEdgeOnly=BOOL:No\n')];
    bse = [bse, sprintf('Name_TSTRING=STRING:%s\n', baseName)];

    %% ---- Build archive by injecting physics into the Extra330 template ----
    % Keep ALL original filenames so g3x.enc signature stays valid.
    % Only the .rfvehicle content is replaced with Nimbus physics.
    % The aircraft will appear as "Nimbus" in RealFlight (via Name_TSTRING in .bse)
    % but use Extra330 visuals until the real CAD model is ready.
    templateRFX = fullfile(getenv('HOME'), 'Downloads', 'Pilot RC Extra330SX103_EA.RFX');

    tmpDir  = fullfile(tempdir, 'nimbus_rfx_tmp');
    if exist(tmpDir, 'dir'), rmdir(tmpDir, 's'); end
    mkdir(tmpDir);

    vehicleName  = 'Nimbus_EA';
    templateBase = 'Pilot RC Extra330SX103';

    assert(exist(templateRFX, 'file') == 2, ...
        'Template RFX not found at: %s\nRe-download the Extra330SX103 file from RealFlight.', templateRFX);

    % Extract entire Extra330 archive — keep ALL original filenames so
    % g3x.enc validation passes. Only the .rfvehicle and .bse content changes.
    unzip(templateRFX, tmpDir);

    rfvFile = [templateBase, '.rfvehicle'];
    bseFile = [templateBase, '.bse'];

    fid = fopen(fullfile(tmpDir, rfvFile), 'w', 'n', 'UTF-8');
    fprintf(fid, '%s', fv);
    fclose(fid);

    fid = fopen(fullfile(tmpDir, bseFile), 'w', 'n', 'UTF-8');
    fprintf(fid, '%s', bse);
    fclose(fid);

    % Resolve outputDir to absolute using the saved project root
    if isempty(outputDir) || outputDir(1) ~= filesep
        outputDir = fullfile(projectRoot, outputDir);
    end
    if ~exist(outputDir, 'dir'), mkdir(outputDir); end
    outRFX = fullfile(outputDir, [vehicleName, '.RFX']);
    outZIP = fullfile(outputDir, [vehicleName, '.zip']);

    % Zip everything in tmpDir — cd so entries have no path prefix
    allFiles = dir(tmpDir);
    fileList = {allFiles(~[allFiles.isdir]).name};

    cd(tmpDir);
    try
        zip(outZIP, fileList);
    catch ME
        cd(projectRoot);
        rethrow(ME);
    end
    cd(projectRoot);

    % rename .zip → .RFX
    if exist(outRFX, 'file'), delete(outRFX); end
    movefile(outZIP, outRFX);

    rmdir(tmpDir, 's');

    fprintf('\n========================================\n');
    fprintf('RFX exported successfully:\n  %s\n', outRFX);
    fprintf('========================================\n');
    fprintf('Wing area check:  S_ref = %.4f m^2 (input: %.4f m^2)\n', ...
        b_half * (c_root + c_tip), rfxIn.b_m/2 * (c_root + c_tip));
    fprintf('Total weight:     %.3f kg  (%.2f N)\n', W_total_kg, rfxIn.W_total_N);
    fprintf('CG x from LE:     %.3f m  (%.1f%% MAC)\n', ...
        rfxIn.cg_x_m, 100 * rfxIn.cg_x_m / MAC);
    fprintf('Prop:             %.0f in x %.0f in  (%.4f m x %.4f m)\n', ...
        rfxIn.D_prop_in, rfxIn.pitch_prop_in, D_prop_m, pitch_m);
    fprintf('Battery:          %dS%dP  %d mAh\n', ...
        rfxIn.n_cells_series, rfxIn.n_cells_parallel, rfxIn.cell_capacity_mah);
    fprintf('\nNOTE: No 3D visual model embedded.\n');
    fprintf('  To add visuals: place Nimbus_EA.kex in the ZIP,\n');
    fprintf('  then set XK_FileName in Nimbus_EA.bse.\n');
    fprintf('========================================\n\n');
end


%% ========================================================================
function v = iif(cond, a, b)
    if cond, v = a; else, v = b; end
end
