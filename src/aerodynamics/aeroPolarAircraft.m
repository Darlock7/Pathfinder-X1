function aeroOut = aeroPolarAircraft(aeroIn)
% aeroPolarAircraft
%
% Purpose:
%   Build a first-pass aircraft drag build-up and aircraft aerodynamic polar
%   using current values from the main sizing script.
%
% Outputs:
%   1) Drag build-up
%   2) C_L vs alpha
%   3) C_L vs C_D
%   4) L/D vs alpha
%
% Notes:
%   - Uses average root/tip airfoil data for first-pass aircraft lift curve
%   - Uses lecture-style polar: CD = CD0 + CL^2/(pi*e*AR)
%   - Cruise condition is based on required aircraft lift:
%         CL_cruise = W / (q*S)

%% ---------------- Required inputs ----------------
rho         = aeroIn.rho_kgm3;       % [kg/m^3]
mu          = aeroIn.mu_Pas;         % [Pa*s]
V           = aeroIn.V_cruise_mps;   % [m/s]
W           = aeroIn.W_N;            % [N]

Sref        = aeroIn.Sref_m2;        % [m^2]
AR          = aeroIn.AR;             % [-]
e           = aeroIn.e;              % [-]
MAC         = aeroIn.MAC_m;          % [m]
sweepC4_deg = aeroIn.sweepC4_deg;    % [deg]

Cla_root    = aeroIn.Cla_root_per_deg;     % [1/deg]
Cla_tip     = aeroIn.Cla_tip_per_deg;      % [1/deg]
aL0_root    = aeroIn.alphaL0_root_deg;     % [deg]
aL0_tip     = aeroIn.alphaL0_tip_deg;      % [deg]
Clmax_root  = aeroIn.Clmax_root;           % [-]
Clmax_tip   = aeroIn.Clmax_tip;            % [-]

useDragBuildUp = aeroIn.useDragBuildUp;
CD0_user       = aeroIn.CD0_user;

alpha_deg = aeroIn.alpha_vec_deg(:).';

plotFigures = aeroIn.plotFigures;

%% ---------------- Derived averages ----------------
Cla_avg    = 0.5 * (Cla_root + Cla_tip);       % [1/deg]
aL0_avg    = 0.5 * (aL0_root + aL0_tip);       % [deg]
Clmax_avg  = 0.5 * (Clmax_root + Clmax_tip);   % [-]

%% ---------------- Atmosphere / Reynolds ----------------
q  = 0.5 * rho * V^2;      % [Pa]
a  = 343.0;                % [m/s] first-pass speed of sound
M  = V / a;                % [-]
Re = rho * V * MAC / mu;   % [-]

%% ---------------- Skin friction ----------------
if Re < 1e5
    Cf = 1.328 / sqrt(Re);
else
    Cf = 0.455 / ((log10(Re))^2.58 * (1 + 0.144*M^2)^0.65);
end

%% ---------------- Drag build-up ----------------
if useDragBuildUp
    Swet_wing = aeroIn.Swet_wing_m2;   % [m^2]
    Swet_fuse = aeroIn.Swet_fuse_m2;   % [m^2]
    Swet_fin  = aeroIn.Swet_fin_m2;    % [m^2]

    Lf = aeroIn.Lf_m;                  % [m]
    Wf = aeroIn.Wf_m;                  % [m]
    Hf = aeroIn.Hf_m;                  % [m]

    tc = aeroIn.tc;                    % [-]
    xc = aeroIn.xc;                    % [-]

    Q_wing = aeroIn.Q_wing;            % [-]
    Q_fuse = aeroIn.Q_fuse;            % [-]
    Q_fin  = aeroIn.Q_fin;             % [-]

    % Wing / fin form factor
    FF_wing = (1 + (0.6/xc)*tc + 100*tc^4) * (1.34*M^0.18*(cosd(sweepC4_deg))^0.28);
    FF_fin  = FF_wing;

    % Body / fuselage form factor
    Amax = Wf * Hf;                        % [m^2] max cross-sectional area
    d_equiv  = sqrt((4/pi) * Amax);        % [m] equivalent circular diameter
    fineness = Lf / d_equiv;               % [-] fineness ratio f = l/d

    FF_fuse = 1 + 60/(fineness^3) + fineness/400;

    % Parasite drag contributions
    CD0_wing = Cf * FF_wing * Q_wing * Swet_wing / Sref;
    CD0_fuse = Cf * FF_fuse * Q_fuse * Swet_fuse / Sref;
    CD0_fin  = Cf * FF_fin  * Q_fin  * Swet_fin  / Sref;

    CD0 = CD0_wing + CD0_fuse + CD0_fin;
else
    CD0_wing = NaN;
    CD0_fuse = NaN;
    CD0_fin  = NaN;
    CD0      = CD0_user;
end

%% ---------------- Finite-wing lift slope ----------------
% Uses same style you were already using in your earlier script
CLalpha_3D = Cla_avg / (1 + (57.3 * Cla_avg) / (pi * e * AR));   % [1/deg]

%% ---------------- First-pass 3D CLmax correction ----------------
CLmax_3D = 0.90 * Clmax_avg * cosd(sweepC4_deg);   % 3D correction: 0.9 factor + swept-wing CLmax reduction

%% ---------------- Aircraft lift and drag curves ----------------
CL_linear = CLalpha_3D * (alpha_deg - aL0_avg);

% Cap lift after stall for first-pass plotting
CL = min(CL_linear, CLmax_3D);

k = 1 / (pi * e * AR);
CD = CD0 + k * CL.^2;
LD = CL ./ CD;

%% ---------------- Cruise operating point ----------------
CL_cruise = W / (q * Sref);
alpha_cruise_deg = aL0_avg + CL_cruise / CLalpha_3D;
CD_cruise = CD0 + k * CL_cruise^2;
LD_cruise = CL_cruise / CD_cruise;

%% ---------------- Max L/D ----------------
[LD_max, idxLD] = max(LD);
CL_at_LDmax = CL(idxLD);
CD_at_LDmax = CD(idxLD);

%% ---------------- Stall estimate ----------------
alpha_stall_deg = aL0_avg + CLmax_3D / CLalpha_3D;

%% ---------------- Plots ----------------
if plotFigures
    % ---------- Drag build-up ----------
    if useDragBuildUp
        figure('Name','Drag Build-Up','NumberTitle','off');
        bar([CD0_wing, CD0_fuse, CD0_fin]);
        set(gca, 'XTickLabel', {'Wing','Body','Fins'});
        xlabel('Drag component');
        ylabel('C_{D0} contribution  [-]');
        title('Parasite Drag Build-Up');
        grid on;
    end

    % ---------- CL vs alpha ----------
    figure('Name','Lift Curve (CL vs Alpha)','NumberTitle','off');
    plot(alpha_deg, CL, 'k', 'LineWidth', 2); hold on;
    plot(alpha_cruise_deg, CL_cruise, 'o', 'LineWidth', 1.5, 'MarkerSize', 8);
    grid on;

    xlabel('\alpha (deg)');
    ylabel('C_L');
    title('Lift Coefficient (C_L) versus Angle of Attack (\alpha)');

    txt1 = sprintf('V = %.2f m/s', V);
    txt2 = sprintf('\\alpha = %.2f deg', alpha_cruise_deg);
    txt3 = sprintf('C_L = %.3f', CL_cruise);

    text(alpha_cruise_deg + 0.7, CL_cruise + 0.08, {txt1, txt2, txt3}, 'FontSize', 11);

    xline(alpha_stall_deg, '--', 'LineWidth', 1.0);
    yline(CLmax_3D, '--', 'LineWidth', 1.0);

    legend('Aircraft lift curve', 'Cruise condition', '\alpha_{stall}', 'C_{L,max}', ...
        'Location', 'best');

    % ---------- CL vs CD ----------
    figure('Name','Drag Polar (CL vs CD)','NumberTitle','off');
    plot(CD, CL, 'k', 'LineWidth', 2); hold on;
    plot(CD_cruise, CL_cruise, '^', 'LineWidth', 1.5, 'MarkerSize', 8);

    % Tangent from origin to (L/D)max point
    CDline = linspace(0, max([CD, CD_cruise])*1.15, 200);
    CLline = LD_max * CDline;
    plot(CDline, CLline, '--', 'LineWidth', 1.2);

    % CD0 point
    plot(CD0, 0, 's', 'LineWidth', 1.2, 'MarkerSize', 7);

    grid on;
    xlabel('C_D');
    ylabel('C_L');
    title('Lift Coefficient (C_L) versus Drag Coefficient (C_D)');

    txt4 = sprintf('V = %.2f m/s', V);
    txt5 = sprintf('\\alpha = %.2f deg', alpha_cruise_deg);
    text(CD_cruise + 0.004, CL_cruise + 0.05, {txt4, txt5}, 'FontSize', 11);

    txt6 = sprintf('Slope = %.2f   (L/D)_{max} = %.2f', LD_max, LD_max);
    text(max(CD0*1.5, 0.01), max(CL_at_LDmax*0.90, 0.15), txt6, 'FontSize', 11);

    txt7 = sprintf('C_{D0} = %.4f', CD0);
    text(CD0 + 0.004, 0.05, txt7, 'FontSize', 11);

    legend('Aircraft drag polar', 'Cruise condition', 'Origin tangent', 'C_{D0}', ...
        'Location', 'best');

    % ---------- L/D vs alpha ----------
    figure('Name','Lift-to-Drag vs Alpha','NumberTitle','off');
    plot(alpha_deg, LD, 'k', 'LineWidth', 2); hold on;
    plot(alpha_cruise_deg, LD_cruise, 'o', 'LineWidth', 1.5, 'MarkerSize', 8);
    grid on;
    xlabel('\alpha (deg)');
    ylabel('L/D [-]');
    title('Aircraft L/D versus Angle of Attack (\alpha)');
    legend('Aircraft L/D', 'Cruise condition', 'Location', 'best');
end

%% ---------------- Output structure ----------------
aeroOut = struct();

aeroOut.Re                   = Re;
aeroOut.Cf                   = Cf;

aeroOut.CD0_wing             = CD0_wing;
aeroOut.CD0_fuse             = CD0_fuse;
aeroOut.CD0_fin              = CD0_fin;
aeroOut.CD0                  = CD0;

aeroOut.CLalpha_2D_avg_perDeg = Cla_avg;
aeroOut.CLalpha_3D_perDeg     = CLalpha_3D;
aeroOut.alphaL0_avg_deg       = aL0_avg;

aeroOut.CLmax_2D_avg         = Clmax_avg;
aeroOut.CLmax_3D             = CLmax_3D;
aeroOut.alpha_stall_deg      = alpha_stall_deg;

aeroOut.alpha_deg            = alpha_deg;
aeroOut.CL                   = CL;
aeroOut.CD                   = CD;
aeroOut.LD                   = LD;

aeroOut.CL_cruise            = CL_cruise;
aeroOut.alpha_cruise_deg     = alpha_cruise_deg;
aeroOut.CD_cruise            = CD_cruise;
aeroOut.LD_cruise            = LD_cruise;

aeroOut.LD_max               = LD_max;
aeroOut.CL_at_LDmax          = CL_at_LDmax;
aeroOut.CD_at_LDmax          = CD_at_LDmax;

end