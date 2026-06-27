function massOut = aircraftMassProperties(massIn)
% aircraftMassProperties
%
% Purpose:
%   Compute total aircraft mass, CG, and inertia tensor about the total
%   aircraft CG from:
%       1) rigid CAD-derived bodies with known CG and Icg
%       2) discrete point masses
%
% Inputs:
%   massIn.cadBodies   : struct of rigid bodies, each with fields:
%       .name
%       .mass_kg
%       .cg_m              [1x3]
%       .Icg_kgm2          [3x3]
%
%   massIn.pointMasses : struct array with fields:
%       .name
%       .mass_kg
%       .r_m               [1x3]
%
% Outputs:
%   massOut.mass_kg
%   massOut.weight_N
%   massOut.cg_m
%   massOut.Icg_kgm2
%   massOut.massBreakdownTable

    arguments
        massIn struct
    end

    g = 9.81;   % [m/s^2]

    if ~isfield(massIn, 'cadBodies')
        massIn.cadBodies = struct();
    end
    if ~isfield(massIn, 'pointMasses')
        massIn.pointMasses = struct([]);
    end

    %% ---------------- Collect all masses for total CG ----------------
    massList = [];
    posList  = [];
    nameList = {};

    % ---- CAD rigid bodies ----
    cadNames = fieldnames(massIn.cadBodies);
    for i = 1:numel(cadNames)
        body = massIn.cadBodies.(cadNames{i});

        assert(isfield(body,'mass_kg') && isfield(body,'cg_m') && isfield(body,'Icg_kgm2'), ...
            'CAD body "%s" must contain mass_kg, cg_m, and Icg_kgm2.', cadNames{i});

        m = body.mass_kg;
        r = reshape(body.cg_m,1,3);

        massList(end+1,1) = m; %#ok<AGROW>
        posList(end+1,:)  = r; %#ok<AGROW>
        if isfield(body,'name')
            nameList{end+1,1} = body.name; %#ok<AGROW>
        else
            nameList{end+1,1} = cadNames{i}; %#ok<AGROW>
        end
    end

    % ---- Point masses ----
    if ~isempty(massIn.pointMasses)
        for i = 1:numel(massIn.pointMasses)
            pm = massIn.pointMasses(i);

            massList(end+1,1) = pm.mass_kg; %#ok<AGROW>
            posList(end+1,:)  = reshape(pm.r_m,1,3); %#ok<AGROW>
            nameList{end+1,1} = pm.name; %#ok<AGROW>
        end
    end

    if isempty(massList) || sum(massList) <= 0
        error('aircraftMassProperties:NoMass', ...
            'No valid mass contributions were provided.');
    end

    %% ---------------- Total mass and CG ----------------
    m_total = sum(massList);
    cg = sum(posList .* massList, 1) / m_total;

    %% ---------------- Total inertia about aircraft CG ----------------
    I_total = zeros(3,3);

    % ---- CAD rigid bodies: shift from body CG to aircraft CG ----
    for i = 1:numel(cadNames)
        body = massIn.cadBodies.(cadNames{i});

        m = body.mass_kg;
        r = reshape(body.cg_m,1,3) - cg;
        I_body_cg = body.Icg_kgm2;

        I_total = I_total + I_body_cg + parallelAxisTensor(m, r);
    end

    % ---- Point masses ----
    if ~isempty(massIn.pointMasses)
        for i = 1:numel(massIn.pointMasses)
            pm = massIn.pointMasses(i);

            m = pm.mass_kg;
            r = reshape(pm.r_m,1,3) - cg;

            I_total = I_total + pointMassTensor(m, r);
        end
    end

    %% ---------------- Mass breakdown table ----------------
    x = posList(:,1);
    y = posList(:,2);
    z = posList(:,3);
    weight_N = g * massList;

    massBreakdownTable = table(nameList, massList, weight_N, x, y, z, ...
        'VariableNames', {'Name','Mass_kg','Weight_N','x_m','y_m','z_m'});

    %% ---------------- Outputs ----------------
    massOut = struct();
    massOut.mass_kg  = m_total;
    massOut.weight_N = m_total * g;
    massOut.cg_m     = cg;
    massOut.Icg_kgm2 = I_total;
    massOut.massBreakdownTable = massBreakdownTable;
end

%% ========================================================================
function I = pointMassTensor(m, r)
% pointMassTensor
% Inertia tensor of a point mass located at r relative to the reference CG.

    x = r(1); y = r(2); z = r(3);

    I = [ m*(y^2 + z^2),   -m*x*y,         -m*x*z; ...
          -m*x*y,          m*(x^2 + z^2),  -m*y*z; ...
          -m*x*z,          -m*y*z,         m*(x^2 + y^2) ];
end

%% ========================================================================
function I = parallelAxisTensor(m, r)
% parallelAxisTensor
% Parallel-axis shift tensor for a rigid body shifted by vector r.

    x = r(1); y = r(2); z = r(3);

    I = [ m*(y^2 + z^2),   -m*x*y,         -m*x*z; ...
          -m*x*y,          m*(x^2 + z^2),  -m*y*z; ...
          -m*x*z,          -m*y*z,         m*(x^2 + y^2) ];
end