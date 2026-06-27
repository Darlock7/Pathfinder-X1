function prop = buildPropSurrogate_10x47SF(filename)
% buildPropSurrogate_10x45MR
% Builds thrust and torque interpolants for APC PER3_10x45MR data only.

    if nargin < 1
        filename = 'PER3_10x47SF.txt';
    end

    % Fixed geometry for this prop
    prop.name = '10x4.7SF';
    prop.filename = filename;
    prop.D_in = 10;
    prop.D_m = prop.D_in * 0.0254;

    % Read file
    txt = fileread(filename);
    lines = splitlines(string(txt));

    RPM_all = [];
    n_all   = [];
    J_all   = [];
    Q_all   = [];
    T_all   = [];

    currentRPM = NaN;

    for k = 1:length(lines)
        line = strtrim(lines(k));

        % Detect RPM header
        tokRPM = regexp(line, 'PROP RPM =\s*([0-9]+)', 'tokens');
        if ~isempty(tokRPM)
            currentRPM = str2double(tokRPM{1}{1});
            continue;
        end

        if isnan(currentRPM)
            continue;
        end

        nums = sscanf(line, '%f');

        % APC row: 2 = J, 10 = Torque (N*m), 11 = Thrust (N)
        if numel(nums) >= 11
            J = nums(2);
            Q = nums(10);
            T = nums(11);

            if isfinite(J) && isfinite(Q) && isfinite(T)
                RPM_all(end+1,1) = currentRPM;
                n_all(end+1,1)   = currentRPM / 60;   % rev/s
                J_all(end+1,1)   = J;
                Q_all(end+1,1)   = Q;
                T_all(end+1,1)   = T;
            end
        end
    end

    % Save raw data
    prop.RPM = RPM_all;
    prop.n   = n_all;
    prop.J   = J_all;
    prop.Q   = Q_all;
    prop.T   = T_all;

    % Interpolants
    prop.F_Q = scatteredInterpolant(n_all, J_all, Q_all, 'natural', 'nearest');
    prop.F_T = scatteredInterpolant(n_all, J_all, T_all, 'natural', 'nearest');
end