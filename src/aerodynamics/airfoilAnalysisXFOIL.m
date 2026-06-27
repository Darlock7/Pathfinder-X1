function airfoilOut = airfoilAnalysisXFOIL(airfoilIn)
% airfoilAnalysisXFOIL
%
% Runs XFOIL for one root airfoil and one tip airfoil.
% Supports:
%   - NACA airfoils, e.g. 'NACA0012'
%   - coordinate files, e.g. 'mh60.dat'
%
% OS logic:
%   - Windows -> xfoilWindows.exe or xfoilWindows
%   - Mac     -> xfoilMAC or xfoilMAC.exe
%
% Recommended first test:
%   rootFoil = 'NACA0012';
%   tipFoil  = 'NACA0009';
%
% Required inputs:
%   airfoilIn.rootFoil
%   airfoilIn.tipFoil
%   airfoilIn.Re_root
%   airfoilIn.Re_tip
%   airfoilIn.alpha_deg
%
% Optional inputs:
%   airfoilIn.xfoilFolder   = '.'
%   airfoilIn.Mach          = 0.0
%   airfoilIn.maxIter       = 150
%   airfoilIn.cleanupFiles  = true
%   airfoilIn.printSummary  = false
%
% Outputs:
%   airfoilOut.root
%   airfoilOut.tip
%   airfoilOut.xfoilExe

    arguments
        airfoilIn struct
    end

    %% ---------------- Defaults ----------------
    if ~isfield(airfoilIn, 'xfoilFolder') || isempty(airfoilIn.xfoilFolder)
        airfoilIn.xfoilFolder = '.';
    end
    if ~isfield(airfoilIn, 'Mach') || isempty(airfoilIn.Mach)
        airfoilIn.Mach = 0.0;
    end
    if ~isfield(airfoilIn, 'maxIter') || isempty(airfoilIn.maxIter)
        airfoilIn.maxIter = 150;
    end
    if ~isfield(airfoilIn, 'cleanupFiles') || isempty(airfoilIn.cleanupFiles)
        airfoilIn.cleanupFiles = true;
    end
    if ~isfield(airfoilIn, 'printSummary') || isempty(airfoilIn.printSummary)
        airfoilIn.printSummary = false;
    end

    %% ---------------- Required input checks ----------------
    requiredFields = {'rootFoil','tipFoil','Re_root','Re_tip','alpha_deg'};
    for k = 1:numel(requiredFields)
        thisField = requiredFields{k};
        if ~isfield(airfoilIn, thisField)
            error('airfoilAnalysisXFOIL:MissingField', ...
                'Missing required input field: %s', thisField);
        end
    end

    alphaVec = airfoilIn.alpha_deg(:);
    if isempty(alphaVec)
        error('airfoilAnalysisXFOIL:EmptyAlpha', ...
            'airfoilIn.alpha_deg must not be empty.');
    end

    %% ---------------- Select XFOIL executable ----------------
    xfoilExe = localGetXFOILExecutable(airfoilIn.xfoilFolder);

    %% ---------------- Run root and tip airfoils ----------------
    rootStruct = localRunOneAirfoil( ...
        char(airfoilIn.rootFoil), ...
        airfoilIn.Re_root, ...
        airfoilIn.Mach, ...
        alphaVec, ...
        airfoilIn.maxIter, ...
        xfoilExe, ...
        airfoilIn.cleanupFiles);

    tipStruct = localRunOneAirfoil( ...
        char(airfoilIn.tipFoil), ...
        airfoilIn.Re_tip, ...
        airfoilIn.Mach, ...
        alphaVec, ...
        airfoilIn.maxIter, ...
        xfoilExe, ...
        airfoilIn.cleanupFiles);

    %% ---------------- Package output ----------------
    airfoilOut = struct();
    airfoilOut.root = rootStruct;
    airfoilOut.tip  = tipStruct;
    airfoilOut.xfoilExe = xfoilExe;
    airfoilOut.aeroTwist_deg = airfoilOut.root.alphaL0_deg - airfoilOut.tip.alphaL0_deg;

    if airfoilIn.printSummary
        fprintf('\n================ XFOIL Airfoil Analysis Summary ================\n');
        fprintf('XFOIL executable used     = %s\n', xfoilExe);
        fprintf('Root foil                 = %s\n', airfoilOut.root.name);
        fprintf('Tip foil                  = %s\n', airfoilOut.tip.name);
        fprintf('Root valid polar points   = %d\n', numel(airfoilOut.root.alpha_deg));
        fprintf('Tip valid polar points    = %d\n', numel(airfoilOut.tip.alpha_deg));
        fprintf('Aerodynamic twist         = %.3f deg\n', airfoilOut.aeroTwist_deg);
        fprintf('===============================================================\n\n');
    end
end


%% ========================================================================
function xfoilExe = localGetXFOILExecutable(xfoilFolder)
% Select the proper XFOIL executable based on OS.

    if ispc
        exeCandidates = {'xfoilWindows.exe', 'xfoilWindows'};
    elseif ismac
        exeCandidates = {'xfoilMAC', 'xfoilMAC.exe'};
    else
        error('airfoilAnalysisXFOIL:UnsupportedOS', ...
            'This version currently supports only Windows and Mac.');
    end

    xfoilExe = '';
    for i = 1:numel(exeCandidates)
        candidate = fullfile(xfoilFolder, exeCandidates{i});
        if exist(candidate, 'file') == 2
            xfoilExe = candidate;
            break;
        end
    end

    if isempty(xfoilExe)
        error('airfoilAnalysisXFOIL:XFOILNotFound', ...
            'Could not find XFOIL executable in folder: %s', xfoilFolder);
    end
end


%% ========================================================================
function out = localRunOneAirfoil(foilName, Re, Mach, alphaVec, maxIter, xfoilExe, cleanupFiles)
% Run XFOIL for one airfoil and parse the resulting polar.

    % Unique filenames in current working directory
    runTag = char(java.util.UUID.randomUUID);
    runTag = regexprep(runTag, '-', '_');

    inpFileName   = ['xfoil_input_' runTag '.inp'];
    polarFileName = ['xfoil_polar_' runTag '.txt'];

    inpFile   = fullfile(pwd, inpFileName);
    polarFile = fullfile(pwd, polarFileName);

    % Remove stale files if somehow present
    localDeleteIfExists(inpFile);
    localDeleteIfExists(polarFile);

    % Identify airfoil type
    foilNameTrim = strtrim(foilName);
    isNACA = startsWith(upper(foilNameTrim), 'NACA');

    % If not NACA, require coordinate file to exist
    if ~isNACA
        if exist(foilNameTrim, 'file') ~= 2
            error('airfoilAnalysisXFOIL:AirfoilFileMissing', ...
                'Could not find coordinate file: %s', foilNameTrim);
        end
    end

    % Write XFOIL input file
    fid = fopen(inpFile, 'w');
    if fid == -1
        error('airfoilAnalysisXFOIL:InputFileCreateFailed', ...
            'Could not create temporary XFOIL input file.');
    end

    try
        % -------- Disable graphics (Mac/Linux headless) --------
        if ~ispc
            fprintf(fid, 'PLOP\n');
            fprintf(fid, 'G\n');
            fprintf(fid, '\n');
        end

        % -------- Load airfoil --------
        if isNACA
            nacaDigits = strtrim(foilNameTrim(5:end));
            fprintf(fid, 'NACA %s\n', nacaDigits);
        else
            fprintf(fid, 'LOAD %s\n', foilNameTrim);
           
        end

        % -------- Panel / operating menu --------
        fprintf(fid, 'PANE\n');
        fprintf(fid, 'OPER\n');

        fprintf(fid, 'VISC %.8g\n', Re);
        fprintf(fid, 'ITER %d\n', maxIter);

        if Mach > 0
            fprintf(fid, 'MACH %.8g\n', Mach);
        end

        % -------- Turn polar accumulation on --------
        fprintf(fid, 'PACC\n');
        fprintf(fid, '%s\n', polarFileName);
        fprintf(fid, '\n');   % blank line = decline dump file

        % -------- Run requested alpha points --------
        for i = 1:numel(alphaVec)
            fprintf(fid, 'ALFA %.8g\n', alphaVec(i));
        end

        % -------- Turn polar accumulation off --------
        fprintf(fid, 'PACC\n');
        fprintf(fid, '\n');   % cleanly finish toggle

        % -------- Exit XFOIL --------
        fprintf(fid, 'QUIT\n');

        fclose(fid);

    catch ME
        fclose(fid);
        rethrow(ME);
    end

    % Run command
    % Run command
    if ispc
        cmd = sprintf('"%s" < "%s"', xfoilExe, inpFile);
    elseif ismac
        try
            fileattrib(xfoilExe, '+x');
        catch
        end
        cmd = sprintf('DISPLAY="" "%s" < "%s"', xfoilExe, inpFile);
    else
        % Linux
        try
            fileattrib(xfoilExe, '+x');
        catch
        end
        cmd = sprintf('DISPLAY="" "%s" < "%s"', xfoilExe, inpFile);
    end

    [status, cmdout] = system(cmd);

    % On Mac/Linux, XFOIL may return non-zero status even on success
    % (due to display issues). Check for polar file existence instead.
    if status ~= 0 && ispc
        if cleanupFiles
            localDeleteIfExists(inpFile);
            localDeleteIfExists(polarFile);
        end
        error('airfoilAnalysisXFOIL:XFOILRunFailed', ...
            'XFOIL failed for airfoil "%s".\nCommand output:\n%s', ...
            foilNameTrim, cmdout);
    end

    if exist(polarFile, 'file') ~= 2
        if cleanupFiles
            localDeleteIfExists(inpFile);
        end
        error('airfoilAnalysisXFOIL:PolarMissing', ...
            ['XFOIL ran, but no polar file was found for airfoil "%s".\n' ...
             'Expected file: %s\n' ...
             'XFOIL output:\n%s'], ...
             foilNameTrim, polarFile, cmdout);
    end

    % Parse polar file
    polar = localReadPolarFile(polarFile);

    if isempty(polar.alpha_deg)
        if cleanupFiles
            localDeleteIfExists(inpFile);
            localDeleteIfExists(polarFile);
        end
        error('airfoilAnalysisXFOIL:NoPolarData', ...
            'Polar file was created, but no valid data rows were parsed for airfoil "%s".', ...
            foilNameTrim);
    end

    % Remove exact duplicate alpha rows if they exist
    [alphaUnique, ia] = unique(polar.alpha_deg, 'stable');

    polar.alpha_deg = alphaUnique;
    polar.CL        = polar.CL(ia);
    polar.CD        = polar.CD(ia);
    polar.CDp       = polar.CDp(ia);
    polar.CM        = polar.CM(ia);
    polar.Top_Xtr   = polar.Top_Xtr(ia);
    polar.Bot_Xtr   = polar.Bot_Xtr(ia);

    % Extract summary metrics
    metrics = localExtractMetrics(polar);

    % Build output struct
    out = struct();
    out.name         = foilNameTrim;
    out.Re           = Re;
    out.Mach         = Mach;

    out.alpha_deg    = polar.alpha_deg;
    out.CL           = polar.CL;
    out.CD           = polar.CD;
    out.CDp          = polar.CDp;
    out.CM           = polar.CM;
    out.Top_Xtr      = polar.Top_Xtr;
    out.Bot_Xtr      = polar.Bot_Xtr;

    out.Cla_per_deg  = metrics.Cla_per_deg;
    out.alphaL0_deg  = metrics.alphaL0_deg;
    out.Cm0          = metrics.Cm0;
    out.Cl_max       = metrics.Cl_max;
    out.bestLD       = metrics.bestLD;
    out.alpha_bestLD = metrics.alpha_bestLD;

    % Cleanup
    if cleanupFiles
        localDeleteIfExists(inpFile);
        localDeleteIfExists(polarFile);
    end
end


%% ========================================================================
function polar = localReadPolarFile(polarFile)
% Read XFOIL polar file.
%
% Expected columns:
%   alpha   CL   CD   CDp   CM   Top_Xtr   Bot_Xtr

    fid = fopen(polarFile, 'r');
    if fid == -1
        error('airfoilAnalysisXFOIL:PolarOpenFailed', ...
            'Could not open polar file: %s', polarFile);
    end

    alpha_deg = [];
    CL = [];
    CD = [];
    CDp = [];
    CM = [];
    Top_Xtr = [];
    Bot_Xtr = [];

    try
        while ~feof(fid)
            line = fgetl(fid);
            if ~ischar(line)
                continue;
            end

            vals = sscanf(line, '%f');
            if numel(vals) >= 7
                alpha_deg(end+1,1) = vals(1); %#ok<AGROW>
                CL(end+1,1)        = vals(2); %#ok<AGROW>
                CD(end+1,1)        = vals(3); %#ok<AGROW>
                CDp(end+1,1)       = vals(4); %#ok<AGROW>
                CM(end+1,1)        = vals(5); %#ok<AGROW>
                Top_Xtr(end+1,1)   = vals(6); %#ok<AGROW>
                Bot_Xtr(end+1,1)   = vals(7); %#ok<AGROW>
            end
        end
        fclose(fid);
    catch ME
        fclose(fid);
        rethrow(ME);
    end

    polar = struct();
    polar.alpha_deg = alpha_deg;
    polar.CL        = CL;
    polar.CD        = CD;
    polar.CDp       = CDp;
    polar.CM        = CM;
    polar.Top_Xtr   = Top_Xtr;
    polar.Bot_Xtr   = Bot_Xtr;
end


%% ========================================================================
function metrics = localExtractMetrics(polar)
% Extract simple first-pass metrics from polar data.

    alpha = polar.alpha_deg(:);
    CL    = polar.CL(:);
    CD    = polar.CD(:);
    CM    = polar.CM(:);

    valid = isfinite(alpha) & isfinite(CL) & isfinite(CD) & isfinite(CM) & (CD > 0);
    alpha = alpha(valid);
    CL    = CL(valid);
    CD    = CD(valid);
    CM    = CM(valid);

    if isempty(alpha)
        metrics = struct( ...
            'Cla_per_deg', NaN, ...
            'alphaL0_deg', NaN, ...
            'Cm0', NaN, ...
            'Cl_max', NaN, ...
            'bestLD', NaN, ...
            'alpha_bestLD', NaN);
        return;
    end

    % Linear-region estimate
    linMask = (alpha >= -2) & (alpha <= 4);
    if nnz(linMask) >= 2
        pCL = polyfit(alpha(linMask), CL(linMask), 1);
        Cla_per_deg = pCL(1);

        if abs(pCL(1)) > eps
            alphaL0_deg = -pCL(2) / pCL(1);
        else
            alphaL0_deg = NaN;
        end

        pCM = polyfit(alpha(linMask), CM(linMask), 1);
        Cm0 = polyval(pCM, 0);
    else
        Cla_per_deg = NaN;
        alphaL0_deg = NaN;
        Cm0 = NaN;
    end

    [Cl_max, ~] = max(CL);

    LD = CL ./ CD;
    [bestLD, idxBest] = max(LD);
    alpha_bestLD = alpha(idxBest);

    metrics = struct();
    metrics.Cla_per_deg  = Cla_per_deg;
    metrics.alphaL0_deg  = alphaL0_deg;
    metrics.Cm0          = Cm0;
    metrics.Cl_max       = Cl_max;
    metrics.bestLD       = bestLD;
    metrics.alpha_bestLD = alpha_bestLD;
end


%% ========================================================================
function localDeleteIfExists(fname)
    if exist(fname, 'file') == 2
        delete(fname);
    end
end